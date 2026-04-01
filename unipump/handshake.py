#!/usr/bin/env python3
"""UNIMEP handshake tester for Raspberry Pi + USB-RS485.

This script combines:
- `UnimepProtocol.pdf` for the application transactions
- `unimep_prot_summary .pdf` for the line framing and poll/ack flow

Important protocol details from the summary examples:
- short messages:
  - POLL      = ADR, 20, FA
  - ACK       = ADR, CX, FA
  - ACK_POLL  = ADR, EX, FA
  - EOT       = ADR, 70, FA
- long messages:
  - ADR, 3X, TYPE, LEN, DATA..., CRC_LO, CRC_HI, 03, FA
- CRC is CRC-16/IBM reflected (poly 0xA001) with init=0x0000
- a long controller command gets a short ACK from the pump
- the controller then uses POLL to fetch the actual data/status packet
"""

from __future__ import annotations

import argparse
import sys
import time
from typing import Iterable

import serial

try:
    from serial.rs485 import RS485Settings
except ImportError:  # pragma: no cover - depends on pyserial build/platform
    RS485Settings = None


STATUS_NAMES = {
    0x00: "PUMP NOT PROGRAMMED",
    0x01: "RESET",
    0x02: "AUTHORIZED",
    0x04: "FILLING",
    0x05: "FILLING COMPLETED",
    0x06: "MAX AMOUNT/VOLUME REACHED",
    0x07: "SWITCHED OFF",
    0x0B: "PAUSED",
}

ETX = 0x03
SF = 0xFA
DLE = 0x10
COMMON_SERIAL_PROFILES = (
    (9600, 8, "N", 1),
    (9600, 7, "E", 1),
    (4800, 8, "N", 1),
    (4800, 7, "E", 1),
    (2400, 8, "N", 1),
    (2400, 7, "E", 1),
)


def crc16_dart(data: bytes) -> int:
    """CRC-16/IBM reflected with init=0x0000, per summary examples."""
    crc = 0x0000
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def crc16_x25(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0x8408
            else:
                crc >>= 1
    return (~crc) & 0xFFFF


CRC_FUNCTIONS = {
    "dart": crc16_dart,
    "modbus": crc16_modbus,
    "ccitt-false": crc16_ccitt_false,
    "x25": crc16_x25,
    "none": lambda _data: 0,
}


def hexdump(data: bytes) -> str:
    return " ".join(f"{byte:02X}" for byte in data)


def packed_bcd_to_int(data: Iterable[int]) -> int:
    digits = []
    for byte in data:
        hi = (byte >> 4) & 0x0F
        lo = byte & 0x0F
        if hi > 9 or lo > 9:
            raise ValueError(f"invalid packed BCD byte: 0x{byte:02X}")
        digits.append(str(hi))
        digits.append(str(lo))
    return int("".join(digits))


def format_bcd_value(data: bytes, decimal_places: int) -> str:
    value = packed_bcd_to_int(data)
    if decimal_places <= 0:
        return str(value)
    scale = 10**decimal_places
    whole = value // scale
    frac = value % scale
    return f"{whole}.{frac:0{decimal_places}d}"


def build_return_status_transaction() -> bytes:
    # CD1, len=1, DCC=0x00 (RETURN STATUS)
    return bytes([0x01, 0x01, 0x00])


def stuff_crc_bytes(crc_lo: int, crc_hi: int) -> bytes:
    stuffed = bytearray()
    for byte in (crc_lo, crc_hi):
        if byte == SF:
            stuffed.append(DLE)
        stuffed.append(byte)
    return bytes(stuffed)


def build_line_frame(
    pump_address: int,
    payload: bytes,
    sequence: int,
    crc_name: str,
) -> bytes:
    ctrl_seq = 0x30 | (sequence & 0x0F)
    body = bytes([pump_address, ctrl_seq]) + payload
    crc_value = CRC_FUNCTIONS[crc_name](body)
    crc_lo = crc_value & 0xFF
    crc_hi = (crc_value >> 8) & 0xFF
    return body + stuff_crc_bytes(crc_lo, crc_hi) + bytes([ETX, SF])


def build_poll(pump_address: int) -> bytes:
    return bytes([pump_address, 0x20, SF])


def build_ack(pump_address: int, sequence: int) -> bytes:
    return bytes([pump_address, 0xC0 | (sequence & 0x0F), SF])


def parse_transactions(payload: bytes) -> list[str]:
    messages: list[str] = []
    idx = 0
    while idx + 2 <= len(payload):
        trans = payload[idx]
        data_len = payload[idx + 1]
        start = idx + 2
        end = start + data_len
        if end > len(payload):
            messages.append(
                f"incomplete transaction 0x{trans:02X}: need {data_len} bytes, "
                f"have {len(payload) - start}"
            )
            break

        data = payload[start:end]
        if trans == 0x01 and data_len == 1:
            status = data[0]
            messages.append(
                f"DC1 Pump status: 0x{status:02X} "
                f"({STATUS_NAMES.get(status, 'UNKNOWN')})"
            )
        elif trans == 0x02 and data_len == 8:
            volume = format_bcd_value(data[:4], decimal_places=2)
            amount = format_bcd_value(data[4:], decimal_places=2)
            messages.append(f"DC2 Filled volume/amount: volume={volume}, amount={amount}")
        elif trans == 0x03 and data_len == 4:
            price = format_bcd_value(data[:3], decimal_places=3)
            nozio = data[3]
            nozzle = nozio & 0x0F
            nozzle_out = bool(nozio & 0x10)
            messages.append(
                "DC3 Nozzle status/filling price: "
                f"price={price}, nozzle={nozzle}, nozzle_state={'out' if nozzle_out else 'in'}"
            )
        elif trans == 0x65 and data_len == 11:
            nozzle = data[0]
            total_volume = format_bcd_value(data[1:6], decimal_places=2)
            total_grade_1 = format_bcd_value(data[6:11], decimal_places=2)
            messages.append(
                "DC101 Volume total counters: "
                f"nozzle={nozzle}, total_volume={total_volume}, total_grade_1={total_grade_1}"
            )
        else:
            messages.append(
                f"unknown transaction 0x{trans:02X} len={data_len}: {hexdump(data)}"
            )

        idx = end

    if idx < len(payload):
        messages.append(f"trailing bytes: {hexdump(payload[idx:])}")

    return messages


def extract_payload_from_reply(reply: bytes) -> bytes:
    if len(reply) >= 4 and reply[-2:] == bytes([ETX, SF]):
        # Long frame: ADR, CTRL/SEQ, PAYLOAD..., CRC_LO, CRC_HI, ETX, SF
        if len(reply) >= 6:
            body = reply[:-2].replace(bytes([DLE, SF]), bytes([SF]))
            return body[2:-2]
        return b""
    # Fallback: parse whole buffer as transactions if the device returns raw payload.
    return reply


def describe_short_message(reply: bytes) -> str | None:
    if len(reply) != 3 or reply[-1] != SF:
        return None
    adr, msg_type, _ = reply
    if msg_type == 0x20:
        return f"SHORT POLL from 0x{adr:02X}"
    if msg_type == 0x70:
        return f"SHORT EOT from 0x{adr:02X}"
    if 0xC0 <= msg_type <= 0xCF:
        return f"SHORT ACK from 0x{adr:02X}, seq={msg_type & 0x0F}"
    if 0xE0 <= msg_type <= 0xEF:
        return f"SHORT ACK_POLL from 0x{adr:02X}, seq={msg_type & 0x0F}"
    return f"SHORT unknown from 0x{adr:02X}: {hexdump(reply)}"


def get_long_frame_sequence(reply: bytes) -> int | None:
    if len(reply) >= 4 and reply[-2:] == bytes([ETX, SF]) and (reply[1] & 0xF0) == 0x30:
        return reply[1] & 0x0F
    return None


def split_stream_frames(data: bytes) -> list[bytes]:
    frames: list[bytes] = []
    idx = 0
    while idx < len(data):
        if (
            idx + 2 < len(data)
            and 0x50 <= data[idx] <= 0x6F
            and data[idx + 2] == SF
        ):
            msg_type = data[idx + 1]
            if msg_type in (0x20, 0x70) or 0xC0 <= msg_type <= 0xCF or 0xE0 <= msg_type <= 0xEF:
                frames.append(data[idx : idx + 3])
                idx += 3
                continue

        if (
            idx + 3 < len(data)
            and 0x50 <= data[idx] <= 0x6F
            and (data[idx + 1] & 0xF0) == 0x30
        ):
            end = idx + 2
            while end + 1 < len(data):
                if data[end] == ETX and data[end + 1] == SF:
                    frames.append(data[idx : end + 2])
                    idx = end + 2
                    break
                end += 1
            else:
                break
            continue

        idx += 1

    if not frames and data:
        return [data]
    return frames


def apply_rs485_mode(ser: serial.Serial, args: argparse.Namespace) -> bool:
    if not args.rs485_mode:
        return False
    if RS485Settings is None:
        print(
            "Warning: pyserial RS485Settings is not available on this system; "
            "continuing without kernel RS485 mode.",
            file=sys.stderr,
        )
        return False

    try:
        ser.rs485_mode = RS485Settings(
            rts_level_for_tx=bool(args.rs485_rts_level_tx),
            rts_level_for_rx=bool(args.rs485_rts_level_rx),
            loopback=False,
            delay_before_tx=args.rs485_delay_before_tx or None,
            delay_before_rx=args.rs485_delay_before_rx or None,
        )
        return True
    except (ValueError, OSError, AttributeError) as exc:
        print(
            "Warning: adapter/driver does not support kernel RS485 mode "
            f"({exc}); continuing without it.",
            file=sys.stderr,
        )
        return False


def open_serial_profile(
    port: str,
    baudrate: int,
    bytesize: int,
    parity: str,
    stopbits: float,
    timeout: float,
    args: argparse.Namespace,
) -> serial.Serial:
    parity_map = {
        "N": serial.PARITY_NONE,
        "E": serial.PARITY_EVEN,
        "O": serial.PARITY_ODD,
    }
    bytesize_map = {
        7: serial.SEVENBITS,
        8: serial.EIGHTBITS,
    }
    ser = serial.Serial(
        port=port,
        baudrate=baudrate,
        bytesize=bytesize_map[bytesize],
        parity=parity_map[parity],
        stopbits=stopbits,
        timeout=timeout,
        write_timeout=timeout,
    )
    apply_rs485_mode(ser, args)
    return ser


def open_serial(args: argparse.Namespace) -> serial.Serial:
    return open_serial_profile(
        port=args.port,
        baudrate=args.baudrate,
        bytesize=args.bytesize,
        parity=args.parity,
        stopbits=args.stopbits,
        timeout=args.timeout,
        args=args,
    )


def read_reply(ser: serial.Serial, timeout_seconds: float, idle_gap: float = 0.25) -> bytes:
    deadline = time.monotonic() + timeout_seconds
    received = bytearray()
    last_data_at: float | None = None
    while time.monotonic() < deadline:
        chunk = ser.read(256)
        if chunk:
            received.extend(chunk)
            last_data_at = time.monotonic()
            if received.endswith(bytes([ETX, SF])):
                break
            if last_data_at is not None and (time.monotonic() - last_data_at) >= idle_gap:
                break
        else:
            if received and last_data_at is not None and (time.monotonic() - last_data_at) >= idle_gap:
                break
            time.sleep(0.05)
    return bytes(received)


def try_handshake(
    ser: serial.Serial,
    request: bytes,
    timeout_seconds: float,
    args: argparse.Namespace,
) -> bytes:
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    if args.pre_tx_delay > 0:
        time.sleep(args.pre_tx_delay)
    ser.write(request)
    ser.flush()
    if args.post_tx_delay > 0:
        time.sleep(args.post_tx_delay)
    return read_reply(ser, timeout_seconds=timeout_seconds)


def decode_single_reply(reply: bytes) -> None:
    print(f"Raw reply: {hexdump(reply)}")
    short_desc = describe_short_message(reply)
    if short_desc is not None:
        print(f"- {short_desc}")
        return
    payload = extract_payload_from_reply(reply)
    decoded = parse_transactions(payload)
    if decoded:
        for line in decoded:
            print(f"- {line}")
    else:
        print("No decodable transactions found in reply payload.")


def decode_and_print_reply(reply: bytes) -> None:
    frames = split_stream_frames(reply)
    if len(frames) <= 1:
        decode_single_reply(reply)
        return

    for index, frame in enumerate(frames, start=1):
        print(f"[frame {index}]")
        decode_single_reply(frame)


def listen_mode(args: argparse.Namespace) -> int:
    print(
        f"Listening on {args.port} @ "
        f"{args.baudrate},{args.bytesize}{args.parity}{int(args.stopbits)} "
        f"for {args.listen_seconds:.1f}s"
    )
    try:
        with open_serial(args) as ser:
            ser.reset_input_buffer()
            reply = read_reply(ser, timeout_seconds=args.listen_seconds, idle_gap=0.5)
    except serial.SerialException as exc:
        print(f"serial error: {exc}", file=sys.stderr)
        return 1

    if not reply:
        print("No traffic observed.")
        return 1

    decode_and_print_reply(reply)
    return 0


def run_status_exchange(
    ser: serial.Serial,
    pump_address: int,
    sequence: int,
    crc_name: str,
    timeout: float,
    poll_count: int,
    args: argparse.Namespace,
) -> bytes:
    command = build_line_frame(
        pump_address=pump_address,
        payload=build_return_status_transaction(),
        sequence=sequence,
        crc_name=crc_name,
    )
    print(f"Sending RETURN STATUS: {hexdump(command)}")
    first_reply = try_handshake(ser, command, timeout_seconds=timeout, args=args)
    if not first_reply:
        return b""

    decode_and_print_reply(first_reply)

    for poll_index in range(1, poll_count + 1):
        poll = build_poll(pump_address)
        print(f"Sending POLL {poll_index}: {hexdump(poll)}")
        reply = try_handshake(ser, poll, timeout_seconds=timeout, args=args)
        if not reply:
            continue

        decode_and_print_reply(reply)
        long_seq = get_long_frame_sequence(reply)
        if long_seq is not None:
            ack = build_ack(pump_address, long_seq)
            print(f"Sending ACK for seq={long_seq}: {hexdump(ack)}")
            ser.reset_input_buffer()
            if args.pre_tx_delay > 0:
                time.sleep(args.pre_tx_delay)
            ser.write(ack)
            ser.flush()
            if args.post_tx_delay > 0:
                time.sleep(args.post_tx_delay)
            return reply

    return first_reply


def run_poll_only(
    ser: serial.Serial,
    pump_address: int,
    timeout: float,
    poll_count: int,
    args: argparse.Namespace,
) -> bytes:
    last_reply = b""
    for poll_index in range(1, poll_count + 1):
        poll = build_poll(pump_address)
        print(f"Sending POLL {poll_index}: {hexdump(poll)}")
        reply = try_handshake(ser, poll, timeout_seconds=timeout, args=args)
        if not reply:
            continue

        decode_and_print_reply(reply)
        last_reply = reply

        long_seq = get_long_frame_sequence(reply)
        if long_seq is not None:
            ack = build_ack(pump_address, long_seq)
            print(f"Sending ACK for seq={long_seq}: {hexdump(ack)}")
            ser.reset_input_buffer()
            if args.pre_tx_delay > 0:
                time.sleep(args.pre_tx_delay)
            ser.write(ack)
            ser.flush()
            if args.post_tx_delay > 0:
                time.sleep(args.post_tx_delay)

    return last_reply


def run_pts_style_replay(
    ser: serial.Serial,
    pump_address: int,
    sequence: int,
    timeout: float,
    args: argparse.Namespace,
) -> bytes:
    last_reply = b""

    print("PTS replay step 1: initial POLL")
    first_poll_reply = try_handshake(
        ser,
        build_poll(pump_address),
        timeout_seconds=timeout,
        args=args,
    )
    if first_poll_reply:
        decode_and_print_reply(first_poll_reply)
        last_reply = first_poll_reply
        long_seq = get_long_frame_sequence(first_poll_reply)
        if long_seq is not None:
            ack = build_ack(pump_address, long_seq)
            print(f"Sending ACK for seq={long_seq}: {hexdump(ack)}")
            if args.pre_tx_delay > 0:
                time.sleep(args.pre_tx_delay)
            ser.write(ack)
            ser.flush()
            if args.post_tx_delay > 0:
                time.sleep(args.post_tx_delay)

    if args.replay_step_delay > 0:
        time.sleep(args.replay_step_delay)

    command = build_line_frame(
        pump_address=pump_address,
        payload=build_return_status_transaction(),
        sequence=sequence,
        crc_name="dart",
    )
    print(f"PTS replay step 2: RETURN STATUS seq={sequence}: {hexdump(command)}")
    command_reply = try_handshake(
        ser,
        command,
        timeout_seconds=timeout,
        args=args,
    )
    if command_reply:
        decode_and_print_reply(command_reply)
        last_reply = command_reply

    if args.replay_step_delay > 0:
        time.sleep(args.replay_step_delay)

    print("PTS replay step 3: follow-up POLL")
    poll_reply = try_handshake(
        ser,
        build_poll(pump_address),
        timeout_seconds=timeout,
        args=args,
    )
    if not poll_reply:
        return last_reply

    decode_and_print_reply(poll_reply)
    last_reply = poll_reply
    long_seq = get_long_frame_sequence(poll_reply)
    if long_seq is not None:
        ack = build_ack(pump_address, long_seq)
        print(f"PTS replay step 4: ACK seq={long_seq}: {hexdump(ack)}")
        if args.pre_tx_delay > 0:
            time.sleep(args.pre_tx_delay)
        ser.write(ack)
        ser.flush()
        if args.post_tx_delay > 0:
            time.sleep(args.post_tx_delay)

    return last_reply


def probe_with_serial_profile(
    port: str,
    baudrate: int,
    bytesize: int,
    parity: str,
    stopbits: float,
    timeout: float,
    transaction: bytes,
    pump_address: int,
    args: argparse.Namespace,
) -> bytes:
    print(f"Opening {port} @ {baudrate},{bytesize}{parity}{int(stopbits)}")
    if args.rs485_mode:
        print(
            "RS485 mode requested "
            f"(rts_tx={args.rs485_rts_level_tx}, "
            f"rts_rx={args.rs485_rts_level_rx}, "
            f"delay_before_tx={args.rs485_delay_before_tx}, "
            f"delay_before_rx={args.rs485_delay_before_rx})"
        )
    if args.pre_tx_delay or args.post_tx_delay:
        print(
            f"Manual TX delays: pre={args.pre_tx_delay}s "
            f"post={args.post_tx_delay}s"
        )
    with open_serial_profile(
        port=port,
        baudrate=baudrate,
        bytesize=bytesize,
        parity=parity,
        stopbits=stopbits,
        timeout=timeout,
        args=args,
    ) as ser:
        if args.replay_pts_status:
            addresses = range(0x50, 0x70) if args.scan_addresses else [pump_address]
            sequences = range(16) if args.try_all_sequences else [args.replay_sequence]
            for address in addresses:
                for sequence in sequences:
                    print(f"Trying PTS replay addr=0x{address:02X} seq={sequence}")
                    reply = run_pts_style_replay(
                        ser=ser,
                        pump_address=address,
                        sequence=sequence,
                        timeout=timeout,
                        args=args,
                    )
                    if reply:
                        print(f"Response received with PTS replay addr=0x{address:02X} seq={sequence}")
                        return reply
                    time.sleep(0.2)
            return b""

        if args.poll_only:
            addresses = range(0x50, 0x70) if args.scan_addresses else [pump_address]
            for address in addresses:
                print(f"Trying POLL addr=0x{address:02X}")
                reply = run_poll_only(
                    ser=ser,
                    pump_address=address,
                    timeout=timeout,
                    poll_count=args.poll_count,
                    args=args,
                )
                if reply:
                    print(f"Response received with POLL addr=0x{address:02X}")
                    return reply
                time.sleep(0.2)
            return b""

        if args.raw_payload:
            request = transaction
            print(f"Handshake request: {hexdump(request)}")
            return try_handshake(ser, request, timeout_seconds=timeout, args=args)

        addresses = range(0x50, 0x70) if args.scan_addresses else [pump_address]
        sequences = range(16) if args.try_all_sequences else [args.sequence]
        crc_names = CRC_FUNCTIONS.keys() if args.try_all_crc else [args.crc]

        for address in addresses:
            for sequence in sequences:
                for crc_name in crc_names:
                    print(f"Trying addr=0x{address:02X} seq={sequence} crc={crc_name}")
                    reply = run_status_exchange(
                        ser=ser,
                        pump_address=address,
                        sequence=sequence,
                        crc_name=crc_name,
                        timeout=timeout,
                        poll_count=args.poll_count,
                        args=args,
                    )
                    if reply:
                        print(f"Response received with addr=0x{address:02X} seq={sequence} crc={crc_name}")
                        return reply
                    time.sleep(0.2)
    return b""


def main() -> int:
    parser = argparse.ArgumentParser(description="UNIMEP handshake tester")
    parser.add_argument("--port", required=True, help="Serial port, e.g. /dev/ttyUSB0")
    parser.add_argument("--pump-address", default="0x50", help="Pump address 0x50-0x6F")
    parser.add_argument("--baudrate", type=int, default=9600)
    parser.add_argument("--bytesize", type=int, choices=[7, 8], default=8)
    parser.add_argument("--parity", choices=["N", "E", "O"], default="N")
    parser.add_argument("--stopbits", type=float, choices=[1, 2], default=1)
    parser.add_argument("--timeout", type=float, default=1.0)
    parser.add_argument(
        "--crc",
        choices=sorted(CRC_FUNCTIONS.keys()),
        default="dart",
        help="Assumed line-protocol CRC",
    )
    parser.add_argument("--sequence", type=int, default=1, help="Low nibble of control/seq byte")
    parser.add_argument("--poll-count", type=int, default=2, help="How many POLL requests to send after the command")
    parser.add_argument(
        "--replay-pts-status",
        action="store_true",
        help="Replay a PTS-like status sequence: POLL, RETURN STATUS, POLL, ACK",
    )
    parser.add_argument(
        "--replay-sequence",
        type=int,
        default=8,
        help="Sequence nibble to use for --replay-pts-status",
    )
    parser.add_argument(
        "--replay-step-delay",
        type=float,
        default=0.0,
        help="Extra delay between replay steps",
    )
    parser.add_argument(
        "--poll-only",
        action="store_true",
        help="Only send short POLL frames and print EOT/data replies",
    )
    parser.add_argument(
        "--raw-payload",
        action="store_true",
        help="Send only the application transaction, without assumed line framing",
    )
    parser.add_argument(
        "--try-all-crc",
        action="store_true",
        help="Try all built-in CRC variants until the pump replies",
    )
    parser.add_argument(
        "--try-all-sequences",
        action="store_true",
        help="Try sequence/control low nibble values 0..15",
    )
    parser.add_argument(
        "--scan-addresses",
        action="store_true",
        help="Try all pump addresses from 0x50 to 0x6F",
    )
    parser.add_argument(
        "--sweep-common-configs",
        action="store_true",
        help="Try common serial settings: 9600/4800/2400 with 8N1 and 7E1",
    )
    parser.add_argument(
        "--listen-seconds",
        type=float,
        default=0.0,
        help="Listen only for incoming traffic for N seconds",
    )
    parser.add_argument(
        "--rs485-mode",
        action="store_true",
        help="Enable pyserial/Linux RS485Settings on the port",
    )
    parser.add_argument(
        "--rs485-rts-level-tx",
        type=int,
        choices=[0, 1],
        default=1,
        help="RTS level while transmitting when --rs485-mode is enabled",
    )
    parser.add_argument(
        "--rs485-rts-level-rx",
        type=int,
        choices=[0, 1],
        default=0,
        help="RTS level while receiving when --rs485-mode is enabled",
    )
    parser.add_argument(
        "--rs485-delay-before-tx",
        type=float,
        default=0.0,
        help="Delay before TX when --rs485-mode is enabled",
    )
    parser.add_argument(
        "--rs485-delay-before-rx",
        type=float,
        default=0.0,
        help="Delay before RX when --rs485-mode is enabled",
    )
    parser.add_argument(
        "--pre-tx-delay",
        type=float,
        default=0.0,
        help="Manual sleep before every transmit",
    )
    parser.add_argument(
        "--post-tx-delay",
        type=float,
        default=0.0,
        help="Manual sleep after every transmit before reading",
    )
    args = parser.parse_args()

    try:
        pump_address = int(args.pump_address, 0)
    except ValueError as exc:
        print(f"invalid pump address: {args.pump_address} ({exc})", file=sys.stderr)
        return 2

    if not (0x50 <= pump_address <= 0x6F):
        print("pump address must be between 0x50 and 0x6F", file=sys.stderr)
        return 2

    if args.listen_seconds > 0:
        return listen_mode(args)

    transaction = build_return_status_transaction()

    try:
        reply = b""
        if args.sweep_common_configs:
            for baudrate, bytesize, parity, stopbits in COMMON_SERIAL_PROFILES:
                reply = probe_with_serial_profile(
                    port=args.port,
                    baudrate=baudrate,
                    bytesize=bytesize,
                    parity=parity,
                    stopbits=stopbits,
                    timeout=args.timeout,
                    transaction=transaction,
                    pump_address=pump_address,
                    args=args,
                )
                if reply:
                    break
        else:
            reply = probe_with_serial_profile(
                port=args.port,
                baudrate=args.baudrate,
                bytesize=args.bytesize,
                parity=args.parity,
                stopbits=args.stopbits,
                timeout=args.timeout,
                transaction=transaction,
                pump_address=pump_address,
                args=args,
            )
    except serial.SerialException as exc:
        print(f"serial error: {exc}", file=sys.stderr)
        return 1

    if not reply:
        print("No reply received.")
        print(
            "Try different serial settings, "
            "--scan-addresses, --try-all-sequences, or --raw-payload."
        )
        return 1

    decode_and_print_reply(reply)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

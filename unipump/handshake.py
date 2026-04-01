#!/usr/bin/env python3
"""Best-effort UNIMEP handshake tester for Raspberry Pi + USB-RS485.

This script is based on the application-layer details in `UnimepProtocol.pdf`.
That document says CRC, parity, and block sequence belong to the lower "line
protocol" and does not fully define them, so the line frame used here is an
assumption based on the examples in the PDF:

- Pump address: 0x50-0x6F
- Control/sequence byte: 0x30 | (seq & 0x0F)
- Command transaction: CD1 = 0x01, LEN = 0x01, DCC = 0x00 (RETURN STATUS)
- Trailer bytes: CRC low, CRC high, ETX=0x03, SF=0xFA

If your pump does not answer, the most likely mismatch is the CRC algorithm,
serial parameters, or the exact line-protocol framing.
"""

from __future__ import annotations

import argparse
import sys
import time
from typing import Iterable

import serial


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
COMMON_SERIAL_PROFILES = (
    (9600, 8, "N", 1),
    (9600, 7, "E", 1),
    (4800, 8, "N", 1),
    (4800, 7, "E", 1),
    (2400, 8, "N", 1),
    (2400, 7, "E", 1),
)


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
    return body + bytes([crc_lo, crc_hi, ETX, SF])


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
        # Assumed format: ADR, CTRL/SEQ, PAYLOAD..., CRC_LO, CRC_HI, ETX, SF
        if len(reply) >= 6:
            return reply[2:-4]
        return b""
    # Fallback: parse whole buffer as transactions if the device returns raw payload.
    return reply


def open_serial_profile(
    port: str,
    baudrate: int,
    bytesize: int,
    parity: str,
    stopbits: float,
    timeout: float,
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
    return serial.Serial(
        port=port,
        baudrate=baudrate,
        bytesize=bytesize_map[bytesize],
        parity=parity_map[parity],
        stopbits=stopbits,
        timeout=timeout,
        write_timeout=timeout,
    )


def open_serial(args: argparse.Namespace) -> serial.Serial:
    return open_serial_profile(
        port=args.port,
        baudrate=args.baudrate,
        bytesize=args.bytesize,
        parity=args.parity,
        stopbits=args.stopbits,
        timeout=args.timeout,
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
) -> bytes:
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    ser.write(request)
    ser.flush()
    return read_reply(ser, timeout_seconds=timeout_seconds)


def decode_and_print_reply(reply: bytes) -> None:
    print(f"Raw reply: {hexdump(reply)}")
    payload = extract_payload_from_reply(reply)
    decoded = parse_transactions(payload)
    if decoded:
        for line in decoded:
            print(f"- {line}")
    else:
        print("No decodable transactions found in reply payload.")


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
    with open_serial_profile(
        port=port,
        baudrate=baudrate,
        bytesize=bytesize,
        parity=parity,
        stopbits=stopbits,
        timeout=timeout,
    ) as ser:
        if args.raw_payload:
            request = transaction
            print(f"Handshake request: {hexdump(request)}")
            return try_handshake(ser, request, timeout_seconds=timeout)

        addresses = range(0x50, 0x70) if args.scan_addresses else [pump_address]
        sequences = range(16) if args.try_all_sequences else [args.sequence]
        crc_names = CRC_FUNCTIONS.keys() if args.try_all_crc else [args.crc]

        for address in addresses:
            for sequence in sequences:
                for crc_name in crc_names:
                    request = build_line_frame(
                        pump_address=address,
                        payload=transaction,
                        sequence=sequence,
                        crc_name=crc_name,
                    )
                    print(
                        f"Trying addr=0x{address:02X} seq={sequence} "
                        f"crc={crc_name}: {hexdump(request)}"
                    )
                    reply = try_handshake(
                        ser,
                        request,
                        timeout_seconds=timeout,
                    )
                    if reply:
                        print(
                            f"Reply received with addr=0x{address:02X} "
                            f"seq={sequence} crc={crc_name}"
                        )
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
        default="modbus",
        help="Assumed line-protocol CRC",
    )
    parser.add_argument("--sequence", type=int, default=0, help="Low nibble of control/seq byte")
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

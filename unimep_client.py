# unimep_client.py
# Python 3.10+
#
# pip install pyserial
#
# Example:
#   python unimep_client.py --port COM5 --addr 0x50 poll
#   python unimep_client.py --port COM5 --addr 0x50 status
#   python unimep_client.py --port COM5 --addr 0x50 authorize
#   python unimep_client.py --port COM5 --addr 0x50 preset-amount 20.00
#   python unimep_client.py --port COM5 --addr 0x50 price-update 1.234 2.345 3.456 4.567
#
# Notes:
# - UNIMEP/Mepsan long frame:
#   ADR, 3X, TYPE, LEN, DATA..., CRC_LO, CRC_HI, 03, FA
# - Short poll:
#   ADR, 20, FA
# - ACK:
#   ADR, CX, FA    where X matches the sender's sequence nibble
# - EOT:
#   ADR, 70, FA
#
# IMPORTANT:
# Replace crc16_unimep() with the correct CRC algorithm if needed.

from __future__ import annotations

import argparse
import binascii
import logging
import time
from dataclasses import dataclass
from decimal import Decimal, ROUND_HALF_UP
from typing import Optional

import serial


logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)s %(message)s"
)


def hx(data: bytes) -> str:
    return binascii.hexlify(data, sep=" ").decode("ascii").upper()


def bcd_pack_digits(digits: str) -> bytes:
    """
    Pack an even number of decimal digits into BCD bytes.
    Example: '00000800' -> b'\\x00\\x00\\x08\\x00'
    """
    if len(digits) % 2 != 0:
        raise ValueError("digits length must be even for BCD packing")
    if not digits.isdigit():
        raise ValueError("digits must be numeric")
    out = bytearray()
    for i in range(0, len(digits), 2):
        hi = int(digits[i], 10)
        lo = int(digits[i + 1], 10)
        out.append((hi << 4) | lo)
    return bytes(out)


def bcd_unpack(data: bytes) -> str:
    s = []
    for b in data:
        s.append(str((b >> 4) & 0x0F))
        s.append(str(b & 0x0F))
    return "".join(s)


def liters_to_bcd4(value: Decimal) -> bytes:
    """
    Protocol examples show 2 decimal places for displayed volume.
    8.00 -> '00000800' -> 00 00 08 00
    """
    scaled = int((value * 100).quantize(Decimal("1"), rounding=ROUND_HALF_UP))
    return bcd_pack_digits(f"{scaled:08d}")


def amount_to_bcd4(value: Decimal) -> bytes:
    """
    Same style as volume examples: 20.00 -> '00002000'
    """
    scaled = int((value * 100).quantize(Decimal("1"), rounding=ROUND_HALF_UP))
    return bcd_pack_digits(f"{scaled:08d}")


def price_to_bcd3(value: Decimal) -> bytes:
    """
    Price examples:
    1.234 -> 00 12 34
    3.456 -> 00 34 56
    So we encode 3 decimal places into 6 digits.
    """
    scaled = int((value * 1000).quantize(Decimal("1"), rounding=ROUND_HALF_UP))
    return bcd_pack_digits(f"{scaled:06d}")


def decode_bcd4_2dp(data: bytes) -> Decimal:
    digits = bcd_unpack(data)
    return Decimal(digits[:-2] + "." + digits[-2:])


def decode_bcd3_3dp(data: bytes) -> Decimal:
    digits = bcd_unpack(data)
    return Decimal(digits[:-3] + "." + digits[-3:])


def crc16_unimep(data: bytes) -> int:
    """
    UNIMEP line framing uses CRC-16/IBM reflected with init=0x0000.
    Captured frames such as:
    50 30 01 01 05 03 04 00 23 30 01 4D 5B 03 FA
    validate as CRC_LO=0x4D, CRC_HI=0x5B with this variant.
    """
    crc = 0x0000
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


@dataclass
class SerialSettings:
    port: str
    baudrate: int = 9600
    timeout: float = 0.5
    write_timeout: float = 0.5


class UnimepClient:
    def __init__(self, settings: SerialSettings, pump_addr: int):
        self.settings = settings
        self.pump_addr = pump_addr
        self.ser: Optional[serial.Serial] = None
        self.tx_seq = 1

    def open(self) -> None:
        self.ser = serial.Serial(
            port=self.settings.port,
            baudrate=self.settings.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=self.settings.timeout,
            write_timeout=self.settings.write_timeout,
        )
        logging.info("Opened %s @ %d", self.settings.port, self.settings.baudrate)

    def close(self) -> None:
        if self.ser and self.ser.is_open:
            self.ser.close()
            logging.info("Closed serial port")

    def _next_seq(self) -> int:
        seq = self.tx_seq
        self.tx_seq += 1
        if self.tx_seq > 0x0F:
            self.tx_seq = 1
        return seq

    def _escape_crc_byte(self, b: int) -> bytes:
        # Summary says if CRC byte equals FA, send DLE before it as 10 FA.
        if b == 0xFA:
            return bytes([0x10, 0xFA])
        return bytes([b])

    def build_short_poll(self) -> bytes:
        return bytes([self.pump_addr, 0x20, 0xFA])

    def build_short_ack(self, seq: int) -> bytes:
        return bytes([self.pump_addr, 0xC0 | (seq & 0x0F), 0xFA])

    def build_long(self, msg_type: int, data: bytes, seq: Optional[int] = None) -> tuple[bytes, int]:
        if seq is None:
            seq = self._next_seq()

        ctrl = 0x30 | (seq & 0x0F)
        frame_wo_crc = bytes([self.pump_addr, ctrl, msg_type, len(data)]) + data
        crc = crc16_unimep(frame_wo_crc)
        crc_lo = crc & 0xFF
        crc_hi = (crc >> 8) & 0xFF

        frame = bytearray(frame_wo_crc)
        frame.extend(self._escape_crc_byte(crc_lo))
        frame.extend(self._escape_crc_byte(crc_hi))
        frame.append(0x03)
        frame.append(0xFA)
        return bytes(frame), seq

    def write(self, payload: bytes) -> None:
        if not self.ser:
            raise RuntimeError("Serial port not open")
        logging.info("TX: %s", hx(payload))
        self.ser.write(payload)
        self.ser.flush()

    def read_until_fa(self, timeout: float = 1.0) -> bytes:
        if not self.ser:
            raise RuntimeError("Serial port not open")

        deadline = time.time() + timeout
        buf = bytearray()
        while time.time() < deadline:
            chunk = self.ser.read(1)
            if not chunk:
                continue
            buf.extend(chunk)
            if chunk[0] == 0xFA:
                break

        data = bytes(buf)
        if data:
            logging.info("RX: %s", hx(data))
        return data

    def send_long_and_wait_ack(self, msg_type: int, data: bytes, retries: int = 3) -> int:
        for _ in range(retries):
            frame, seq = self.build_long(msg_type, data)
            self.write(frame)
            resp = self.read_until_fa(timeout=1.0)
            expected = self.build_short_ack(seq)
            if resp == expected:
                return seq
            logging.warning("Unexpected ACK response, expected %s got %s", hx(expected), hx(resp))
        raise TimeoutError("Pump did not ACK long frame")

    def poll_once(self) -> bytes:
        self.write(self.build_short_poll())
        return self.read_until_fa(timeout=1.0)

    def ack_pump_message_if_needed(self, frame: bytes) -> None:
        if len(frame) >= 2 and frame[0] == self.pump_addr and (frame[1] & 0xF0) == 0x30:
            seq = frame[1] & 0x0F
            self.write(self.build_short_ack(seq))

    # -------- Commands --------

    def cmd_return_status(self) -> None:
        self.send_long_and_wait_ack(0x01, bytes([0x00]))

    def cmd_return_filling_info(self) -> None:
        self.send_long_and_wait_ack(0x01, bytes([0x04]))

    def cmd_reset(self) -> None:
        self.send_long_and_wait_ack(0x01, bytes([0x05]))

    def cmd_authorize(self) -> None:
        self.send_long_and_wait_ack(0x01, bytes([0x06]))

    def cmd_stop(self) -> None:
        self.send_long_and_wait_ack(0x01, bytes([0x08]))

    def cmd_pause(self) -> None:
        self.send_long_and_wait_ack(0x01, bytes([0x0B]))

    def cmd_resume(self) -> None:
        self.send_long_and_wait_ack(0x01, bytes([0x0C]))

    def cmd_allow_nozzles(self, nozzles: list[int]) -> None:
        data = bytes(nozzles)
        self.send_long_and_wait_ack(0x02, data)

    def cmd_preset_volume(self, liters: Decimal) -> None:
        self.send_long_and_wait_ack(0x03, liters_to_bcd4(liters))

    def cmd_preset_amount(self, amount: Decimal) -> None:
        self.send_long_and_wait_ack(0x04, amount_to_bcd4(amount))

    def cmd_price_update(self, prices: list[Decimal]) -> None:
        payload = b"".join(price_to_bcd3(p) for p in prices)
        self.send_long_and_wait_ack(0x05, payload)

    def cmd_request_total_volume(self, nozzle: int) -> None:
        # Main summary examples show 65 01 04 for nozzle 4 volume total.
        self.send_long_and_wait_ack(0x65, bytes([nozzle]))

    # -------- Decoding pump messages --------

    @staticmethod
    def is_eot(frame: bytes) -> bool:
        return len(frame) == 3 and frame[1] == 0x70 and frame[2] == 0xFA

    def decode_pump_frame(self, frame: bytes) -> dict:
        if len(frame) < 3:
            return {"kind": "unknown", "raw": hx(frame)}

        if self.is_eot(frame):
            return {"kind": "eot"}

        if len(frame) >= 5 and (frame[1] & 0xF0) == 0x30:
            msg_type = frame[2]
            lng = frame[3]
            data = frame[4:4 + lng]

            if msg_type == 0x01 and lng == 1:
                status_map = {
                    0x00: "PUMP_NOT_PROGRAMMED",
                    0x01: "RESET",
                    0x02: "AUTHORIZED",
                    0x04: "FILLING",
                    0x05: "FILLING_COMPLETED",
                    0x06: "MAX_AMOUNT_VOLUME_REACHED",
                    0x07: "SWITCHED_OFF",
                    0x0B: "PAUSED",
                }
                return {
                    "kind": "status",
                    "status_code": data[0],
                    "status": status_map.get(data[0], f"UNKNOWN_{data[0]:02X}")
                }

            if msg_type == 0x02 and lng == 8:
                vol = decode_bcd4_2dp(data[:4])
                amo = decode_bcd4_2dp(data[4:8])
                return {
                    "kind": "filling_values",
                    "volume": str(vol),
                    "amount": str(amo),
                }

            if msg_type == 0x03 and lng == 4:
                price = decode_bcd3_3dp(data[:3])
                nozio = data[3]
                nozzle = nozio & 0x0F
                nozzle_out = bool((nozio >> 4) & 0x01)
                return {
                    "kind": "nozzle_status",
                    "price": str(price),
                    "nozzle": nozzle,
                    "nozzle_out": nozzle_out,
                }

            if msg_type == 0x05 and lng == 1:
                return {
                    "kind": "alarm",
                    "alarm_code": data[0]
                }

            if msg_type == 0x65 and lng >= 6:
                nozzle = data[0]
                total = Decimal(bcd_unpack(data[1:6])[:-2] + "." + bcd_unpack(data[1:6])[-2:])
                return {
                    "kind": "volume_total",
                    "nozzle": nozzle,
                    "total_volume": str(total),
                }

            return {
                "kind": "long_message",
                "msg_type": f"0x{msg_type:02X}",
                "data": hx(data),
            }

        return {"kind": "unknown", "raw": hx(frame)}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True)
    parser.add_argument("--addr", default="0x50", help="Pump address, e.g. 0x50")
    parser.add_argument("--baud", type=int, default=9600)

    sub = parser.add_subparsers(dest="cmd", required=True)

    sub.add_parser("poll")
    sub.add_parser("status")
    sub.add_parser("filling-info")
    sub.add_parser("reset")
    sub.add_parser("authorize")
    sub.add_parser("stop")
    sub.add_parser("pause")
    sub.add_parser("resume")

    p_allow = sub.add_parser("allow-nozzles")
    p_allow.add_argument("nozzles", nargs="+", type=int)

    p_pv = sub.add_parser("preset-volume")
    p_pv.add_argument("liters")

    p_pa = sub.add_parser("preset-amount")
    p_pa.add_argument("amount")

    p_price = sub.add_parser("price-update")
    p_price.add_argument("prices", nargs="+")

    p_tot = sub.add_parser("total-volume")
    p_tot.add_argument("nozzle", type=int)

    args = parser.parse_args()

    pump_addr = int(args.addr, 16) if args.addr.lower().startswith("0x") else int(args.addr)
    client = UnimepClient(SerialSettings(args.port, args.baud), pump_addr)

    try:
        client.open()

        if args.cmd == "poll":
            frame = client.poll_once()
            print(client.decode_pump_frame(frame))
            client.ack_pump_message_if_needed(frame)

        elif args.cmd == "status":
            client.cmd_return_status()
            frame = client.poll_once()
            print(client.decode_pump_frame(frame))
            client.ack_pump_message_if_needed(frame)

        elif args.cmd == "filling-info":
            client.cmd_return_filling_info()
            frame = client.poll_once()
            print(client.decode_pump_frame(frame))
            client.ack_pump_message_if_needed(frame)

        elif args.cmd == "reset":
            client.cmd_reset()

        elif args.cmd == "authorize":
            client.cmd_authorize()

        elif args.cmd == "stop":
            client.cmd_stop()

        elif args.cmd == "pause":
            client.cmd_pause()

        elif args.cmd == "resume":
            client.cmd_resume()

        elif args.cmd == "allow-nozzles":
            client.cmd_allow_nozzles(args.nozzles)

        elif args.cmd == "preset-volume":
            client.cmd_preset_volume(Decimal(args.liters))

        elif args.cmd == "preset-amount":
            client.cmd_preset_amount(Decimal(args.amount))

        elif args.cmd == "price-update":
            client.cmd_price_update([Decimal(x) for x in args.prices])

        elif args.cmd == "total-volume":
            client.cmd_request_total_volume(args.nozzle)
            frame = client.poll_once()
            print(client.decode_pump_frame(frame))
            client.ack_pump_message_if_needed(frame)

        return 0

    finally:
        client.close()


if __name__ == "__main__":
    raise SystemExit(main())
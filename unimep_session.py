# unimep_session.py
import serial
import time
import binascii

PORT = "/dev/ttyUSB0"
ADDR = 0x50

def hx(data: bytes) -> str:
    return binascii.hexlify(data, sep=" ").decode().upper()

def bcd_to_str(data: bytes) -> str:
    out = ""
    for b in data:
        out += str((b >> 4) & 0x0F)
        out += str(b & 0x0F)
    return out

def decode_price_3(data: bytes) -> str:
    s = bcd_to_str(data)
    return f"{int(s[:-3])}.{s[-3:]}"

def decode_amount_or_volume_4(data: bytes) -> str:
    s = bcd_to_str(data)
    return f"{int(s[:-2])}.{s[-2:]}"

def parse_transactions(msg_type: int, payload: bytes):
    """
    Parse one or more transactions inside payload.
    For real pump traffic, the first transaction is usually:
      01 01 xx  -> status
      02 08 ... -> vol/amount
      03 04 ... -> nozzle/price
    and some frames may chain multiple transactions.
    """
    i = 0
    items = []

    while i + 2 <= len(payload):
        trans = payload[i]
        lng = payload[i + 1]
        start = i + 2
        end = start + lng

        if end > len(payload):
            items.append({"kind": "truncated", "at": i, "remaining": hx(payload[i:])})
            break

        data = payload[start:end]

        if trans == 0x01 and lng == 1:
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
            items.append({
                "type": "status",
                "code": data[0],
                "value": status_map.get(data[0], f"UNKNOWN_{data[0]:02X}")
            })

        elif trans == 0x02 and lng == 8:
            items.append({
                "type": "filling_values",
                "volume": decode_amount_or_volume_4(data[:4]),
                "amount": decode_amount_or_volume_4(data[4:8]),
            })

        elif trans == 0x03 and lng == 4:
            nozio = data[3]
            items.append({
                "type": "nozzle_status",
                "price": decode_price_3(data[:3]),
                "nozzle": nozio & 0x0F,
                "nozzle_out": bool((nozio >> 4) & 0x01),
                "raw_nozio": f"0x{nozio:02X}",
            })

        elif trans == 0x05 and lng == 1:
            items.append({
                "type": "alarm",
                "alarm_code": data[0]
            })

        elif trans == 0x65:
            items.append({
                "type": "totals",
                "raw": hx(data)
            })

        else:
            items.append({
                "type": f"unknown_trans_0x{trans:02X}",
                "len": lng,
                "raw": hx(data)
            })

        i = end

    return items

def read_frame(ser: serial.Serial, timeout=1.0) -> bytes:
    deadline = time.time() + timeout
    buf = bytearray()

    while time.time() < deadline:
        chunk = ser.read(ser.in_waiting or 1)
        if chunk:
            buf.extend(chunk)
            if buf and buf[-1] == 0xFA:
                return bytes(buf)

    return bytes(buf)

def build_poll(addr: int) -> bytes:
    return bytes([addr, 0x20, 0xFA])

def build_ack(addr: int, seq: int) -> bytes:
    return bytes([addr, 0xC0 | (seq & 0x0F), 0xFA])

def decode_frame(frame: bytes):
    if not frame:
        return {"kind": "empty"}

    if len(frame) == 3 and frame[0] == ADDR and frame[1] == 0x70 and frame[2] == 0xFA:
        return {"kind": "eot"}

    if len(frame) >= 8 and frame[0] == ADDR and (frame[1] & 0xF0) == 0x30:
        seq = frame[1] & 0x0F

        # strip ADR CTRL ... CRC CRC 03 FA
        if frame[-2:] != b"\x03\xFA":
            return {"kind": "unknown", "raw": hx(frame)}

        # We are not validating CRC here yet
        body = frame[2:-4]  # everything after ADR,CTRL and before CRC_LO,CRC_HI,03,FA
        transactions = parse_transactions(frame[2], body)
        return {
            "kind": "long",
            "seq": seq,
            "raw": hx(frame),
            "transactions": transactions
        }

    return {"kind": "unknown", "raw": hx(frame)}

def main():
    ser = serial.Serial(
        PORT,
        baudrate=9600,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0.2,
        write_timeout=0.5
    )

    print(f"Opened {PORT}")

    try:
        while True:
            poll = build_poll(ADDR)
            ser.write(poll)
            ser.flush()
            print("TX", hx(poll))

            frame = read_frame(ser, timeout=0.8)
            if not frame:
                time.sleep(0.1)
                continue

            print("RX", hx(frame))
            decoded = decode_frame(frame)
            print(decoded)

            if decoded.get("kind") == "long":
                ack = build_ack(ADDR, decoded["seq"])
                ser.write(ack)
                ser.flush()
                print("TX", hx(ack))

            time.sleep(0.1)

    finally:
        ser.close()

if __name__ == "__main__":
    main()
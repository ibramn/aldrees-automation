import serial
import time
import binascii

PORT = "/dev/ttyAMA0"
BAUD = 9600

def hx(b: bytes) -> str:
    return binascii.hexlify(b, sep=" ").decode().upper()

def bcd_digits(data: bytes) -> str:
    s = ""
    for b in data:
        s += str((b >> 4) & 0x0F)
        s += str(b & 0x0F)
    return s

def price3(data: bytes) -> str:
    s = bcd_digits(data)
    return f"{int(s[:-3])}.{s[-3:]}"

def split_frames(buf: bytearray):
    out = []
    while True:
        try:
            i = buf.index(0xFA)
        except ValueError:
            break
        out.append(bytes(buf[:i+1]))
        del buf[:i+1]
    return out

def decode_payload(payload: bytes):
    i = 0
    items = []
    while i + 2 <= len(payload):
        trans = payload[i]
        lng = payload[i+1]
        data = payload[i+2:i+2+lng]
        if len(data) < lng:
            items.append({"type": "TRUNCATED", "raw": hx(payload[i:])})
            break

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
                "type": "STATUS",
                "code": f"{data[0]:02X}",
                "value": status_map.get(data[0], "UNKNOWN")
            })

        elif trans == 0x03 and lng == 4:
            nozio = data[3]
            items.append({
                "type": "NOZZLE_STATUS",
                "price": price3(data[:3]),
                "nozzle": nozio & 0x0F,
                "nozzle_out": bool((nozio >> 4) & 0x01),
                "raw_nozio": f"{nozio:02X}"
            })

        elif trans == 0x02:
            items.append({
                "type": "TRANS_02",
                "len": lng,
                "raw": hx(data)
            })

        else:
            items.append({
                "type": f"TRANS_{trans:02X}",
                "len": lng,
                "raw": hx(data)
            })

        i += 2 + lng
    return items

def classify(frame: bytes):
    if len(frame) == 3 and frame[1] == 0x20:
        return {"kind": "POLL"}
    if len(frame) == 3 and frame[1] == 0x70:
        return {"kind": "EOT"}
    if len(frame) == 3 and (frame[1] & 0xF0) == 0xC0:
        return {"kind": "ACK", "seq": frame[1] & 0x0F}
    if len(frame) >= 8 and (frame[1] & 0xF0) == 0x30 and frame[-2:] == b"\x03\xFA":
        seq = frame[1] & 0x0F
        payload = frame[2:-4]   # exclude ADR/CTRL and CRCLO/CRCHI/03/FA
        return {"kind": "LONG", "seq": seq, "transactions": decode_payload(payload)}
    return {"kind": "UNKNOWN", "raw": hx(frame)}

ser = serial.Serial(PORT, BAUD, timeout=0.2)
buf = bytearray()

print(f"Listening on {PORT} @ {BAUD}")

try:
    while True:
        data = ser.read(ser.in_waiting or 1)
        if data:
            buf.extend(data)
            for frame in split_frames(buf):
                print(time.strftime("%H:%M:%S"), hx(frame), classify(frame))
finally:
    ser.close()
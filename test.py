import binascii
import time

import RPi.GPIO as GPIO
import serial

PORT = "/dev/ttyAMA0"
ADDR = 0x50
DE_RE = 17
PRE_TX_DELAY = 0.001
POST_TX_DELAY = 0.0
READ_TIMEOUT = 1.0
IDLE_GAP = 0.15
EXCHANGES_PER_PROFILE = 4
PROFILES = [
    ("8N1", serial.EIGHTBITS, serial.PARITY_NONE, serial.STOPBITS_ONE),
    ("7E1", serial.SEVENBITS, serial.PARITY_EVEN, serial.STOPBITS_ONE),
]


def hx(data: bytes) -> str:
    return binascii.hexlify(data, sep=" ").decode().upper()


def crc16_unimep(data: bytes) -> int:
    # Captured pump frames match CRC-16/IBM reflected with init=0x0000.
    crc = 0x0000
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def build_poll() -> bytes:
    return bytes([ADDR, 0x20, 0xFA])


def build_ack(seq: int) -> bytes:
    return bytes([ADDR, 0xC0 | (seq & 0x0F), 0xFA])


def build_return_status(seq: int) -> bytes:
    payload = bytes([0x01, 0x01, 0x00])
    body = bytes([ADDR, 0x30 | (seq & 0x0F)]) + payload
    crc = crc16_unimep(body)
    return body + bytes([crc & 0xFF, (crc >> 8) & 0xFF, 0x03, 0xFA])


def read_reply_window(ser: serial.Serial, timeout: float = READ_TIMEOUT, idle_gap: float = IDLE_GAP) -> bytes:
    deadline = time.monotonic() + timeout
    buf = bytearray()
    last_data_at = None

    while time.monotonic() < deadline:
        chunk = ser.read(256)
        if chunk:
            buf.extend(chunk)
            last_data_at = time.monotonic()
        else:
            if buf and last_data_at is not None and (time.monotonic() - last_data_at) >= idle_gap:
                break
            time.sleep(0.02)

    return bytes(buf)


def split_frames(stream: bytes) -> list[bytes]:
    frames = []
    i = 0

    while i < len(stream):
        if i + 2 < len(stream) and stream[i] == ADDR and stream[i + 2] == 0xFA:
            msg_type = stream[i + 1]
            if msg_type in (0x20, 0x70) or 0xC0 <= msg_type <= 0xCF or 0xE0 <= msg_type <= 0xEF:
                frames.append(stream[i:i + 3])
                i += 3
                continue

        if i + 3 < len(stream) and stream[i] == ADDR and (stream[i + 1] & 0xF0) == 0x30:
            end = i + 2
            while end + 1 < len(stream):
                if stream[end] == 0x03 and stream[end + 1] == 0xFA:
                    frames.append(stream[i:end + 2])
                    i = end + 2
                    break
                end += 1
            else:
                break
            continue

        i += 1

    return frames


def filter_echo(frames: list[bytes], expected_echo: bytes) -> list[bytes]:
    filtered = []
    for frame in frames:
        if frame == expected_echo:
            print("ECHO", hx(frame))
            continue
        filtered.append(frame)
    return filtered


def tx(ser: serial.Serial, frame: bytes) -> None:
    GPIO.output(DE_RE, 1)
    time.sleep(PRE_TX_DELAY)
    ser.reset_input_buffer()
    ser.write(frame)
    ser.flush()
    if POST_TX_DELAY > 0:
        time.sleep(POST_TX_DELAY)
    GPIO.output(DE_RE, 0)
    print("TX", hx(frame))


def open_serial(bytesize: int, parity: str, stopbits: float) -> serial.Serial:
    return serial.Serial(
        PORT,
        baudrate=9600,
        bytesize=bytesize,
        parity=parity,
        stopbits=stopbits,
        timeout=0.05,
        write_timeout=0.5,
    )


def run_exchange(ser: serial.Serial, seq: int) -> tuple[int, int]:
    rx_count = 0

    return_status = build_return_status(seq)
    tx(ser, return_status)
    raw = read_reply_window(ser, timeout=0.8)
    frames = filter_echo(split_frames(raw), return_status)
    if frames:
        for frame in frames:
            print("RX", hx(frame))
            rx_count += 1
    else:
        print("RX", "LEN=0 (no external reply)")

    poll = build_poll()
    tx(ser, poll)
    raw = read_reply_window(ser, timeout=0.8)
    frames = filter_echo(split_frames(raw), poll)
    if frames:
        for frame in frames:
            print("RX", hx(frame))
            rx_count += 1
            if len(frame) >= 4 and (frame[1] & 0xF0) == 0x30 and frame[-2:] == b"\x03\xFA":
                ack = build_ack(frame[1] & 0x0F)
                tx(ser, ack)
    else:
        print("RX", "LEN=0 (no external reply)")

    seq = 1 if seq >= 0x0F else seq + 1
    return seq, rx_count


GPIO.setmode(GPIO.BCM)
GPIO.setup(DE_RE, GPIO.OUT)
GPIO.output(DE_RE, 0)

seq = 1

try:
    profile_index = 0
    while True:
        profile_name, bytesize, parity, stopbits = PROFILES[profile_index]
        print(f"\n=== Testing {profile_name} on {PORT} ===")

        ser = open_serial(bytesize, parity, stopbits)
        profile_rx = 0
        try:
            for _ in range(EXCHANGES_PER_PROFILE):
                seq, rx_count = run_exchange(ser, seq)
                profile_rx += rx_count
                time.sleep(1)
        finally:
            ser.close()

        if profile_rx:
            print(f"RX detected on {profile_name}; staying on this profile.")
            ser = open_serial(bytesize, parity, stopbits)
            try:
                while True:
                    seq, _ = run_exchange(ser, seq)
                    time.sleep(1)
            finally:
                ser.close()

        profile_index = (profile_index + 1) % len(PROFILES)
finally:
    GPIO.cleanup()
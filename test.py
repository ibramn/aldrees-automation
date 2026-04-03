import serial
import binascii
import time

PORT = "/dev/ttyUSB0"

def hx(b: bytes) -> str:
    return binascii.hexlify(b, sep=" ").decode().upper()

ser = serial.Serial(
    PORT,
    baudrate=9600,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    timeout=2,
    write_timeout=1
)

try:
    frame = bytes([0x50, 0x20, 0xFA])
    print("TX", hx(frame))
    ser.write(frame)
    ser.flush()

    time.sleep(0.3)

    data = ser.read(100)
    print("RX", hx(data), "LEN=", len(data))
finally:
    ser.close()
import serial
import time
import binascii

PORT = "/dev/ttyAMA0"   # or /dev/ttyAMA0
BAUD = 9600

def hx(data: bytes) -> str:
    return binascii.hexlify(data, sep=" ").decode().upper()

ser = serial.Serial(
    PORT,
    baudrate=BAUD,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    timeout=0.2
)

print(f"Sniffing on {PORT} @ {BAUD}")

buf = bytearray()

try:
    while True:
        data = ser.read(ser.in_waiting or 1)
        if data:
            buf.extend(data)

            while 0xFA in buf:
                i = buf.index(0xFA)
                frame = bytes(buf[:i + 1])
                del buf[:i + 1]
                print(time.strftime("%H:%M:%S"), hx(frame))
finally:
    ser.close()
import serial
import RPi.GPIO as GPIO
import time
import binascii

PORT = "/dev/ttyAMA0"
DE_RE = 17

def hx(b):
    return binascii.hexlify(b, sep=' ').decode().upper()

GPIO.setmode(GPIO.BCM)
GPIO.setup(DE_RE, GPIO.OUT)
GPIO.output(DE_RE, 0)

ser = serial.Serial(
    PORT,
    baudrate=9600,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,   # try EVEN too
    stopbits=serial.STOPBITS_ONE,
    timeout=0.5
)

try:
    while True:
        poll = b'\x50\x20\xFA'

        GPIO.output(DE_RE, 1)   # TX
        time.sleep(0.002)

        ser.reset_input_buffer()
        ser.write(poll)
        ser.flush()

        time.sleep(0.02)
        GPIO.output(DE_RE, 0)   # RX

        time.sleep(0.3)
        data = ser.read(32)

        print("TX", hx(poll))
        print("RX", hx(data), "LEN=", len(data))
        time.sleep(1)
finally:
    ser.close()
    GPIO.cleanup()
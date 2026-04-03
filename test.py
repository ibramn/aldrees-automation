import serial
import RPi.GPIO as GPIO
import time
import binascii

DE_RE = 17

GPIO.setmode(GPIO.BCM)
GPIO.setup(DE_RE, GPIO.OUT)
GPIO.output(DE_RE, 0)

ser = serial.Serial("/dev/serial0", 9600, timeout=1)

def hx(b):
    return binascii.hexlify(b, sep=' ').decode().upper()

while True:
    poll = b'\x50\x20\xFA'

    GPIO.output(DE_RE, 1)   # TX
    time.sleep(0.002)

    ser.reset_input_buffer()
    ser.write(poll)
    ser.flush()

    time.sleep(0.01)
    GPIO.output(DE_RE, 0)   # RX

    time.sleep(0.3)
    data = ser.read(100)

    print("TX", hx(poll))
    print("RX", hx(data), "LEN=", len(data))
    time.sleep(1)
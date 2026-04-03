import serial
import RPi.GPIO as GPIO
import time

DE_RE = 17

GPIO.setmode(GPIO.BCM)
GPIO.setup(DE_RE, GPIO.OUT)

ser = serial.Serial("/dev/serial0", 9600, timeout=1)

def send(data):
    GPIO.output(DE_RE, 1)  # TX mode
    time.sleep(0.001)
    ser.write(data)
    ser.flush()
    time.sleep(0.01)
    GPIO.output(DE_RE, 0)  # RX mode

def read():
    return ser.read(100)

while True:
    send(b'\x50\x20\xFA')
    time.sleep(0.2)
    print(read())
    time.sleep(1)
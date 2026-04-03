import serial
import RPi.GPIO as GPIO
import time

DE_RE = 17

GPIO.setmode(GPIO.BCM)
GPIO.setup(DE_RE, GPIO.OUT)
GPIO.output(DE_RE, 0)  # start in receive mode

ser = serial.Serial("/dev/serial0", 9600, timeout=1)

while True:
    GPIO.output(DE_RE, 1)      # TX mode
    time.sleep(0.002)

    ser.write(b'\x50\x20\xFA')
    ser.flush()

    time.sleep(0.01)
    GPIO.output(DE_RE, 0)      # RX mode

    time.sleep(0.2)
    data = ser.read(100)
    print(data)
    time.sleep(1)
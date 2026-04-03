import serial
import time
import binascii

ser = serial.Serial(
    "/dev/ttyUSB0",
    9600,
    bytesize=8,
    parity='N',
    stopbits=1,
    timeout=5
)

print("Listening...")
while True:
    data = ser.read(ser.in_waiting or 1)
    if data:
        print(time.strftime("%H:%M:%S"), binascii.hexlify(data, sep=' ').decode().upper())
import serial, time, binascii

ser = serial.Serial("/dev/ttyAMA0", 9600, timeout=1)

while True:
    data = ser.read(256)
    if data:
        print(time.strftime("%H:%M:%S"), binascii.hexlify(data, sep=b' ').decode().upper())
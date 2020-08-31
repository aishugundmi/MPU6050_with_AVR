import serial
from serial import SerialException
import sys

ser = 0
num = 0

while 1:
    try:
        ser = serial.Serial(port='COM'+str(num), baudrate = 1000000, timeout = 1000)
        print('COM' + str(num) + ' FOUND!');
        break;
    except SerialException:
        num = num + 1
        if(num >= 50):
            print("NO COM PORT FOUND!");
            sys.exit(0)


    
while 1:
	print(ser.readline().decode("UTF-8")[:-1])
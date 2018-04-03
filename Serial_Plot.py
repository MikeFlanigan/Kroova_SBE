import serial
import time

ser = serial.Serial('COM3', 9600, timeout=0)

while True:
    print (ser.readline())
##    time.sleep(1)
    

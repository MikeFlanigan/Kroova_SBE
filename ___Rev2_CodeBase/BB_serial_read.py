import Adafruit_BBIO.UART as UART
import serial
import time

UART.setup("UART1") # RX P9_26 TX P9_24

ser = serial.Serial(port = "/dev/ttyO1", baudrate=9600)

buffer = ""
while True:
    
    oneByte = ser.read(1)
    if oneByte == b"\r":    # method should return bytes
##        continue 
##        print(buffer)
        dist = buffer
        print(dist)
##        ser.reset_input_buffer()
##        time.sleep(0.001) # rest 
    else:
        try:
            buffer += oneByte.decode("ascii")
        except UnicodeDecodeError as e:
            print('UnicodeDecodeError: ',e)

ser.close()

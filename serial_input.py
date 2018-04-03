import serial

ser = serial.Serial(
    port='COM4',\
    baudrate=9600,\
    parity=serial.PARITY_NONE,\
    stopbits=serial.STOPBITS_ONE,\
    bytesize=serial.EIGHTBITS,\
        timeout=0)

print("connected to: " + ser.portstr)

#this will store the line
line = []
val = ''
rval = 0

while True:
    try:
        for c in ser.read():
            if c == '\r':
                print(line)
                line = []
                rval = int(val)
                val = ''
                break
            else:
                line.append(c)
                val = val + c
    except KeyboardInterrupt:
        break
ser.close()
print("closed")


##while True:
##    try:
##        line = ser.read(5)
##           
##        print(line)
##    except KeyboardInterrupt:
##        break
##ser.close()
##print("closed")

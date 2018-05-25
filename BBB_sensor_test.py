'''
Sensor test
'''

import serial
import Adafruit_BBIO.ADC as ADC

# serial input setup
ser = serial.Serial(
    port = '/dev/ttyUSB0',
    baudrate = 9600,
    parity = serial.PARITY_NONE,
    stopbits = serial.STOPBITS_ONE,
    bytesize = serial.EIGHTBITS,
        timeout = 0)
print("connected to: ")
print(ser.portstr)

poten_pin = "P9_33"
poten_value = 0 # input range 0 - 1.0
ADC.setup()

line = []
val = ''
rval = 0.0
ival =  0

sen_gain = 0.003384*25.4 # converts sensor reading to mm
gained_val = 0.0 # sensor reading in mm


while True:
    try:

    	for c in ser.read():
            if c == '\r':
##                print(line)
                line = []

                try:
                    ival = int(val)
                except ValueError:
                    print("value error: ", val)

                rval = float(ival)
                val = ''
                break
            else:
                line.append(c)
                val = val + c

		gained_val = rval * sen_gain # sensor reading in mm

		poten_value = ADC.read(poten_pin)
		poten_value = ADC.read(poten_pin) # read twice due to possible known ADC driver bug

		print("Ultrasonic [mm]: ", gained_val, " Wand [0-1]: ", poten_value)
    except KeyboardInterrupt: # allows for easy program stop by tester
        break
    
# clean up
ser.close()

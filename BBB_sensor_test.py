'''
Sensor test
'''
import Adafruit_BBIO.ADC as ADC
from SBE_functions import *

# serial input setup
ser = setupSerial()

poten_pin = "P9_33"
poten_value = 0 # input range 0 - 1.0
ADC.setup()

rval = 0.0

sen_gain = 0.003384*25.4 # converts sensor reading to mm
gained_val = 0.0 # sensor reading in mm


while True:
    try:
    	rval = ToughSonicRead(ser)
    	gained_val = rval * sen_gain # sensor reading in mm

    	poten_value = ADC.read(poten_pin)
    	poten_value = ADC.read(poten_pin) # read twice due to possible known ADC driver bug
    	print("Ultrasonic [mm]: ", int(gained_val), " Wand [0-1]: ", poten_value)
    except KeyboardInterrupt: # allows for easy program stop by tester
        break
    
# clean up
ser.close()

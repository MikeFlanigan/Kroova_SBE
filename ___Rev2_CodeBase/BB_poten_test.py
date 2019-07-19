import Adafruit_BBIO.ADC as ADC

ADC.setup()
poten_pin = "P9_40"
poten_value = 0 # input range 0 - 1.0
##poten_values = [target_RH]*1000 # rolling average the analog read to smooth it out

while True:
    print(ADC.read(poten_pin))

    

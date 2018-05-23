'''
SBE_wing.py
'''

import serial
import Adafruit_BBIO.PWM as PWM
import Adafruit_BBIO.GPIO as GPIO
import time
import datetime
import Adafruit_BBIO.ADC as ADC
import random


# user settings
debug_mode = False
enable_output = True
Servo_Type = 3 # enummerator for servos --- 1 = Hobby king cheap, 2 = Ball tilt, 3 = Car wing


# Good set of gains below
P_gain = 0.24 
I_gain = 0.0002 #.0002
D_gain = 1.75 #1.75#3.00 # was 3.75

# safety control angle for steady descent
control_aoa = 2 # aoa

error = 0 # 10x the error in mm
last_error = 0 # for storing errors from previous loop
sum_error = 0 # integral term of error

D_read_Hz = 20 # Read derivative changes at 100x per second. This can be tuned to roughly match the dynamics of the system.
D_read_ms = 1.0/D_read_Hz*1000 # time in ms between each derivative read
last_derivative_read = datetime.datetime.now()
last_derivate_error = 0
rolling_avg_D_errors = [0]*50 # 50 seems good 

P_term = 0
I_term = 0
D_term = 0

target_RH_init = 647 # mm
target_RH = target_RH_init # mm
output_angle = 90 # initial servo output angle
target_aoa = output_angle

# Analog reading 
time.sleep(15) # attempt to solve bootup problem
ADC.setup()
poten_pin = "P9_33"
poten_value = 0 # input range 0 - 1.0
poten_values = [target_RH]*1000 # rolling average the analog read to smooth it out

# Run button
run_pin = "P9_15"
if not PC_testing: GPIO.setup(run_pin, GPIO.IN)



# servo output setup
if Servo_Type == 1:
    ## HK 15138
    duty_min = 3.5 
    duty_max = 14.0 
    servo_max = 180 # degrees
    servo_min = 0 # degrees
elif Servo_Type == 2:
    ## HS-815BB (ball tilt servo)
    duty_min = 7.5 
    duty_max = 11.25 
    servo_max = 180 # degrees
    servo_min = 0 # degrees
elif Servo_Type == 3:
    ## DS3218mg (wing)
    # note these two are "soft" limits based on the wing build and desired mechanical limits
    duty_min = 6.0 
    duty_max = 11.0  
    servo_max = 180 # degrees # update?
    servo_min = 0 # degrees # update?
duty_span = duty_max - duty_min

if enable_output:
    servo_pin = "P8_13"
    PWM.start(servo_pin, (duty_max-duty_min)/2.0+duty_min, 60)

output_angle = 90 # initial servo output angle

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


line = []
val = ''
rval = 0.0
ival =  0

sen_gain = 0.003384*25.4 # converts sensor reading to mm

gained_val = 0.0 # sensor reading in mm


while True:
    try:
    	if GPIO.input(run_pin): flight_control = True

    	if flight_control:
	        for c in ser.read():
	            if c == '\r':
	##                print(line)
	                line = []

	                try:
	                    ival = int(val)
	                except ValueError:
	                    print("value error: ", val)
	                
	##                print(val)
	##                print(ival)
	                rval = float(ival)
	##                print(rval)
	                val = ''
	                break
	            else:
	                line.append(c)
	                val = val + c

	        gained_val = rval * sen_gain # sensor reading in mm
	##        print(gained_val)
	##        print(round(gained_val,2), " inches")

	        error = int(gained_val)-target_RH
	        
	        P_term =  error*P_gain

	        # I_term = (error + I_term)*I_gain 
	        if (target_aoa < 60 and target_aoa > -60): sum_error = error + sum_error
	        else: sum_error = sum_error
	        I_term = sum_error*I_gain 

	        rolling_avg_D_errors.append(error)
	        rolling_avg_D_errors.pop(0)

	        D_term = (error - float(sum(rolling_avg_D_errors))/len(rolling_avg_D_errors))*D_gain

	        target_aoa = P_term +  I_term + D_term # control equation
	        if enable_output: print("P term:", int(P_term)," D term:",int(D_term)," I term:",int(I_term),"target aoa:",int(target_aoa)," error:",error," sum error: ",sum_error," target RH:",target_RH)
	        target_aoa = -target_aoa # flip if needed 
	        output_angle = target_aoa + 90 # convert aoa to absolute angle for servo

	        # update error terms
	        last_error = error

	        # threshold servo commands in case of errors
	        if output_angle > servo_max: output_angle = servo_max
	        elif output_angle < servo_min: output_angle = servo_min
	        
	        # print(output_angle)

	        if enable_output:
	            duty = float(output_angle)
	            duty = ((duty / 180) * duty_span + duty_min) 
	            PWM.set_duty_cycle(servo_pin, duty)
	    elif not flight_control:
	    	if enable_output:
	            duty = float(control_aoa)
	            duty = ((duty / 180) * duty_span + duty_min) 
	            PWM.set_duty_cycle(servo_pin, duty)

    except KeyboardInterrupt: # allows for easy program stop by tester
        break
    
# clean up
ser.close()
if enable_output: 
    PWM.stop(servo_pin)
    PWM.cleanup()





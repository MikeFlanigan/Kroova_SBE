'''
SBE_wing.py
'''
import Adafruit_BBIO.PWM as PWM
import Adafruit_BBIO.GPIO as GPIO
import time
import datetime
import Adafruit_BBIO.ADC as ADC
import random
from SBE_functions import *

# user settings
debug_mode = False
enable_output = True
PC_testing = False

filter_US_input = True
US_input_array = []
US_filter_size = 50

# Good set of gains below
P_gain = 0.15 
I_gain = 0.0002 #
D_gain = 0.10 #

# safety control angle for steady descent
control_aoa = 65 # aoa

error = 0 # 10x the error in mm
sum_error = 0 # integral term of error

D_read_Hz = 20 # Read derivative changes at 100x per second. This can be tuned to roughly match the dynamics of the system.
D_read_ms = 1.0/D_read_Hz*1000 # time in ms between each derivative read
last_derivative_read = datetime.datetime.now()
last_derivate_error = 0
rolling_avg_D_errors = [0]*70 # 50 seems good 

# initialization
P_term = 0
I_term = 0
D_term = 0

target_RH_init = 647 # mm
target_RH = target_RH_init # mm
output_angle = control_aoa # initial servo output angle
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
# 1-HK 15138, 2-HS-815BB (ball tilt servo), 3-DS3218mg (wing)"soft" limits, 4-Moth servo
Servo_Type = 3
duty_min, duty_max = servo_parameters(Servo_Type) # parameters for various kroova servos stored in the function
duty_span = duty_max - duty_min

if enable_output:
    servo_pin = "P8_13"
    PWM.start(servo_pin, (duty_max-duty_min)/2.0+duty_min, 60)

output_angle = 90 # initial servo output angle

# serial input setup
ser = setupSerial()

rval = 0.0


sen_gain = 0.003384*25.4 # converts sensor reading to mm

gained_val = 0.0 # sensor reading in mm


while True:
    try:
    	if GPIO.input(run_pin): flight_control = True
    	else: flight_control = False

    	if flight_control:
    		rval = ToughSonicRead(ser)

	        gained_val = rval * sen_gain # sensor reading in mm

	        if filter_US_input:
	        	US_input_array.append(gained_val)
	        	if len(US_input_array) >= US_filter_size:
	        		US_input_array.pop(0)

	        	gained_val = sum(US_input_array)/len(US_input_array) # averaged US input

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
	        target_aoa = -target_aoa # flip if needed 
	        output_angle = target_aoa + 90 # convert aoa to absolute angle for servo

	        # threshold servo commands in case of errors
	        if output_angle > servo_max: output_angle = servo_max
	        elif output_angle < servo_min: output_angle = servo_min

	        if enable_output: print("P term:", int(P_term)," D term:",int(D_term)," I term:",int(I_term), "target angle:",int(target_aoa+90),"output angle:",int(output_angle)," error:",error," sum error: ",sum_error," target RH:",target_RH)

	        if enable_output:
	            duty = float(output_angle)
	            duty = ((duty / 180) * duty_span + duty_min) 
	            PWM.set_duty_cycle(servo_pin, duty)
        elif not flight_control:
	    	if enable_output:
				duty = float(control_aoa)
				duty = ((duty / 180) * duty_span + duty_min) 
				PWM.set_duty_cycle(servo_pin, duty)
				print("Control angle:", control_aoa)

    except KeyboardInterrupt: # allows for easy program stop by tester
        break
    
# clean up
ser.close()
if enable_output: 
    PWM.stop(servo_pin)
    PWM.cleanup()





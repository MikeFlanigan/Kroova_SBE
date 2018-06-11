
from SBE_functions import *
import datetime
import Adafruit_BBIO.PWM as PWM

enable_debug_print = True

# ------------- CONTROL VARIABLES setup ------------
servo_max = 107
servo_min = 69

US_input_array = []
US_rolling_avg_window = 30
averaged_US_input = 200 # some initial

# Gains
P_gain = -0.05
I_gain = 0.000 
D_gain = 0.0 

servo_control_offset = 92.0

error = 0 
last_error = 0 # for storing errors from previous loop
sum_error = 0 # integral term of error

# last_derivate_error = 0
# rolling_avg_D_errors = [0]*70 # 50 seems good 

target_RH = 850 # mm

US_max_thresh = 1050 # mm -- based on blade rider foil and mounting setup as of 6/11/18
US_min_thresh = 50 # mm -- based on blade rider foil and mounting setup as of 6/11/18
# --- End ----- CONTROL VARIABLES setup ------------


# ------------- Timing setup ------------
control_freq = 100 # hz (actually means it will record a bit slower than this due to checking time without interrupts)
control_freq_micros = 1.0/control_freq*1000*1000 # number of microseconds before data grab
control_timer = datetime.datetime.now()
# --- End ----- Timing setup ------------

# ------------- Serial reading setup ------------
# time.sleep(15) # attempt to solve bootup problem
ser = setupSerial()

sen_gain = 0.003384*25.4 # converts sensor reading to mm
gained_val = 0.0 # sensor reading in mm
# --- End ----- Serial reading setup ------------

# ------------- Servo output setup -------------
# 1-HK 15138, 2-HS-815BB (ball tilt servo), 3-DS3218mg (wing)"soft" limits, 4-Moth servo
Servo_Type = 4
duty_min, duty_max = servo_parameters(Servo_Type) # parameters for various kroova servos stored in the function
duty_span = duty_max - duty_min

servo_pin = "P8_13"
PWM.start(servo_pin, (duty_max-duty_min)/2.0+duty_min, 60)

control_servo_angle = 90 # initial servo output angle
# --- End ----- Servo output setup ------------


while True:
	try:
		# -------- Ultrasonic serial reading --------
		# reading at full speed seems to minimize error values
		rval = ToughSonicRead(ser)
		if US_min_thresh < rval*sen_gain < US_max_thresh:
			gained_val = rval * sen_gain # sensor reading in mm
		elif rval*sen_gain > US_max_thresh and abs(averaged_US_input-rval*sen_gain) < 200: # this allows the max thresh to be higher if the avg values are high
			gained_val = rval * sen_gain # sensor reading in mm
		# else gained val doesn't update and is equal to the last reading 
		# --End -- Ultrasonic serial reading --------

		if (datetime.datetime.now() - control_timer).microseconds >= control_freq_micros:
			control_timer = datetime.datetime.now() # resets the timer

			US_input_array.append(gained_val)
			if len(US_input_array) >= US_rolling_avg_window:
				US_input_array.pop(0)
				averaged_US_input = sum(US_input_array)/len(US_input_array)
				gained_val =  averaged_US_input # averaged US input

			error = int(gained_val)-target_RH

			P_term =  error*P_gain

			if (control_servo_angle < servo_max and control_servo_angle > servo_min): sum_error = error + sum_error
			else: sum_error = sum_error
			I_term = sum_error*I_gain 

			# rolling_avg_D_errors.append(error)
			# rolling_avg_D_errors.pop(0)
			# D_term = (error - float(sum(rolling_avg_D_errors))/len(rolling_avg_D_errors))*D_gain
			D_term = (error - last_error)*D_gain
			# update error terms
			last_error = error

			control_servo_angle = P_term +  I_term + D_term + servo_control_offset # control equation

			# threshold servo commands in case of errors
			if control_servo_angle > servo_max: control_servo_angle = servo_max
			elif control_servo_angle < servo_min: control_servo_angle = servo_min

			duty = float(control_servo_angle)
			duty = ((duty / 180) * duty_span + duty_min) 
			PWM.set_duty_cycle(servo_pin, duty)

	        if enable_debug_print: 
	        	print("Error:",error,
	        		"P term:", int(P_term)," D term:",int(D_term)," I term:",int(I_term),
	        		"control_servo_angle:",int(control_servo_angle))

	except KeyboardInterrupt: # allows for easy program stop by tester
		break
    
# clean up
ser.close()
PWM.stop(servo_pin)
PWM.cleanup()
# ---------- Program Mode Controls ---------------
saving_data = True
auto_mount_usb = True
enable_servo = True
# -- End --- Program Mode Controls ---------------
import Adafruit_BBIO.GPIO as GPIO
import Adafruit_BBIO.ADC as ADC
import Adafruit_BBIO.PWM as PWM
import datetime
import numpy as np 
import os
import time
from SBE_functions import *


'''
data_row : row of data to be saved to a csv, format: 
unix time (converted to PST tho), potentiometer [0-1.0],ultrasonic [mm],target_RH [mm],error[mm],P_term,I_term,D_term,Servo output angle [degrees]
'''
data_row = [] 
data_array = [] # list of data rows, each row is saved at the measurement frequency, once per period

usb_path = "/media/usb/"
now = datetime.datetime.now()

red_pin = "P9_41"
r_led = False
GPIO.setup(red_pin, GPIO.OUT)

record_sw_pin = "P9_15"
GPIO.setup(record_sw_pin, GPIO.IN)

print("ADC setup...")
time.sleep(15) # attempt to solve bootup problem
poten_pin = "P9_33"
poten_value = 0 # input range 0 - 1.0
try:
	ADC.setup()
except:
	e = sys.exc_info()[0]
	print("ERROR: ",e)
	print("retrying...")
	# try again
	time.sleep(15) # attempt to solve bootup problem
	ADC.setup()

timer_1hz = datetime.datetime.now()
timer_led = datetime.datetime.now()

epoch = datetime.datetime.utcfromtimestamp(0) # 1970 UNIX time

Control_ON = False


blink_count = 0 # counter for led indicator

# ------------- Timing setup ------------
control_freq = 100 # hz (actually means it will record a bit slower than this due to checking time without interrupts)
control_freq_micros = 1.0/control_freq*1000*1000 # number of microseconds before data grab
control_timer = datetime.datetime.now()
# --- End ----- Timing setup ------------

timer_program = datetime.datetime.now()

# ------------- Serial reading setup ------------
time.sleep(15) # attempt to solve bootup problem
ser = setupSerial()

sen_gain = 0.003384*25.4 # converts sensor reading to mm
gained_val = 0.0 # sensor reading in mm
# --- End ----- Serial reading setup ------------

watch_dog_01_count = 0 # counter for csv saving timeout

# ---------------- Mounting USB drive ----------
mounted_successfully = False
unmounted_successfully = True
failed_mount = False
mount_timer = datetime.datetime.now()
if auto_mount_usb and not os.path.isfile('/media/usb/important_text.txt'):
	mount_check = os.system('sudo mount /dev/sda1 /media/usb')
	while True:
		if mount_check == 0:
			mounted_successfully = True
			break
		elif (datetime.datetime.now()-mount_timer).seconds > 15:
			print('failed to mount')
			failed_mount = True
			break
# ------ End ----- Mounting USB drive ----------

# ------------- Servo output setup -------------
# 1-HK 15138, 2-HS-815BB (ball tilt servo), 3-DS3218mg (wing)"soft" limits, 4-Moth servo
Servo_Type = 4
duty_min, duty_max = servo_parameters(Servo_Type) # parameters for various kroova servos stored in the function
duty_span = duty_max - duty_min

servo_pin = "P8_13"
if enable_servo: PWM.start(servo_pin, (duty_max-duty_min)/2.0+duty_min, 60)

control_servo_angle = 90 # initial servo output angle
# --- End ----- Servo output setup ------------

# ------------- CONTROL VARIABLES setup ------------
servo_max = 107
servo_min = 69

US_input_array = []
US_rolling_avg_window = 30
averaged_US_input = 200 # some initial

# Gains
P_gain = -0.05
I_gain = 0.0 # -0.0002 # decent starting parameters
D_gain = 0.0 # -0.1 # decent starting parameters, might need stronger of all..

I_max = 10 # based on full range of flap motion being ~ 25

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

while True:
	# recording running switch
	if GPIO.input(record_sw_pin): Control_ON = True
	else: Control_ON = False


	# -------- Ultrasonic serial reading --------
	if enable_servo:
		# -------- Ultrasonic serial reading --------
		# reading at full speed seems to minimize error values
		rval = ToughSonicRead(ser)
		if US_min_thresh < rval*sen_gain < US_max_thresh:
			gained_val = rval * sen_gain # sensor reading in mm
		elif rval*sen_gain > US_max_thresh and abs(averaged_US_input-rval*sen_gain) < 200: # this allows the max thresh to be higher if the avg values are high
			gained_val = rval * sen_gain # sensor reading in mm
		# else gained val doesn't update and is equal to the last reading 
	else:
		# reading at full speed seems to minimize error values
		rval = ToughSonicRead(ser)
		gained_val = rval * sen_gain # sensor reading in mm
	# --End -- Ultrasonic serial reading --------


	if not Control_ON:
		## -------------- LED state indicator -----------
		if (datetime.datetime.now() - timer_1hz).seconds >= 1:
			timer_1hz = datetime.datetime.now() # resets the timer
			r_led = not r_led
		## --- End ------ LED state indicator -----------

		# --------------- Saving csv data file ------------
		# save any data that may have come from a stopped recording session
		if len(data_array) > 500 and saving_data:
			print("saving after recording stop")
			now = datetime.datetime.now()
			f_name = (usb_path+"Moth_Data_" + str(now.year)+"-"+str(now.month)+"-"+str(now.day)
			         +"_"+str(now.hour)+"h"+str(now.minute)+"m"+str(now.second)+"s")
			print(f_name)
			data_array = np.asarray(data_array)
			np.savetxt((f_name+".csv"),data_array,delimiter=",")
			watch_dog_01 = datetime.datetime.now()
			while not os.path.isfile(f_name+".csv"):
				if (datetime.datetime.now() - watch_dog_01).seconds > 5:
					print("ERROR in watch_dog_01")
					watch_dog_01 = datetime.datetime.now()
					watch_dog_01_count += 1 # increment the error flag count
			data_array = [] # reset the data array now that it's been saved

			# ---------------- Unmounting USB drive ----------
			mount_timer = datetime.datetime.now()
			if auto_mount_usb and os.path.isfile('/media/usb/important_text.txt'):
				unmounted_successfully = False
				mount_check = os.system('sudo umount /media/usb')
				while True:
					if mount_check == 0:
						unmounted_successfully = True
						break
					elif (datetime.datetime.now()-mount_timer).seconds > 15:
						failed_mount = True
						print('failed to unmount')
						break
			# ------ End ----- Unmounting USB drive ----------
		# ------ End ----- Saving csv data file ------------



	elif Control_ON:
		# ---------------- Mounting USB drive ----------
		if auto_mount_usb and not os.path.isfile('/media/usb/important_text.txt') and not failed_mount:
			mount_timer = datetime.datetime.now()
			mounted_successfully = False
			mount_check = os.system('sudo mount /dev/sda1 /media/usb')
			while True:
				if mount_check == 0:
					mounted_successfully = True
					break
				elif (datetime.datetime.now()-mount_timer).seconds > 15: # not sure this is written correctly...
					failed_mount = True
					print('failed to mount')
					break
		# ------ End ----- Mounting USB drive ----------

		## -------------- LED state indicator -----------
		if blink_count == 0 and (datetime.datetime.now() - timer_led).microseconds/1000 >= 600 :
			timer_led = datetime.datetime.now() # resets the timer
			blink_count += 1 
			r_led = False 
			
		elif blink_count > 0 and (datetime.datetime.now() - timer_led).microseconds/1000 >= 200 :
			timer_led = datetime.datetime.now() # resets the timer
			blink_count += 1 
			r_led = not r_led 
		if blink_count >= 5: blink_count = 0
		## --- End ------ LED state indicator -----------

		# ------------- Log and control at specified interval ----------------
		if (datetime.datetime.now() - control_timer).microseconds >= control_freq_micros:
			control_timer = datetime.datetime.now() # resets the timer

			# ---------------- CONTROLS SECTION -----------------------------
			US_input_array.append(gained_val)
			if len(US_input_array) >= US_rolling_avg_window:
				US_input_array.pop(0)
				averaged_US_input = sum(US_input_array)/len(US_input_array)
				gained_val =  averaged_US_input # averaged US input

			error = int(gained_val)-target_RH

			P_term =  error*P_gain

			# second part may be unnecessary thresholding, but the goal is to fix weird capsize I term skews 
			if ((servo_min < control_servo_angle < servo_max) and (-I_max < sum_error*I_gain < I_max)): sum_error = error + sum_error
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
			if enable_servo: PWM.set_duty_cycle(servo_pin, duty)
			# ------- END -------- CONTROLS SECTION -------------------------------

			# -------------------- LOGGING SECTION -------------------------------
			data_row = []

			unix_time_stamp = ((datetime.datetime.now()-epoch)-datetime.timedelta(hours=7)).total_seconds()
			data_row.append(unix_time_stamp)

			poten_value = ADC.read(poten_pin)
			poten_value = ADC.read(poten_pin) # read twice due to possible known ADC driver bug
			data_row.append(poten_value)

			data_row.append(gained_val)
			# new values
			data_row.append(target_RH)
			data_row.append(error)
			data_row.append(P_term)
			data_row.append(I_term)
			data_row.append(D_term)
			data_row.append(control_servo_angle)

			if saving_data: data_array.append(data_row)
			# ------- END -------- LOGGING SECTION -------------------------------
		# ---- End ---- Sensor readings at specified interval ----------------

		# --------------- Saving csv data file ------------
		# save any data that may have come from a stopped recording session
		if len(data_array) > 200000 and saving_data: # roughly 33 minutes of data if recording at 100 hz
			print("saving after 200000 row limit")
			now = datetime.datetime.now()
			f_name = (usb_path+"Moth_Data_" + str(now.year)+"-"+str(now.month)+"-"+str(now.day)
			         +"_"+str(now.hour)+"h"+str(now.minute)+"m"+str(now.second)+"s")
			print(f_name)
			data_array = np.asarray(data_array)
			np.savetxt((f_name+".csv"),data_array,delimiter=",")
			watch_dog_01 = datetime.datetime.now()
			while not os.path.isfile(f_name+".csv"):
				if (datetime.datetime.now() - watch_dog_01).seconds > 5:
					print("ERROR in watch_dog_01")
					watch_dog_01 = datetime.datetime.now()
					watch_dog_01_count += 1 # increment the error flag count
			data_array = [] # reset the data array now that it's been saved
		# ------ End ----- Saving csv data file ------------


	# --------------- Outputs ----------------
	if r_led: GPIO.output(red_pin,GPIO.HIGH)#print("red on")
	else: GPIO.output(red_pin,GPIO.LOW)#print("red off")
	# ------ End ---- Outputs --------------


# clean up
ser.close()
PWM.stop(servo_pin)
PWM.cleanup()
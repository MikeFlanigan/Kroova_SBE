# ---------- Program Mode Controls ---------------
PC_testing = False
saving_data = True
Ultrasonic_enable = True
auto_mount_usb = False
# -- End --- Program Mode Controls ---------------

if not PC_testing: 
	import Adafruit_BBIO.GPIO as GPIO
	import Adafruit_BBIO.ADC as ADC
import serial
import datetime
import numpy as np 
import os

'''
row of data to be saved to a csv, format: 
microseconds since program start, unix time (converted to PST tho), potentiometer [0-1.0],ultrasonic [mm]
'''
data_row = [] 
data_array = [] # list of data rows, each row is saved at the measurement frequency, once per period

usb_path = "/media/usb/"
now = datetime.datetime.now()

red_pin = "P9_41"
r_led = False
if not PC_testing:
	GPIO.setup(red_pin, GPIO.OUT)

record_sw_pin = "P9_15"
if not PC_testing: GPIO.setup(record_sw_pin, GPIO.IN)

poten_pin = "P9_33"
poten_value = 0 # input range 0 - 1.0
if not PC_testing: ADC.setup()


timer_1hz = datetime.datetime.now()
timer_led = datetime.datetime.now()

epoch = datetime.datetime.utcfromtimestamp(0) # 1970 UNIX time

Recording = False

blink_count = 0 # counter for led indicator

recording_freq = 100 # hz
recording_freq_micros = 1.0/recording_freq*1000*1000 # number of microseconds before data grab
timer_data_save = datetime.datetime.now()

timer_program = datetime.datetime.now()

# ------------- Serial reading setup ------------
if Ultrasonic_enable:
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

while True:
	# recording running switch
	if GPIO.input(record_sw_pin): Recording = True
	else: Recording = False

	# ---------- TESTING ONLY CODE -----------------
	# if (datetime.datetime.now()-timer_program).seconds > 20:
	# 	timer_program = datetime.datetime.now() 
	# 	Recording = not Recording
	# 	print("Recording: ",Recording)
	# --- End -- TESTING ONLY CODE -----------------

	if not Recording:
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



	elif Recording:
		# ---------------- Mounting USB drive ----------
		if auto_mount_usb and not os.path.isfile('/media/usb/important_text.txt') and not failed_mount:
			mount_timer = datetime.datetime.now()
			mounted_successfully = False
			mount_check = os.system('sudo mount /dev/sda1 /media/usb')
			while True:
				if mount_check == 0:
					mounted_successfully = True
					break
				elif (datetime.datetime.now()-mount_timer).seconds > 15:
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

		# ------------- Sensor readings at specified interval ----------------
		if (datetime.datetime.now() - timer_data_save).microseconds >= recording_freq_micros:
			timer_data_save = datetime.datetime.now() # resets the timer
			data_row = []

			unix_time_stamp = ((datetime.datetime.now()-epoch)-datetime.timedelta(hours=7)).total_seconds()
			data_row.append(unix_time_stamp)
			data_row.append((datetime.datetime.now()-timer_program).microseconds)

			poten_value = ADC.read(poten_pin)
			poten_value = ADC.read(poten_pin) # read twice due to possible known ADC driver bug
			# print(poten_value)
			data_row.append(poten_value)

			# -------- Ultrasonic serial reading --------
			if Ultrasonic_enable:
				for c in ser.read():
					if c == '\r':
						# print(line)
						line = []

						try:
							ival = int(val)
						except ValueError:
							print("unexpected ultrasonic value: ", val)
						rval = float(ival)
						val = ''
						break
					else:
						line.append(c)
						val = val + c
				gained_val = rval * sen_gain # sensor reading in mm
            # --End -- Ultrasonic serial reading --------
			data_row.append(gained_val)

			data_array.append(data_row)
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


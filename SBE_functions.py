'''
SBE functions
'''
import serial 

# line = []
val = ''
rval = 0.0
ival =  0

def ToughSonicRead(lobj):
	global line, val, rval, ival
	for c in lobj.read():
		if c == '\r':
			line = []
			try:
				ival = int(val)
			except ValueError:
				print("unexpected ultrasonic value: ", val) # may want to update this to return the unexpected value

			rval = float(ival)
			val = ''
			lobj.reset_input_buffer() ## critical if this logger is running slower or out of sync with the sensor...
			break
		else:
			# line.append(c)
			val = val + c
	return rval

def setupSerial():
	lobj = serial.Serial(
	   port = '/dev/ttyUSB0',
	   baudrate = 9600,
	   parity = serial.PARITY_NONE,
	   stopbits = serial.STOPBITS_ONE,
	   bytesize = serial.EIGHTBITS,
	       timeout = 0)
	print("connected to: ")
	print(lobj.portstr)
	return lobj

def servo_parameters(S_enum):
	# 1-HK 15138, 2-HS-815BB (ball tilt servo), 3-DS3218mg (wing)"soft" limits, 4-Moth servo
	if S_enum == 1:
	    ## HK 15138
	    duty_min = 3.5 
	    duty_max = 14.0 
	elif S_enum == 2:
	    ## HS-815BB (ball tilt servo)
	    duty_min = 7.5 
	    duty_max = 11.25 
	elif S_enum == 3:
	    ## DS3218mg (wing)
	    # note these two are "soft" limits based on the wing build and desired mechanical limits
	    duty_min = 6.0 
	    duty_max = 11.0  
	elif S_enum == 4:
	    # Moth servo
	    duty_min = 5.5
	    duty_max = 14.0

	return duty_min, duty_max
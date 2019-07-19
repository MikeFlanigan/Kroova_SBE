

import datetime
import time 
import numpy as np 
import os
import sys
import Adafruit_BBIO.UART as UART
import serial
import Adafruit_BBIO.ADC as ADC
import Adafruit_BBIO.GPIO as GPIO
from SBE_funcs import *

BB_SW1_Pin = "P8_8" # start stop recording
BB_SW1 = False
GPIO.setup(BB_SW1_Pin, GPIO.IN)

BB_LED1_Pin = "P8_14" # indicates logging is running 
BB_LED2_Pin = "P8_16"
BB_LED3_Pin = "P8_18"

BB_LED1 = False # indicates logging is running 
BB_LED2 = False
BB_LED3 = False

GPIO.setup(BB_LED1_Pin, GPIO.OUT)
GPIO.setup(BB_LED2_Pin, GPIO.OUT)
GPIO.setup(BB_LED3_Pin, GPIO.OUT)

log_freq = 60 # Hz
log_per = 1/log_freq*1000 # milliseconds
timer_log = datetime.datetime.now()

timer_1hz = datetime.datetime.now()
timer_10hz = datetime.datetime.now()
Hz_1 = False
Hz_10 = False

Recording = False  
record_edge = False

session_minutes = 3.5 # minutes
record_sesh_length = 60*session_minutes # seconds

data_row = [] 
data_array = [] # list of data rows, each row is saved at the measurement frequency, once per period

usb_path = "/media/usb/"

debug_data_checking = False

print("ADC setup...")
# UPDATE !!!!!!!
print('DEBUGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGG')
##time.sleep(15) # attempt to solve bootup problem
poten_pin = "P9_40"
poten_value = 0 # input range 0 - 1.0
try: ADC.setup()
except:
    print("ADC error.")
    e = sys.exc_info()[0]
    print("ERROR: ",e)
    print("retrying...")
    time.sleep(15) # attempt to solve bootup problem
    ADC.setup()
	
UART.setup("UART1") # RX P9_26 TX P9_24
##ser = serial.Serial(port = "/dev/ttyO1", baudrate=9600)
ser = serial.Serial(port = "/dev/ttyO1", baudrate=115200, timeout = 0.0005)
buffer = ""
US_dist = 0

init_time = datetime.datetime.now()
rows = 0

ascii_err_count = 0

seconds = 0
last_seconds = 0

rec_blink_count = 0

time_stop = False

timer_recording = datetime.datetime.now()
# ------------------ Function nn def ----------------------

# ------ End of ---- Function nn def ----------------------
speed_check = datetime.datetime.now()

GPS_spd = 0.0
IMU_RH_acc_unb = 0.0
Heel = 0.0
Pitch = 0.0
try:
    while True:
        now = datetime.datetime.now()

        # ------------------ 1 Hz timer ----------------------
        seconds = (now - timer_1hz).microseconds/1000000
        if abs(last_seconds - seconds) >= 0.300 : # detects the roll over edge in the microseconds timer
            timer_1hz = now # resets the timer
            Hz_1 = True # only on for a single pass once a second
        else: Hz_1 = False
        last_seconds = seconds
        # ------ End of ---- 1 Hz timer ----------------------
        # ------------------ 10 Hz timer ----------------------
        if abs(now - timer_10hz).microseconds/1000000 >= 0.100 :
            timer_10hz = now # resets the timer
            Hz_10 = True # only on for a single pass once a second
        else:
            Hz_10 = False
        # ------ End of ---- 10 Hz timer ----------------------
        
        # ------------------ Collect Data ----------------------
        if GPIO.input(BB_SW1_Pin): BB_SW1 = True     
        else:
            BB_SW1 = False
            Recording = False
            time_stop = False
            timer_recording = datetime.datetime.now()
            
        if BB_SW1 and not Recording and not time_stop: Recording = True

        # always listening for US data from the Arduino
        oneByte = ser.read(1)
        if oneByte == b"\r":
            
            buffer = "".join(buffer.splitlines())
            
##            GPS_hr, GPS_min, GPS_sec, GPS_spd, GPS_lat, GPS_lon, US_dist, RHA, IMU_RH_offset, IMU_RH_acc, Heel, Pitch = buffer.split(",")
##            print(GPS_hr, GPS_min, GPS_sec, GPS_spd, GPS_lat, GPS_lon, US_dist, RHA, IMU_RH_offset, IMU_RH_acc, Heel, Pitch)
##            print(US_dist,' ',RHA)
            
            GPS_spd, US_dist, IMU_RH_acc_unb, Heel, Pitch = buffer.split(",")
##            print(GPS_spd, US_dist, IMU_RH_acc_unb, Heel, Pitch)

##            speed_check = datetime.datetime.now()
            buffer = ""
            ser.reset_input_buffer() # maybe return to greatness
##            print(US_dist,' ','elapsed: ', (datetime.datetime.now() - speed_check).microseconds)

        else:
            try:
                buffer += oneByte.decode("ascii")
            except UnicodeDecodeError as e:
                ascii_err_count += 1 # figure out how often this is happening
                print('UnicodeDecodeError: ',e)

        # always reading the potentiometer value
        poten_value = ADC.read(poten_pin)
        poten_value = ADC.read(poten_pin) # listed bug by Adafruit requiring double read
        # ------ End of ---- Collect Data ----------------------
        # if recording -----------
        if Recording:
            if record_edge != Recording:
                mount_usb()
                record_edge = Recording
            
            if (now - timer_log).microseconds/1000 >= log_per:
                timer_log = now
                rows += 1
                # append more data
                data_row = []
                data_row.append(now.microsecond) 
                data_row.append(poten_value)
                
                try: data_row.append(int(US_dist))
                except ValueError: print('US_ dist WRONG TYPE!!')
                try:data_row.append(float(GPS_spd))
                except ValueError: print('gps spd WRONG TYPE!!')
                try:data_row.append(float(IMU_RH_acc_unb))
                except ValueError: print('IMU rh WRONG TYPE!!')
                try:data_row.append(float(Heel))
                except ValueError: print('heel WRONG TYPE!!')
                try:data_row.append(float(Pitch))
                except ValueError: print('pitch WRONG TYPE!!')
                data_array.append(data_row)
                
                    
            # max session duration elapsed
            if Recording and ((now - timer_recording).seconds >= record_sesh_length):
                Recording = False
                time_stop = True
                print("breaking due to session length duration")

        if debug_data_checking:
            ts_str = "ts: " + str(now.minute)+'.'+str(now.microsecond).rjust(7)
            us_str = " US dist: " + str(US_dist).rjust(6)
            poten_str = " poten: " + str(np.round(poten_value,4)).rjust(6)
##            GPS_spd, US_dist, IMU_RH_acc_unb, Heel, Pitch
            print( ts_str + us_str + poten_str)

        if not Recording:
            timer_recording = datetime.datetime.now()
            if record_edge != Recording:
                record_edge = Recording

                f_name = (usb_path+"Moth_Data_" + str(now.year)+"-"+str(now.month)+"-"+str(now.day)
                 +"_"+str(now.hour)+"h"+str(now.minute)+"m"+str(now.second)+"s")
                print(f_name)
                data_array = np.asarray(data_array)
                np.savetxt((f_name+".csv"),data_array,delimiter=",")
                data_array = []

                if os.path.isfile(f_name+".csv"):
                    unmount_usb()
                else:
                    print('pause for file save...')
                    time.sleep(1)
                    unmount_usb()


        # ------------------ LED States ----------------------
        if Recording and Hz_1:
            rec_blink_count = 0
        if Recording and Hz_10:
            rec_blink_count += 1
            if rec_blink_count == 1 or rec_blink_count == 3 or rec_blink_count == 4 or rec_blink_count == 6:
                BB_LED1 = not BB_LED1
        if not Recording:
            if Hz_1: BB_LED2 = not BB_LED2
            BB_LED1 = False
        else: BB_LED2 = False
        # ------ End of ---- LED States ----------------------
        
        # ------------------ BeagleBone Outputs ----------------------
        if BB_LED1: GPIO.output(BB_LED1_Pin,GPIO.HIGH)
        else: GPIO.output(BB_LED1_Pin,GPIO.LOW)
        if BB_LED2: GPIO.output(BB_LED2_Pin,GPIO.HIGH)
        else: GPIO.output(BB_LED2_Pin,GPIO.LOW)
        # ------ End of ---- BeagleBone Outputs ----------------------

        # try straight ditching this
##        time.sleep(0.001) # rest the loop 1 millisecond.... is this bad or good practice? 
        
except KeyboardInterrupt:
    print('keyboard interrupt')
    pass
##except Exception as e:
##    # log errors here
##    print(e)
##    print('error')
##    pass

# UPDATE !!!!!!!
elapsed = (datetime.datetime.now()-init_time).seconds + (datetime.datetime.now()-init_time).microseconds/1000000
##print(elapsed)
##print(elapsed*60)
##print(rows)

print("NUMBER OF ASCII DECODING ERRORS: ", ascii_err_count)
# ------------------ Clean up hardware/ports ----------------------
ser.close()
GPIO.output(BB_LED1_Pin,GPIO.LOW)
GPIO.output(BB_LED2_Pin,GPIO.LOW)
GPIO.output(BB_LED3_Pin,GPIO.LOW)
GPIO.cleanup()
UART.cleanup()
unmount_usb()
# ------ End of ---- Clean up hardware/ports ----------------------

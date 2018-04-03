########################################################

# Top level program here, combining read and sweep

import serial
import RPi.GPIO as GPIO
import time
import datetime
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt


# user settings
debug_mode = False
enable_output = True

# gains
P_gain = 0.95 # proportional gain
I_gain = 0.0 # integral gain
D_gain = 0.0 # derivative gain

# parameters
servo_max = 175 # degrees
servo_min = 5 # degrees

# timing setup
delta = 0
last_time = datetime.datetime.now()
loop_time_last = 0.0 # ms
loop_time_average = 0.0 # ms
loop_times = [] # list of last n loop times
num_loops_avg = 100
oneshot = True # flag to do something once in the code

# setup to run for t seconds
start_time = datetime.datetime.now()
run_for_fixed_time = True
duration = 5

# test logging
enable_logging = True
dist_log = []

# servo output setup
if enable_output:
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(18, GPIO.OUT)
    pwm = GPIO.PWM(18, 100) # sets PWM freq to 100 Hz
    pwm.start(5)

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

error = 0 # 10x the error in mm
target_RH = 550 # mm
target_aoa = output_angle

while True:
    try:
        # ------------- timing code -------------
        delta = datetime.datetime.now() - last_time
        last_time = datetime.datetime.now()
        loop_time_last = float(delta.microseconds)/float(1000)
        if len(loop_times) < num_loops_avg:
            loop_times.insert(0,loop_time_last)
        elif oneshot == True:
            oneshot = False
            print("queue filled")
            loop_times.pop(num_loops_avg-1)
            loop_times.insert(0,loop_time_last)
        else:
            loop_times.pop(num_loops_avg-1)
            loop_times.insert(0,loop_time_last)
            
##        print(loop_time_last," milliseconds")

        if run_for_fixed_time:
            if (datetime.datetime.now() - start_time).seconds > duration:
                print("run duration reached")
                break
        # ------------- END OF timing code -------------
        # ------------- logging code -------------------
        dist_log.append(gained_val)

        # ------ END OF logging code -------------------
        
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
##        print(error)

        target_aoa = error*P_gain # control equation
        output_angle = target_aoa + 90 # convert aoa to absolute angle for servo

        # threshold servo commands in case of errors
        if output_angle > servo_max: output_angle = servo_max
        elif output_angle < servo_min: output_angle = servo_min
        

        if enable_output:
            duty = float(output_angle) / 10.0 + 2.5 # convert angle 0-180 to duty cycle percent
            pwm.ChangeDutyCycle(duty) # send servo pwm duty cycle command

    except KeyboardInterrupt: # allows for easy program stop by tester
        break
    
# clean up
ser.close()
if enable_output: pwm.stop()

# timing wrap up
total = 0 # total time for n loops
for lap in loop_times:
    total = total + lap
loop_time_average = total/num_loops_avg
print("average loop time: ",loop_time_average)

# plot logs 
if enable_logging:
    plt.plot(dist_log)
    plt.plot([0,len(dist_log)-1],[target_RH,target_RH])
    plt.xlabel('code loop iteration')
    plt.ylabel('distance [mm]')
    plt.grid(True)
    axis1 = plt.gca()
    axis1.set_ylim([150,1350])
    plt.show()
    plt.pause(0.0001)

########################################################

# just a serial read and print below 

##import serial
##import datetime
##
### timing setup
##delta = 0
##last_time = datetime.datetime.now()
##loop_time_last = 0.0 # ms
##loop_time_average = 0.0 # ms
##loop_times = [] # list of last n loop times
##num_loops_avg = 100
##oneshot = True # flag to do something once in the code
##
##ser = serial.Serial(
##    port = '/dev/ttyUSB0',
##    baudrate = 9600,
##    parity = serial.PARITY_NONE,
##    stopbits = serial.STOPBITS_ONE,
##    bytesize = serial.EIGHTBITS,
##        timeout = 0)
##
##print("connected to: ")
##print(ser.portstr)
##
##line = []
##val = ''
##rval = 0
##
##
##while True:
##    
##    try:
##        # ------------- timing code -------------
##        delta = datetime.datetime.now() - last_time
##        last_time = datetime.datetime.now()
##        loop_time_last = float(delta.microseconds)/float(1000)
##        if len(loop_times) < num_loops_avg:
##            loop_times.insert(0,loop_time_last)
##        elif oneshot == True:
##            oneshot = False
##            print("queue filled")
##            loop_times.pop(num_loops_avg-1)
##            loop_times.insert(0,loop_time_last)
##        else:
##            loop_times.pop(num_loops_avg-1)
##            loop_times.insert(0,loop_time_last)
##            
####        print(loop_time_last," milliseconds")
##        # ------------- END OF timing code -------------
##        
##        
##        for c in ser.read():
##            if c == '\r':
####                print(line)
##                line = []
##                val = ''
##                break
##            else:
##                line.append(c)
##                val = val + c
##    except KeyboardInterrupt:
##        break
##ser.close()
##print("closed")
##
### timing wrap up
##total = 0 # total time for n loops
##for lap in loop_times:
##    total = total + lap
##loop_time_average = total/num_loops_avg
##print("average loop time: ",loop_time_average)

########################################################

## useful Tkinter slider bar GUI below

##from Tkinter import *
##import RPi.GPIO as GPIO
##import time
##
##GPIO.setmode(GPIO.BCM)
##GPIO.setup(18, GPIO.OUT)
##pwm = GPIO.PWM(18, 100)
##pwm.start(5)
##
##class App:
##
##    def __init__(self, master):
##        frame = Frame(master)
##        frame.pack()
##        scale = Scale(frame, from_=0, to=180,
##              orient=HORIZONTAL, command=self.update)
##        scale.grid(row=0)
##
##
##    def update(self, output_angle):
##        duty = float(output_angle) / 10.0 + 2.5
##        pwm.ChangeDutyCycle(duty)
##
##root = Tk()
##root.wm_title('Servo Control')
##app = App(root)
##root.geometry("200x50+0+0")
##root.mainloop()
##pwm.stop()

########################################################
## just a shaky servo sweep below

##import RPi.GPIO as GPIO
##import time
##
##GPIO.setmode(GPIO.BCM)
##GPIO.setup(18, GPIO.OUT)
##pwm = GPIO.PWM(18, 100) # sets PWM freq to 100 Hz
##pwm.start(5)
##
##output_angle = 0
##change = 1
##while True:
##    time.sleep(.1)
##    duty = float(output_angle) / 10.0 + 2.5
##    pwm.ChangeDutyCycle(duty)
##    output_angle = output_angle + change
##    if output_angle >= 180:
##        change = -1
##    elif output_angle <= 0:
##        change = 1
##        
##    print(output_angle)
##pwm.stop()

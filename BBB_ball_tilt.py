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
Servo_Type = 2 # enummerator for servos --- 1 = Hobby king cheap, 2 = Ball tilt, 3 = Car wing
set_RH_w_pot = False # use a potentiometer to set the ride height
add_noise_to_RH = True # add varying amounts of constant disturbances to the RH 

# gains
# P_gain = 0.225#0.130#0.0625#0.25 # proportional gain
# I_gain = 0.0002#0.00015#0.0002 # integral gain
# D_gain = 5.5#2.85#135.0 # derivative gain

# Need to test control loop speed effects, maybe run everything on timed intervals to make independent of faster loops 

# Competition settings, sensitive to measurement noise but strong fast control 
# P_gain = 0.34#0.24
# I_gain = 0.0005#0.00030
# D_gain = 7.9#5.5#5.5

# Good set of gains below
# handle static SP well and analog noise decently
# encounters an area of unstable oscillations in the D term between SP 400 - 700 sometimes
P_gain = 0.24 
I_gain = 0.0002 #.0002
D_gain = 1.75 #1.75#3.00 # was 3.75


error = 0 # 10x the error in mm
last_error = 0 # for storing errors from previous loop
sum_error = 0 # integral term of error

D_read_Hz = 20 # Read derivative changes at 100x per second. This can be tuned to roughly match the dynamics of the system.
D_read_ms = 1.0/D_read_Hz*1000 # time in ms between each derivative read
last_derivative_read = datetime.datetime.now()
last_derivate_error = 0
rolling_avg_D_errors = [0]*50 # 50 seems good 

# initialization
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

# Noise
noise_range = 50 # mm - looking to get to 200 mm
wave_freq = 250 # ms
wave_noise_timer = datetime.datetime.now()
temp_wave = 0 # wave size parameter

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
run_for_fixed_time = False
duration = 5

# test logging
enable_logging = False
dist_log = []


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
        # ------------- pot code --------------
        if set_RH_w_pot:
            print(ADC.read(poten_pin))
        # poten_values.append(ADC.read("P9_40"))
        # poten_values.pop(0)
        # poten_value = sum(poten_values)/len(poten_values)
        # target_RH = int(poten_value/0.4*800+300)
        # print(target_RH)

        # ------------- Ride Height Noise Code -----
        # adding noise to the RH should provide similar disturbances to the type of noise the system will experience due 
        # to wave chop 
        if add_noise_to_RH:
            if (datetime.datetime.now() - wave_noise_timer).microseconds/1000 > wave_freq:
                wave_noise_timer = datetime.datetime.now()
                # generate an incoming wave 
                temp_wave = random.randint(0,noise_range)
                if target_RH == target_RH_init: target_RH = target_RH + temp_wave
                else: target_RH = target_RH_init

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
        ser.reset_input_buffer() ## super critical if this logger is running slower or out of sync with the sensor...
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
        # print(error)
        
        P_term =  error*P_gain

        # I_term = (error + I_term)*I_gain 
        if (target_aoa < 60 and target_aoa > -60): sum_error = error + sum_error
        else: sum_error = sum_error
        I_term = sum_error*I_gain 


        # if (datetime.datetime.now() - last_derivative_read).microseconds/1000 >= D_read_ms:
        #     D_term = (error - last_derivate_error)*D_gain
        #     last_derivate_error = error # update the derivate error 
        #     last_derivative_read = datetime.datetime.now() # update the derivative read time

        rolling_avg_D_errors.append(error)
        rolling_avg_D_errors.pop(0)
        # D_term = (error - float(sum(rolling_avg_D_errors))/len(rolling_avg_D_errors))*D_gain

        D_term = (error - float(sum(rolling_avg_D_errors))/len(rolling_avg_D_errors))*D_gain
        # if P_term > 60: D_term = (error - float(sum(rolling_avg_D_errors))/len(rolling_avg_D_errors))*D_gain*1.25
        # else: D_term = (error - float(sum(rolling_avg_D_errors))/len(rolling_avg_D_errors))*D_gain

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

    except KeyboardInterrupt: # allows for easy program stop by tester
        break
    
# clean up
ser.close()
if enable_output: 
    PWM.stop(servo_pin)
    PWM.cleanup()

# timing wrap up
total = 0 # total time for n loops
for lap in loop_times:
    total = total + lap
loop_time_average = total/num_loops_avg
print("average loop time: ",loop_time_average," ms")


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
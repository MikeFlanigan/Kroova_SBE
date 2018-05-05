import serial
import Adafruit_BBIO.PWM as PWM
import Adafruit_BBIO.GPIO as GPIO
import time
import datetime
import Adafruit_BBIO.ADC as ADC


# user settings
debug_mode = False
enable_output = True

# gains
# P_gain = 0.225#0.130#0.0625#0.25 # proportional gain
# I_gain = 0.0002#0.00015#0.0002 # integral gain
# D_gain = 5.5#2.85#135.0 # derivative gain

# Need to test control loop speed effects, maybe run everything on timed intervals to make independent of faster loops 

# Competition settings, sensitive to measurement noise but strong fast control 
# P_gain = 0.34#0.24
# I_gain = 0.0005#0.00030
# D_gain = 7.9#5.5#5.5
enable_competition_foolery = False

# Good set of gains below
# handle static SP well and analog noise decently
# encounters an area of unstable oscillations in the D term between SP 400 - 700 sometimes
P_gain = 0.24 
I_gain = 0.0002#.0002
D_gain = 3.7#1.75#3.00 # was 3.75


error = 0 # 10x the error in mm
last_error = 0 # for storing errors from previous loop
sum_error = 0 # integral term of error

memory_weight_D_error = 0.96 # a larger value increases the value of old derivatives over new derivatives 
new_D_gain = 30 # magnifies non zero derivative errors to help add realistic inertia ?? ... 
D_read_Hz = 20 # Read derivative changes at 100x per second. This can be tuned to roughly match the dynamics of the system.
D_read_ms = 1.0/D_read_Hz*1000 # time in ms between each derivative read
last_derivative_read = datetime.datetime.now()
last_derivate_error = 0
rolling_avg_D_errors = [0]*50 # 50 seems good 


P_term = 0
I_term = 0
D_term = 0

target_RH = 647 # mm
output_angle = 90 # initial servo output angle
target_aoa = output_angle

# Analog reading 
ADC.setup()
pot_value = 0
pot_values = [target_RH]*1000 # rolling average the analog read to smooth it out

# parameters
servo_max = 180 # degrees
servo_min = 0 # degrees

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
Servo_Type = 3 # enummerator for servos 
if Servo_Type == 1:
    ## HK 15138
    duty_min = 3.5 
    duty_max = 14.0 
elif Servo_Type == 2:
    ## HS-815BB (ball tilt servo)
    duty_min = 7.5 
    duty_max = 11.25 
elif Servo_Type == 3:
    ## DS3218mg (wing)
    # note these two are "soft" limits based on the wing build and desired mechanical limits
    duty_min = 6.0 
    duty_max = 11.0  

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
        # pot_values.append(ADC.read("P9_40"))
        # pot_values.pop(0)
        # pot_value = sum(pot_values)/len(pot_values)
        # target_RH = int(pot_value/0.4*800+300)
        # print(target_RH)


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
        # print(error)
        
        P_term =  error*P_gain

        # I_term = (error + I_term)*I_gain 
        if (target_aoa < 60 and target_aoa > -60): sum_error = error + sum_error
        else: sum_error = sum_error
        I_term = sum_error*I_gain 

        # D_term = (1-memory_weight_D_error)*((error - last_error)*D_gain)+memory_weight_D_error*D_term

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

if competiton_scoring:
    GPIO.cleanup()

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
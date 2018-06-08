
# just a serial read and print below 
import datetime
from SBE_functions import *
import sys


pyth_vers = sys.version_info[0]

# timing setup
delta = 0
last_time = datetime.datetime.now()
loop_time_last = 0.0 # ms
loop_time_average = 0.0 # ms
loop_times = [] # list of last n loop times
num_loops_avg = 100
oneshot = True # flag to do something once in the code


ser = setupSerial()

if pyth_vers == 2: raw_input("press enter to continue...")
elif pyth_vers == 3: input("press enter to continue...")

line = []
val = ''
rval = 0.0
ival =  0

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
       # ------------- END OF timing code -------------
        rval = ToughSonicRead(ser)
        print(rval)

    except KeyboardInterrupt:
        print("keyinterrupt")
        break
ser.close()
print("closed")

# timing wrap up
total = 0 # total time for n loops
for lap in loop_times:
    total = total + lap
loop_time_average = total/num_loops_avg
print("average loop time: ",loop_time_average)
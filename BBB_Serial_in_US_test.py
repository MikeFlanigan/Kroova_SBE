
# just a serial read and print below 

import serial
import datetime

# timing setup
delta = 0
last_time = datetime.datetime.now()
loop_time_last = 0.0 # ms
loop_time_average = 0.0 # ms
loop_times = [] # list of last n loop times
num_loops_avg = 100
oneshot = True # flag to do something once in the code

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
rval = 0


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
       
       # print(ser.read(), "= that thing")
       for c in ser.read():
       	   # print(c)
           if c == '\r':
               print(line)
               line = []
               val = ''
               break
           else:
               line.append(c)
               val = val + c
   except KeyboardInterrupt:
       break
ser.close()
print("closed")

# timing wrap up
total = 0 # total time for n loops
for lap in loop_times:
   total = total + lap
loop_time_average = total/num_loops_avg
print("average loop time: ",loop_time_average)
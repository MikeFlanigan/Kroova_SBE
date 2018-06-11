import Adafruit_BBIO.PWM as PWM
from SBE_functions import *

servo_pin = "P8_13"

# 1-HK 15138, 2-HS-815BB (ball tilt servo), 3-DS3218mg (wing)"soft" limits, 4-Moth servo
Servo_Type = 4 # enummerator for servos 1
duty_min, duty_max = servo_parameters(Servo_Type) # parameters for various kroova servos stored in the function

duty_span = duty_max - duty_min

# PWM.start(servo_pin, (100-duty_min), 60.0)
PWM.start(servo_pin, (duty_max-duty_min)/2+duty_min, 60.0)

angle_f = 9.5 # initiate to something in the middle
while True:
    angle = raw_input("Angle (0 to 180 x to exit):")
    if angle == 'x':
        PWM.stop(servo_pin)
        PWM.cleanup()
        break
    angle_f = float(angle)

    # comment these two lines and uncomment the below to find duty cycle limits manually
    duty = ((angle_f / 180) * duty_span + duty_min) 
    PWM.set_duty_cycle(servo_pin, duty)

  # PWM.set_duty_cycle(servo_pin,angle_f) # uncomment this line to find duty cycle limits manually
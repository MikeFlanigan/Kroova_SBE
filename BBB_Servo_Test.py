import Adafruit_BBIO.PWM as PWM

servo_pin = "P8_13"

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
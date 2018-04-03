import Adafruit_BBIO.PWM as PWM

servo_pin = "P8_13"
duty_min = 7.5 # HK 15138 = 3.5 # HS-815BB = (tilt ball) 7.5
duty_max = 11.25  # HK 15138 = 14 # HS-815BB = 11.25 
duty_span = duty_max - duty_min

# PWM.start(servo_pin, (100-duty_min), 60.0)
PWM.start(servo_pin, (duty_max-duty_min)/2+duty_min, 60.0)

while True:
    angle = raw_input("Angle (0 to 180 x to exit):")
    if angle == 'x':
        PWM.stop(servo_pin)
        PWM.cleanup()
        break
    angle_f = float(angle)
    # duty = 100 - ((angle_f / 180) * duty_span + duty_min) 
    duty = ((angle_f / 180) * duty_span + duty_min) 
    #PWM.set_duty_cycle(servo_pin,angle_f)
    PWM.set_duty_cycle(servo_pin, duty)

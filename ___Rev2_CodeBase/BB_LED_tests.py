import Adafruit_BBIO.GPIO as GPIO
import time

BB_LED1_Pin = "P8_14"
BB_LED2_Pin = "P8_16"
BB_LED3_Pin = "P8_18"

BB_LED1 = False
BB_LED2 = False
BB_LED3 = False

GPIO.setup(BB_LED1_Pin, GPIO.OUT)
GPIO.setup(BB_LED2_Pin, GPIO.OUT)
GPIO.setup(BB_LED3_Pin, GPIO.OUT)

BB_DO1_Pin = "P9_25"
BB_DO2_Pin = "P9_27"

BB_DO1 = False
BB_DO2 = False

GPIO.setup(BB_DO1_Pin, GPIO.OUT)
GPIO.setup(BB_DO2_Pin, GPIO.OUT)

try: 
    while True:
        GPIO.output(BB_LED1_Pin,GPIO.HIGH)
        GPIO.output(BB_LED2_Pin,GPIO.HIGH)
        GPIO.output(BB_LED3_Pin,GPIO.HIGH)
        print("all leds on")
        time.sleep(3)
        
        GPIO.output(BB_DO1_Pin,GPIO.HIGH)
        print("DO1 high")
        GPIO.output(BB_DO2_Pin,GPIO.LOW)
        
        GPIO.output(BB_LED1_Pin,GPIO.HIGH)#print("LED1 on")
        time.sleep(1)

        GPIO.output(BB_DO2_Pin,GPIO.HIGH)
        print("DO2 high")
        GPIO.output(BB_DO1_Pin,GPIO.LOW)
        
        GPIO.output(BB_LED1_Pin,GPIO.LOW)#print("LED1 off")
        GPIO.output(BB_LED2_Pin,GPIO.HIGH)
        time.sleep(1)
        GPIO.output(BB_LED2_Pin,GPIO.LOW)
        GPIO.output(BB_LED3_Pin,GPIO.HIGH)
        time.sleep(1)
        GPIO.output(BB_LED3_Pin,GPIO.LOW)
except KeyboardInterrupt:
    GPIO.output(BB_LED1_Pin,GPIO.LOW)
    GPIO.output(BB_LED2_Pin,GPIO.LOW)
    GPIO.output(BB_LED3_Pin,GPIO.LOW)

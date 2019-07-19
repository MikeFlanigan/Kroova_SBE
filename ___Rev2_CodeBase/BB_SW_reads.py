import Adafruit_BBIO.GPIO as GPIO

BB_SW1_Pin = "P8_8"
BB_SW2_Pin = "P8_10"

BB_SW1 = False
BB_SW2 = False

GPIO.setup(BB_SW1_Pin, GPIO.IN)
GPIO.setup(BB_SW2_Pin, GPIO.IN)

while True:
    if GPIO.input(BB_SW1_Pin): BB_SW1 = True
    else: BB_SW1 = False
    
    if GPIO.input(BB_SW2_Pin): BB_SW2 = True
    else: BB_SW2 = False

    print(BB_SW1, BB_SW2)

import Adafruit_BBIO.GPIO as GPIO
import time

red_pin = "P9_12"
blue_pin = "P9_41"

GPIO.setup(red_pin, GPIO.OUT)
GPIO.setup(blue_pin, GPIO.OUT)


for x in range(0,3):
	GPIO.output(red_pin, GPIO.HIGH)
	GPIO.output(blue_pin, GPIO.HIGH)
	time.sleep(1.5)
	GPIO.output(red_pin, GPIO.LOW)
	GPIO.output(blue_pin, GPIO.LOW)
	time.sleep(1.5)



GPIO.cleanup()


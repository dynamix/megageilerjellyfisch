import RPi.GPIO as GPIO
import time
print("trigger reset")
GPIO.setmode(GPIO.BCM)
GPIO.setup(23, GPIO.OUT, initial=GPIO.HIGH)
GPIO.output(23, GPIO.LOW)
time.sleep(0.025)
GPIO.output(23, GPIO.HIGH)

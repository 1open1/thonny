import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)
pin = 11
GPIO.setup(pin, GPIO.OUT)
pwm = GPIO.PWM(pin, 50)
pwm.start(5)
print("Waiting for 1 second...")

time.sleep(1)
duty = 2
while duty <= 17:
    pwm.ChangeDutyCycle(duty)
    time.sleep(5)
    duty = duty + 2

print("Turning back to 0 degrees")
pwm.ChangeDutyCycle(2)
time.sleep(5)
pwm.ChangeDutyCycle(0)
pwm.stop()
GPIO.cleanup()
print("Everything is cleaned")

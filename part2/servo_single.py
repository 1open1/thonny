import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)
GPIO.setup(11, GPIO.OUT)

servo = GPIO.PWM(11, 50)
servo.start(5)
print("Waiting for 1 second...")
time.sleep(1)

duty = 2

while duty <= 17:
    servo.ChangeDutyCycle(duty)
    print(f"Moving to position with duty cycle: {duty}")
    time.sleep(5)

    print("Turning back to 0 degrees")
    servo.ChangeDutyCycle(2)
    time.sleep(5)

    duty += 2

servo.ChangeDutyCycle(0)
time.sleep(0.5)
servo.stop()
GPIO.cleanup()
print("Everything is cleaned")

import RPi.GPIO as GPIO
from time import sleep

GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

control_pin = 10
GPIO.setup(control_pin, GPIO.OUT)  # connect to enable pin-1 of Driver IC

pwm = GPIO.PWM(control_pin, 50)  # to setup the pwm commands type 50Hz frequency

set_angle = int(input("Enter Servo Motor Rotation Angle: "))

pwm.start(0)
delay = 1
duty_cycle = (set_angle / 18) + 2.5

GPIO.output(control_pin, True)
pwm.ChangeDutyCycle(duty_cycle)
# GPIO.output(control_pin, True)
sleep(delay)

GPIO.output(control_pin, False)
pwm.ChangeDutyCycle(0)

pwm.stop()
GPIO.cleanup()

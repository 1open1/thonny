import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)
servo_pin = 11
GPIO.setup(servo_pin, GPIO.OUT)

servo = GPIO.PWM(servo_pin, 50)
servo.start(0)

def set_angle(angle):
    duty = 2 + (angle / 18)
    servo.ChangeDutyCycle(duty)
    time.sleep(0.5)
    servo.ChangeDutyCycle(0)

try:
    cycles = 3
    for i in range(cycles):
        print(f"Cycle {i+1}:")
        for angle in range(0, 181, 30):
            print(f"Moving to {angle} degrees")
            set_angle(angle)
        for angle in range(180, -1, -30):
            print(f"Moving to {angle} degrees")
            set_angle(angle)
    print("Completed all cycles")
except KeyboardInterrupt:
    print("\nProcess interrupted by user")
finally:
    servo.stop()
    GPIO.cleanup()
    print("GPIO cleaned up and program exited")

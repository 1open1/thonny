import RPi.GPIO as GPIO
import time

RAIN_SENSOR_PIN = 10  

def setup():
    GPIO.setmode(GPIO.BOARD)  
    GPIO.setup(RAIN_SENSOR_PIN, GPIO.IN)

def loop():
    try:
        while True:
            if GPIO.input(RAIN_SENSOR_PIN) == 0:
                print("No rain detected")
            else:
                print("Rain detected!")
            time.sleep(1)
    except KeyboardInterrupt:
        print("Program stopped")
        GPIO.cleanup()

if __name__ == '__main__':
    setup()
    loop()

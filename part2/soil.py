import RPi.GPIO as gpio
import time

channel = 37
gpio.setmode(gpio.BOARD)
gpio.setup(channel, gpio.IN)

def call_back(channel):
    if gpio.input(channel):
        print(f"Moisture Detected...")
    else:
        print(f"Moisture Not Detected...")

gpio.add_event_detect(channel, gpio.BOTH, bouncetime=300)
gpio.add_event_callback(channel, call_back)

while 1:
    time.sleep(1)

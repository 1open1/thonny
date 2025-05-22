# Interfacing DHT Sensor with LCD Screen using Raspberry Pi

from RPLCD.gpio import CharLCD  # Import LCD library
import Adafruit_DHT  # Import DHT sensor library
import RPi.GPIO as gpio  # Import GPIO library
import time  # Import time library

# GPIO setup
gpio.setwarnings(False)  # Disable warnings
gpio.setmode(gpio.BOARD)  # Use BOARD numbering mode

# Initialize LCD
lcd = CharLCD(cols=20, rows=2, pin_rs=35, pin_e=33, 
              pins_data=[40, 38, 36, 32], numbering_mode=gpio.BOARD)

# DHT sensor setup
sensor = Adafruit_DHT.DHT11  # Define sensor type
pin = 17  # GPIO pin connected to the DHT sensor

try:
    while True:
        # Read humidity and temperature
        humidity, temperature = Adafruit_DHT.read_retry(sensor, pin)

        if humidity is not None and temperature is not None:
            print(f"Humidity: {humidity:.1f}%")
            print(f"Temperature: {temperature:.1f}C")

            lcd.clear()  # Clear LCD before writing new values
            lcd.cursor_pos = (0, 0)
            lcd.write_string(f"Temp: {temperature:.1f}C")
            lcd.cursor_pos = (1, 0)
            lcd.write_string(f"Humidity: {humidity:.1f}%")
        else:
            print("Failed to get reading. Try again!")

        time.sleep(1)  # Wait for 1 second before next reading

except KeyboardInterrupt:
    print("\nExiting... Cleaning up GPIO")
    lcd.clear()
    gpio.cleanup()

import Adafruit_DHT
import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD) # Use physical pin numbering
GPIO.setwarnings(False)
GPIO.cleanup()

sensor = Adafruit_DHT.DHT11

pin = 17 # GPIO17 is connected to board P15 ---11 pin
print ("temp & humd in progress")

try:
    while(1):
        humidity, temperature = Adafruit_DHT.read_retry(sensor, pin)
        print ("Humidity ="+str(humidity))
        print ("Temperature ="+str(temperature))
        time.sleep(2)  
except KeyboardInterrupt:
    GPIO.cleanup()


# Aim:Interfacingof LEDs with Rasberry pi: single led,2 led,multiple,

# single 

import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)
GPIO.setup(40, GPIO.OUT)

try:
    while True:
        print("LED ON")
        GPIO.output(40, GPIO.HIGH)
        time.sleep(1)

        print("LED OFF")
        GPIO.output(40, GPIO.LOW)
        time.sleep(1)

except KeyboardInterrupt:
    print("\nExiting program")
    GPIO.cleanup() 

# Two LEDs Blinking Alternately
import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

LED1 = 38  # First LED
LED2 = 40  # Second LED

GPIO.setup(LED1, GPIO.OUT)
GPIO.setup(LED2, GPIO.OUT)

try:
    while True:
        print("LED1 ON, LED2 OFF")
        GPIO.output(LED1, GPIO.HIGH)
        GPIO.output(LED2, GPIO.LOW)
        time.sleep(1)

        print("LED1 OFF, LED2 ON")
        GPIO.output(LED1, GPIO.LOW)
        GPIO.output(LED2, GPIO.HIGH)
        time.sleep(1)

except KeyboardInterrupt:
    print("\nExiting program")
    GPIO.cleanup()

# multiple

import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

LED_PINS = [36, 38, 40]  # Change these pins based on your connections

# Set up all LED pins
for pin in LED_PINS:
    GPIO.setup(pin, GPIO.OUT)

try:
    while True:
        for pin in LED_PINS:
            GPIO.output(pin, GPIO.HIGH)
            print(f"LED on PIN {pin} is ON")
            time.sleep(0.5)
            GPIO.output(pin, GPIO.LOW)

except KeyboardInterrupt:
    print("\nExiting program")
    GPIO.cleanup()
import RPi.GPIO as g
from time import sleep as s
from RPLCD import *
from RPLCD.gpio import CharLCD

g.cleanup()
g.setwarnings( False)
g.setmode( g.BOARD)
g.setup( 10, g.IN)
g.setup( 35, g.OUT)

lcd = CharLCD( cols = 20,
               rows = 2,
               pin_rs = 7,
               pin_e = 11,
               pins_data = [ 40,38,36,32],
               numbering_mode = g.BOARD)

try:
    while True:
        state = g.input( 10)
        if state == 0:
            print( "Gas Detected!")
            lcd.cursor_pos = (0,0)
            lcd.write_string( "GASGAS")
            g.output( 35, 1)
        elif state == 1:
            print( "No gas")
            lcd.cursor_pos = (0,0)
            lcd.write_string( "Chill")
            g.output( 35, 0)
        s(5)
except KeyboardInterrupt:
    lcd.clear()
    g.cleanup()

import time
import RPi.GPIO as gpio  # Import Raspberry Pi GPIO library

gpio.setwarnings(False)  # Disable warnings
gpio.setmode(gpio.BOARD)  # Use physical pin numbering

gpio.setup(40, gpio.IN, pull_up_down=gpio.PUD_UP)  # Set GPIO 40 as input with pull-up resistor
gpio.setup(33, gpio.OUT)  # Set GPIO 33 as output for LED

try:
    while True:  # Infinite loop
        button_state = gpio.input(40)  # Read button state

        if button_state == False:  # If button is pressed
            gpio.output(33, False)  # Turn OFF LED
            print("Button Pressed")  # Display message
            time.sleep(0.2)  # Delay to avoid bouncing
        else:
            gpio.output(33, True)  # Turn ON LED

except:
    gpio.cleanup()  # Reset GPIO when the program is stopped    
 # Import necessary libraries
import RPi.GPIO as gpio  # Library to control Raspberry Pi GPIO pins
import time  # Library for adding delays

# Set up GPIO mode (use BOARD numbering)
gpio.setmode(gpio.BOARD)

# Define GPIO pins
button_pin = 40  # Switch is connected to GPIO 40
led1_pin = 31  # LED 1 connected to GPIO 31
led2_pin = 35  # LED 2 connected to GPIO 35

# Set up the switch as an input with a pull-up resistor
gpio.setup(button_pin, gpio.IN, pull_up_down=gpio.PUD_UP)

# Set up LEDs as output
gpio.setup(led1_pin, gpio.OUT)
gpio.setup(led2_pin, gpio.OUT)

try:
    while True:
        button_state = gpio.input(button_pin)  # Read button state

        if button_state == False:  # Button is pressed (LOW)
            gpio.output(led1_pin, True)  # Turn LED 1 ON
            gpio.output(led2_pin, True)  # Turn LED 2 ON
            print("Button Pressed - LEDs ON")
        else:  # Button is released (HIGH)
            gpio.output(led1_pin, False)  # Turn LED 1 OFF
            gpio.output(led2_pin, False)  # Turn LED 2 OFF
            print("Button Released - LEDs OFF")

        time.sleep(0.1)  # Small delay to debounce

except KeyboardInterrupt:  # Handle Ctrl+C exit
    print("\nExiting... Cleaning up GPIO")
    gpio.cleanup()  # Reset GPIO settings.  
   # Import necessary libraries
import RPi.GPIO as gpio  # Library to control Raspberry Pi GPIO pins
import time  # Library for adding delays

# Set up GPIO mode (use BOARD numbering)
gpio.setmode(gpio.BOARD)

# Define GPIO pins
button1_pin = 40  # Switch 1 connected to GPIO 40
button2_pin = 38  # Switch 2 connected to GPIO 38
led1_pin = 31  # LED 1 connected to GPIO 31
led2_pin = 35  # LED 2 connected to GPIO 35

# Set up the switches as inputs with pull-up resistors
gpio.setup(button1_pin, gpio.IN, pull_up_down=gpio.PUD_UP)
gpio.setup(button2_pin, gpio.IN, pull_up_down=gpio.PUD_UP)

# Set up LEDs as output
gpio.setup(led1_pin, gpio.OUT)
gpio.setup(led2_pin, gpio.OUT)

try:
    while True:
        button1_state = gpio.input(button1_pin)  # Read Switch 1 state
        button2_state = gpio.input(button2_pin)  # Read Switch 2 state

        # Control LED 1 with Switch 1
        if button1_state == False:  # Button 1 is pressed (LOW)
            gpio.output(led1_pin, True)  # Turn LED 1 ON
            print("Switch 1 Pressed - LED 1 ON")
        else:  # Button 1 is released (HIGH)
            gpio.output(led1_pin, False)  # Turn LED 1 OFF
            print("Switch 1 Released - LED 1 OFF")

        # Control LED 2 with Switch 2
        if button2_state == False:  # Button 2 is pressed (LOW)
            gpio.output(led2_pin, True)  # Turn LED 2 ON
            print("Switch 2 Pressed - LED 2 ON")
        else:  # Button 2 is released (HIGH)
            gpio.output(led2_pin, False)  # Turn LED 2 OFF
            print("Switch 2 Released - LED 2 OFF")

        time.sleep(0.1)  # Small delay to debounce

except KeyboardInterrupt:  # Handle Ctrl+C exit
    print("\nExiting... Cleaning up GPIO")
    gpio.cleanup()  # Reset GPIO settings

# Turning on Buzzer using PIR Sensor
import RPi.GPIO as gpio  # Import Raspberry Pi GPIO library
import time  # Import time library

# Define GPIO pins
sensor = 11  # PIR sensor connected to GPIO 11
buzzer = 19  # Buzzer connected to GPIO 19

# GPIO Setup
gpio.setwarnings(False)  # Disable warnings
gpio.setmode(gpio.BOARD)  # Use BOARD pin numbering
gpio.setup(sensor, gpio.IN)  # Set PIR sensor as input
gpio.setup(buzzer, gpio.OUT)  # Set buzzer as output
gpio.output(buzzer, False)  # Initially turn buzzer OFF

# Initializing PIR Sensor
print("Initializing PIR sensor...")
time.sleep(1)  # Wait for the sensor to stabilize
print("PIR Ready")

try:
    while True:
        if gpio.input(sensor):  # If motion is detected
            gpio.output(buzzer, True)  # Turn ON buzzer
            print("Motion Detected!")
            time.sleep(1)  # Wait for 1 second
        else:
            gpio.output(buzzer, False)  # Turn OFF buzzer
            time.sleep(1)  # Wait for 1 second

except KeyboardInterrupt:  # Handle Ctrl+C interruption
    print("\nExiting... Cleaning up GPIO")
    gpio.cleanup()  # Reset GPIO settings
import RPi.GPIO as gpio  # Import Raspberry Pi GPIO library
import time  # Import time library

# Define GPIO pins
sensor = 11  # PIR sensor connected to GPIO 11
led = 19  # LED connected to GPIO 19

# GPIO Setup
gpio.setwarnings(False)  # Disable warnings
gpio.setmode(gpio.BOARD)  # Use BOARD pin numbering
gpio.setup(sensor, gpio.IN)  # Set PIR sensor as input
gpio.setup(led, gpio.OUT)  # Set LED as output
gpio.output(led, False)  # Initially turn LED OFF

# Initializing PIR Sensor
print("Initializing PIR sensor...")
time.sleep(12)  # Wait for 12 seconds for the sensor to stabilize
print("PIR Ready")

try:
    while True:
        if gpio.input(sensor):  # If motion is detected
            gpio.output(led, True)  # Turn ON LED
            time.sleep(1)  # Wait for 1 second
        else:
            gpio.output(led, False)  # Turn OFF LED
            time.sleep(1)  # Wait for 1 second

except KeyboardInterrupt:  # Handle Ctrl+C interruption
    print("\nExiting... Cleaning up GPIO")
    gpio.cleanup()  # Reset GPIO settings
import RPi.GPIO as GPIO
from time import sleep

# Set GPIO mode
GPIO.setmode(GPIO.BOARD)

# Define relay pins
relay_pins = [19, 23, 24, 25]

# Set up GPIO pins as output
for pin in relay_pins:
    GPIO.setup(pin, GPIO.OUT)

try:
    while True:
        # Turn all relays ON
        GPIO.output(19, GPIO.HIGH)
        GPIO.output(23, GPIO.HIGH)
        GPIO.output(24, GPIO.HIGH)
        GPIO.output(25, GPIO.HIGH)

        sleep(5)  # Wait for 5 seconds

        # Turn all relays OFF
        GPIO.output(19, GPIO.LOW)
        GPIO.output(23, GPIO.LOW)
        GPIO.output(24, GPIO.LOW)
        GPIO.output(25, GPIO.LOW)

        sleep(5)  # Wait for 5 seconds

except KeyboardInterrupt:
    print("Program terminated")
    GPIO.cleanup()  # Reset GPIO settings
import RPi.GPIO as GPIO
from time import sleep

ip = 35
op = 40

GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)
GPIO.setup(ip, GPIO.IN)
GPIO.setup(op, GPIO.OUT, initial=0)

print("Started...")

try:
    while True:
        if not GPIO.input(ip):
            GPIO.output(op, 1)
            print("Sensed")
            while not GPIO.input(ip):
                sleep(0.2)
        else:
            GPIO.output(op, 0)
except KeyboardInterrupt:
    GPIO.cleanup()


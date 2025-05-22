# DHTwiththingspeak.py
import Adafruit_DHT
import RPi.GPIO as GPIO
import time
from urllib.request import urlopen

GPIO.setmode(GPIO.BOARD)  # Use physical pin numbering
GPIO.setwarnings(False)

WRITE_API = "NHZWIGQIMSBY7DJH"  # Replace your ThingSpeak API key here
BASE_URL = "https://api.thingspeak.com/update?api_key={}".format(WRITE_API)

sensor = Adafruit_DHT.DHT11
pin = 17  # GPIO17 is connected to board P15 -- 11 pin

print("temp & humd in progress")

try:
    while(1):
        humidity, temperature = Adafruit_DHT.read_retry(sensor, pin)
        print("Humidity = " + str(humidity))
        print("Temperature = " + str(temperature))
        thingspeakHttp = BASE_URL + "&field1={:.2f}&field2={:.2f}".format(humidity, temperature)
        print(thingspeakHttp)
        conn = urlopen(thingspeakHttp)
        time.sleep(3)

except KeyboardInterrupt:
    GPIO.cleanup()

# ServoMotorSweepControl.py
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

# rain.py

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

# servo_multiple.py
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

# servo_single.py

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

# soil.py
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
# matlab
channelID = 2934028; 
readAPIKey = '57HB9YD28FI7C4VF';
writeAPIKey = 'YOUR_WRITE_API_KEY'; 
data = thingSpeakRead(channelID, "NumPoints", 100, "ReadKey", readAPIKey); 
thingSpeakWrite(channelID, "Fields", [1, 2, 3, 4], "Values", [max(data), min(data), mean(data), std(data)], "WriteKey", writeAPIKey);

maxval = max(data);
minval = min(data);
meanval = mean(data);
stdval = std(data);

disp("Maximum Value:");
disp(maxval);
disp("Minimum Value:");
disp(minval);
disp("Mean Value:");
disp(meanval);
disp("Standard Deviation:");
disp(stdval);



# custom input
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

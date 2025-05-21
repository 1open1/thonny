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

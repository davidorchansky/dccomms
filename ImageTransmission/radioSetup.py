import Adafruit_BBIO.GPIO as GPIO
from time import sleep 
rxs = "P8_8"
txs = "P8_7"
GPIO.setup(txs, GPIO.OUT) #TX
GPIO.setup(rxs, GPIO.OUT) #RX
GPIO.output(txs, GPIO.LOW)
GPIO.output(rxs, GPIO.HIGH)
#GPIO.cleanup()
while True:
	sleep(1)

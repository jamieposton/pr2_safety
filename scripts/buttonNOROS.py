#!/usr/bin/env python

import RPi.GPIO as GPIO
from time import sleep

import socket

GPIO.setmode(GPIO.BCM)
GPIO.setup(24, GPIO.IN, pull_up_down = GPIO.PUD_UP)

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

def talker():
	sock.connect(("134.197.40.237", 2048))
	while True:
		buttonbool = False
		sock.send(buttonbool)
		if(GPIO.input(24) == 0):
			print("Button Pressed")
			buttonbool = True
			sock.send(str(buttonbool)[0])
			return




if __name__ == '__main__':
	talker()

sock.close()
GPIO.cleanup()

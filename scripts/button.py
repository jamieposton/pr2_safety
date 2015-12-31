#!/usr/bin/env python

import RPi.GPIO as GPIO
import rospy
from std_msgs.msg import Bool
from time import sleep

GPIO.setmode(GPIO.BCM)
GPIO.setup(24, GPIO.IN, pull_up_down = GPIO.PUD_UP)

def talker():
	pub = rospy.Publisher('buttonBool', Bool)
	rospy.init_node('ButtonPi', anonymous=True)
	rate = rospy.Rate(10) #10Hz
	while True:
		buttonbool = False
		if(GPIO.input(24) == 0):
			print("Button Pressed")
			buttonbool = True
			rospy.loginfo(buttonbool)
			pub.publish(buttonbool)
			rate.sleep()
			return
		rospy.loginfo(buttonbool)
		pub.publish(buttonbool)
		rate.sleep()

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
GPIO.cleanup()

#!/usr/bin/env python
# -*- coding: utf-8 -*-

import roslib; roslib.load_manifest('p3dx')
import rospy

from std_msgs.msg import *
from sensor_msgs.msg import *

class DeadMansButton():
	def __init__(self):
		rospy.init_node("dead_mans_button")

		self.state = False
		self.dmb_time = rospy.Time.now()
		self.dmb_state = False
		self.dmb_idx = rospy.get_param("~/dead_mans_button_index", -5)
		self.dmb_pub = rospy.Publisher("~/dead_mans_button_okay", Bool)
		rospy.Subscriber("/joy", Joy, self.joy_state)

	def joy_state(self, msg):
		self.dmb_state = True if msg.buttons[self.dmb_idx] == 1 else False
		self.dmb_time = rospy.Time.now()

	def run(self, rate = 50.0):
		r = rospy.Rate(rate)
		d = rospy.get_param("~/dead_mans_button_delay", 5.0)
		while not rospy.is_shutdown():
			dt = rospy.Time.now() - self.dmb_time

			dmb_state = self.dmb_state if dt.to_sec() < d else False

			# im Falle des Abreissens der Verbindung nach d Sekunden stoppen			
			self.dmb_pub.publish(dmb_state) 
			
			r.sleep()


if __name__ == "__main__":
	dmb = DeadMansButton()
	dmb.run()

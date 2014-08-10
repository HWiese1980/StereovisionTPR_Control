#!/usr/bin/env python
import roslib; roslib.load_manifest('p3dx')
import rospy

from geometry_msgs.msg import *
from sensor_msgs.msg import *
from std_msgs.msg import *
from std_srvs.srv import Empty 

class ps3Control():
	def __init__(self):
		self.deadmankey = False
		self.motors_enabled = False
		self.motor_state_switching = False

	def dead_mans_button_state(self, msg):
		if self.deadmankey != msg.data:
			rospy.loginfo("Dead Mans Button: %s", "okay" if msg.data else "He's dead, Jim!")
		
		self.deadmankey = msg.data
		if not self.motor_state_switching:
			if self.deadmankey and not self.motors_enabled:
				self.motor_state_switching = True
				self.motors_on()
			elif not self.deadmankey and self.motors_enabled:
				self.motor_state_switching = True
				self.motors_off()

	def motors_state_changed(self, msg):
		rospy.loginfo("State of motors changed: %s" % ("on" if msg.data else "off"))
		self.motors_enabled = msg.data
		self.motor_state_switching = False

	def ps3input(self, msg):
		trans = msg.axes[1]
		ang = msg.axes[0]

		tw = Twist()
		tw.linear.x = trans * self.speed
		tw.angular.z = ang * self.tr

		if self.motors_enabled: 
			self.cmdvel_pub.publish(tw)
		else:
			self.cmdvel_pub.publish(Twist())

	def p3dxteleop(self):
		rospy.init_node("p3dxteleop")

		self.cmdvel_pub = rospy.Publisher("/rosaria/cmd_vel", Twist)
		self.speed = rospy.get_param("~/max_velocity", 0.5)
		self.tr = rospy.get_param("~/max_turnrate", 1.0)
		self.motors_on = rospy.ServiceProxy('/rosaria/enable_motors', Empty)
		self.motors_off = rospy.ServiceProxy('/rosaria/disable_motors', Empty)

		rospy.Subscriber("~/dead_mans_button_okay", Bool, self.dead_mans_button_state)
		rospy.Subscriber("/rosaria/motors_state", Bool, self.motors_state_changed)

		rospy.Subscriber("/joy", Joy, self.ps3input)
		rospy.spin()


if __name__ == "__main__":
	p = ps3Control()
	p.p3dxteleop()
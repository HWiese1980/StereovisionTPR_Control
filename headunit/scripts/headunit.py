#!/usr/bin/env python
# -*- coding: utf-8 -*-

import roslib; roslib.load_manifest('headunit')
import rospy

import numpy

from math import radians, degrees

from std_msgs.msg import *
from geometry_msgs.msg import Quaternion, Twist, PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from dynamixel_msgs.msg import JointState
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry

from subprocess import call

class Headunit:
	def __init__(self):
		self.use_motors = True
		self.sleeping = False
		self.told_sleeping = False
		self.use_robot_orientation = False
		self.current = dict(pitch = 0.0, yaw = 0.0, roll = 0.0)
		self.robot_odometry_yaw = 0.0
		self.lerp_timer = None
		self.reset_offset_button_pressed = False
		self.yaw_offset = 0.0

	### Den Kopf langsam in die gewünschte Position fahren
	def lerp_head(self, p, y, r, dt, rate = 0.01):

		dp = (p - self.current["pitch"])
		dy = (y - self.current["yaw"])		
		dr = (r - self.current["roll"])

		sp = dp / dtmsg.pose.pose.orientation
		sy = dy / dt
		sr = dr / dt

		rospy.loginfo("c: %s, d: %s, s: %s" % (self.current, [dp, dy, dr], [sp, sy, sr]))

		self.lerp_params = {"sp": sp, "sy": sy, "sr": sr, "dt": dt, "time": rospy.Time.now()}
		self.lerp_params.update(self.current)

		if self.lerp_timer == None: 
			self.lerp_timer = rospy.Timer(rospy.Duration(rate), self.lerp_timer_cb)

	def lerp_timer_cb(self, event):
		delta = event.current_real - self.lerp_params['time']
		ds = delta.to_sec()

		if delta.to_sec() > self.lerp_params['dt']: 
			self.lerp_timer.shutdown()
			self.lerp_timer = None

		s = dict()
		(s["pitch"], s["yaw"], s["roll"]) = [(self.lerp_params[i[1]] + self.lerp_params[i[0]] * ds) for i in (('sp','pitch'), ('sy','yaw'), ('sr','roll'))]

		self.pitch_pub.publish(s["pitch"])
		self.yaw_pub.publish(s["yaw"])
		self.roll_pub.publish(s["roll"])


	def turn_head(self, p, y, r):
		if self.sleeping and not self.told_sleeping:
			rospy.loginfo("Headunit: Hey, I'm sleeping, friend! Leave me alone!")
			self.told_sleeping = True
		elif not self.sleeping:

			y = numpy.clip(y, self.yaw_min, self.yaw_max)
			# Arbeitsraum pitch wird um den tatsächlichen Yaw Winkel eingeschränkt, um Gimbal Lock Effekte
			# zu verringern.
			pitch_min_reduced = min(0, self.pitch_min + abs(y))
			pitch_max_reduced = max(0, self.pitch_max - abs(y))

			p = numpy.clip(p, pitch_min_reduced, pitch_max_reduced) - self.yaw_offset
			r = numpy.clip(r, self.roll_min, self.roll_max)
			
			#rospy.loginfo("publish p: %f, y: %f, r: %f" % (p,y,r))
			self.pitch_pub.publish(p) 
			self.yaw_pub.publish(y)
			self.roll_pub.publish(r)

	def rift_orientation_callback(self, msg):
		#rospy.loginfo("rift callback")
		if not self.sleeping and self.use_motors:
			(p,y,r) = euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
			if self.use_robot_orientation:
				#rospy.loginfo("hmd yaw: %f, odo yaw: %f, diff: %f" % (y, self.robot_odometry_yaw, y - self.robot_odometry_yaw))
				self.turn_head(-p, y - self.robot_odometry_yaw,-r)
			else:
				self.turn_head(-p,y,-r) # Invertierte Roll- und Pitch-Achse aufgrund gedreht eingebauter Servos

	def odometry_callback(self, msg):
		#rospy.loginfo("odometry yaw")
		(po,ro,yo) = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
		self.robot_odometry_yaw = yo

	def head_state_callback(self, state, joint):
		self.current[joint] = state.current_pos
		#remove lines below for 30 Hz publisher
		# q = quaternion_from_euler(-self.current["pitch"], self.current["yaw"], -self.current["roll"])
		# msg = PoseStamped()
		# msg.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
		# msg.header.stamp = rospy.Time.now()
 		# self.orient_pub.publish(msg)

 	def publish_head_orientation(self):
 		q = quaternion_from_euler(-self.current["pitch"], self.current["yaw"], -self.current["roll"])
		msg = PoseStamped()
		msg.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
		msg.header.stamp = rospy.Time.now()
 		self.orient_pub.publish(msg)

	def dead_mans_button(self, msg):
		deadman_active = self.deadman
		self.sleepmode(not msg.data and deadman_active) # Totmannknopf invertiert für Schlafmodus
		

	def euler_callback(self, twist):
		if not self.sleeping:
			(p,y,r) = (twist.angular.x, twist.angdead_mans_buttonular.y, twist.angular.z)
			self.turn_head(p,y,r)

	def sleepmode(self, on_off, arg = ""):
		if on_off != self.sleeping:
			rospy.loginfo("Headunit: switching sleepmode: %s" % "on" if on_off else "off")
			if on_off:
				self.sleeping = True
				self.told_sleeping = False
				rospy.loginfo("Headunit: Alright, falling asleep... chhhrrrr...")
				
				if arg != "quietly":
					self.lerp_head(1.57, 0.0, 0.0, 1.0)
				else:
					rospy.loginfo("Headunit: hush! Don't move!")

			else:
				rospy.loginfo("Headunit: Waking up... uaahhh!")
				
				if arg != "quietly":
					self.lerp_head(0.0, 0.0, 0.0, 1.5)
				else:
					rospy.loginfo("Headunit: hush! Don't move!")

				self.sleeping = False
				self.told_sleeping = False

	# Totmanneinrichtung ersetzt Schlafmodus über Kommandozeile
	def command_callback(self, cmd):

		commandline = cmd.data.split()
		command = commandline[0]
		args = commandline[1:]
		rospy.loginfo("Headunit: Receiving command %s (%s)... " % (command, args))
		#if command == 'sleep':
		#	self.sleepmode(True, args[0])
		#if command == 'wakeup':
		#	self.sleepmode(False, args[0])
		if command == 'deadman':
			self.deadman = args[0] == '1'
			rospy.set_param("deadman_active", self.deadman)
		else:
			rospy.logwarn("Headunit: Huh? What does '%s' mean?" % cmd.data)

	def joy_callback(self, msg):
		if msg.buttons[-6] == 1 and not self.reset_offset_button_pressed: 
			self.reset_offset_button_pressed = True
		elif msg.buttons[-6] == 0 and self.reset_offset_button_pressed:
			self.yaw_offset = self.current["yaw"]
			self.reset_offset_button_pressed = False

	def run(self):
		rospy.init_node('headunit')

		self.use_motors = rospy.get_param('~use_motors', True)
		self.use_robot_orientation = rospy.get_param('~use_robot_orientation', False)

		self.deadman = rospy.get_param("deadman_active", False)

		self.pitch_pub = rospy.Publisher('/pitch_controller/command', Float64)
		self.yaw_pub = rospy.Publisher('/yaw_controller/command', Float64)
		self.roll_pub = rospy.Publisher('/roll_controller/command', Float64)
		self.euler_pub = rospy.Publisher('/headunit/euler', Twist)
		#self.orient_pub = rospy.Publisher('/headunit/orientation', Quaternion)
		self.orient_pub = rospy.Publisher('/headunit/orientation_stamped', PoseStamped)

		self.pitch_min = radians(rospy.get_param('~pitch_min', -75))
		self.pitch_max = radians(rospy.get_param('~pitch_max', +90))
		self.yaw_min = radians(rospy.get_param('~yaw_min', -80))
		self.yaw_max = radians(rospy.get_param('~yaw_max', +80))
		self.roll_min = radians(rospy.get_param('~roll_min', -45))
		self.roll_max = radians(rospy.get_param('~roll_max', +45))

		if self.deadman: 
			self.sleepmode(True)
		else:
			rospy.loginfo("Headunit: Dead man button deactivated. Never gonna sleep again!")

		rospy.loginfo("-- Headunit Configuration --")
		rospy.loginfo("-- Limits:")
		rospy.loginfo("     Pitch: %.2f - %.2f" % (self.pitch_min, self.pitch_max))
		rospy.loginfo("     Yaw  : %.2f - %.2f" % (self.yaw_min, self.yaw_max))
		rospy.loginfo("     Roll : %.2f - %.2f" % (self.roll_min, self.roll_max))

		#rospy.Subscriber('/oculus/orientation', Quaternion, self.rift_orientation_callback)
		rospy.Subscriber('/oculus/orientation_stamped', PoseStamped, self.rift_orientation_callback)
		rospy.Subscriber('/headunit/euler', Twist, self.euler_callback)
		rospy.Subscriber('/me/commands', String, self.command_callback)
		rospy.Subscriber('/dead_mans_button_okay', Bool, self.dead_mans_button)
		rospy.Subscriber('/joy', Joy, self.joy_callback)
		
		rospy.Subscriber('/pitch_controller/state', JointState, self.head_state_callback, "pitch")
		rospy.Subscriber('/yaw_controller/state', JointState, self.head_state_callback, "yaw")
		rospy.Subscriber('/roll_controller/state', JointState, self.head_state_callback, "roll")

		rospy.Subscriber('/rosaria/pose', Odometry, self.odometry_callback)

		#rospy.spin()

		r = rospy.Rate(30) # 30hz
		while not rospy.is_shutdown():
			self.publish_head_orientation()
			r.sleep()

if __name__ == '__main__':
	hu = Headunit()
	hu.run()


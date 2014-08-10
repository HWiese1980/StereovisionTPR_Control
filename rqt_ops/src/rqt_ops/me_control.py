#!/usr/bin/env python
import os
import rospy

import numbers, decimal

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import QTimer, Qt

from dynamixel_msgs.msg import JointState
from rosgraph_msgs.msg import Log

from std_msgs.msg import String, Float32, Float64, Bool

from threading import Lock

class MeControl(Plugin):
	def __init__(self, context):
		super(MeControl, self).__init__(context)
		self.setObjectName('MeControl')

		self.displayLock = Lock()

		from argparse import ArgumentParser
		parser = ArgumentParser()
		# ... Argumente ueberlegen...

		self.logModel = LogModel(self)
		self.logModel.rowsInserted.connect(self.autoScrollLog)

		self._widget = QWidget()
		ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'MeControl.ui')
		loadUi(ui_file, self._widget)
		self._widget.setObjectName('MeControlUi')
		self._widget.setWindowTitle("ME Control")

		self.dsp_angle = dict(pitch = self._widget.pitchAngle.setValue, yaw = self._widget.yawAngle.setValue, roll = self._widget.rollAngle.setValue)
		self.dsp_temp = dict(pitch = self._widget.tempPitch.display, yaw = self._widget.tempYaw.display, roll = self._widget.tempRoll.display)
		self.dsp_load = dict(pitch = self._widget.loadPitch.display, yaw = self._widget.loadYaw.display, roll = self._widget.loadRoll.display)
		self.dsp_speed = dict(pitch = self._widget.speedPitch.display, yaw = self._widget.speedYaw.display, roll = self._widget.speedRoll.display)
		self.dsp_battery = dict(voltage = self._widget.batteryVoltage.setValue)
		# self.dsp_motors = dict(state = self._widget.motorsState.setCheckState)

		# if context.serial_number() > 1:
		# 	self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

		rospy.Subscriber('/pitch_controller/state', JointState, self.stateCallback, "head/pitch")
		rospy.Subscriber('/yaw_controller/state', JointState, self.stateCallback, "head/yaw")
		rospy.Subscriber('/roll_controller/state', JointState, self.stateCallback, "head/roll")
		rospy.Subscriber('/rosaria/battery_voltage', Float64, self.stateCallback, "engine/battery/voltage")
		# rospy.Subscriber('/rosaria/motors_state', Bool, self.stateCallback, "engine/motors/state")
		
		rospy.Subscriber('/rosout_agg', Log, self.logCallback)

		self.cmdpub = rospy.Publisher("/me/commands", String)

		self._widget.logList.setModel(self.logModel)

		context.add_widget(self._widget)

	def logCallback(self, log):
		items = [ QStandardItem(field) for field in [log.file, log.function, "%i"%log.line, log.msg] ]
		for item in items: item.setFlags(Qt.NoItemFlags)
		self.logModel.appendRow(items)

	def autoScrollLog(self):
		QTimer.singleShot(0, self._widget.logList.scrollToBottom)

	def stateCallback(self, state, v):
		with self.displayLock:
			source_path = v.split('/')
			if len(source_path) >= 2: 
				pt = source_path[0]
				sv = source_path[1]
				sb = source_path[2] if len(source_path) > 2 else ""
				if pt == "head":
					a = state.current_pos if isinstance(state.current_pos, numbers.Number) else 0.0
					t = state.motor_temps[0] if len(state.motor_temps) > 0 else 0.0
					l = state.load if isinstance(state.load, numbers.Number) else 0.0
					v = state.velocity if isinstance(state.velocity, numbers.Number) else 0.0

					self.dsp_angle[sv](a)
					self.dsp_temp[sv](t)
					self.dsp_load[sv](l)
					self.dsp_speed[sv](v)
				elif pt == "engine":
					if sv == "battery" and isinstance(state.data, numbers.Number):
						self.dsp_battery[sb](state.data)
					# elif sv == "motors":
					# 	self.dsp_motors[sb](Qt.Checked if state.data else Qt.Unchecked)
			else:
				rospy.logerror("Invalid source path: %s" % v)
					

	def shutdown_plugin(self):
		pass

	def save_settings(self, plugin_settings, instance_settings):
		pass

	def restore_settings(self, plugin_settings, instance_settings):
		pass

class LogModel(QStandardItemModel):
	header_labels = ['File', 'Function', 'Line', 'Message']
	def __init__(self, parent=None):
		QStandardItemModel.__init__(self, parent)

	def headerData(self, section, orientation, role=Qt.DisplayRole):
		if role == Qt.DisplayRole and orientation == Qt.Horizontal:
			return self.header_labels[section]

		return QStandardItemModel.headerData(self, section, orientation, role)


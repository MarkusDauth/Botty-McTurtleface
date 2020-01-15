#!/usr/bin/python
# -*- coding: utf-8 -*-

'''
Author: David Kostka
Date: 17.12.19

Description:
Control-Module for Botty.

This class acts as a 'command center'.
Its task is to plan, control, and distribute tasks to and from the different modules, wich are as follows:
- arm
- camera
- lidar/navigation
- speech
- (controller)
'''

import rospy
from Botty import *
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from controller.msg import Command, Entity
import re
import actionlib
from kobuki_msgs.msg import AutoDockingAction, AutoDockingGoal
from actionlib_msgs.msg import GoalStatus
from threading import Thread

from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

from motor.srv import call,callResponse

from camera.srv import *

class Camera(object):
	def __init__(self):
		try: 
			rospy.wait_for_service('/action_controller/find_object')
			rospy.loginfo("Camera initialized")
		except:
			rospy.loginfo("Camera failed to initialize")
	
	def find(self):
		found = None

		try:
			self.findObj = rospy.ServiceProxy('/action_controller/find_object', FindObjects)
			found = self.findObj().object
		except rospy.ServiceException, e:
			rospy.loginfo("Camera failed")
		
		return found

class Nav(object):
	def __init__(self):
		self.enable_dock = False

		try:
			rospy.wait_for_service('motor')
			self.job = rospy.ServiceProxy('motor', call)
			self.stopper = rospy.ServiceProxy('motor', call)

			if self.enable_dock:
				self.init_dock()

			rospy.loginfo("Navigator initialized")
		except:
			rospy.loginfo("Navigator failed to initialize")
			return
	
	def init_dock(self):
		self.dock_client = actionlib.SimpleActionClient('dock_drive_action', AutoDockingAction)
		self.dock_client.wait_for_server(rospy.Duration.from_sec(5))

	def dock(self):	
		goal = AutoDockingGoal();
		self.dock_client.send_goal(goal)
		self.dock_client.wait_for_result()
		print(self.dock_client.get_result())

	def cancel(self):
		if self.enable_dock:
			self.dock_client.cancel_all_goals()
		
		param=[]
		command=call()
		command.call="stopp"
		command.param=param
		self.stopper(command.call, command.param)

	def drive(self, meters):
		param=[]
		param.append(meters)
		#param.append(1)
		command=call()
		command.call="forwardByMeters"
		command.param=param
		return self.job(command.call, command.param).success

	def turn(self, angle):
		param=[]
		param.append(angle)
		command=call()
		if angle > 0:
			command.call="turnRigthByAngle"
		else:
			command.call="turnLeftByAngle"
		command.param=param
		return self.job(command.call, command.param).success
	
	def go_to(self, x, y):
		param=[]
		param.append(x)
		param.append(y)
		command=call()
		command.call="moveToPosition"
		command.param=param
		return self.job(command.call, command.param).success

class Controller(object):
	
	def __init__(self):
		rospy.loginfo("Initializing Controller")

		self.known_obj = {"ball", "cup", "object", "arrow"}

		# Initializing Threads
		self.mutator_thread = Thread()
		self.observer_thread = Thread()

		# initialize node
		rospy.init_node("control")
		rospy.on_shutdown(self.shutdown)
		#rate = rospy.Rate(10)

		# Sound handler
		self.soundhandle = SoundClient()
		self.voice = 'voice_kal_diphone'
		#self.voice = 'voice_nitech_us_rms_arctic_hts'
		self.volume = 1.0

		# Modules
		self.cam = Camera()
		self.nav = Nav()

		# Subscribe to speech commands
		self.sub = rospy.Subscriber("/botty/speech/commands", Command, self.process_command)

		rospy.loginfo("Controller initialized")
		self.say("I am Botty McTurtleface")

	def process_command(self, cmd):
		print("Incoming command ...")
		if cmd.action == Action.STOP:
			self.stop_all()
			return

		if not self.mutator_thread.isAlive():
			if cmd.action == Action.GO:
				self.go(cmd.obj)
				return
		if not self.observer_thread.isAlive():
			if cmd.action == Action.SEARCH:
				self.observer_thread = Thread(target=self.search, args=(cmd.obj,))
				self.observer_thread.start()
				return
		
		self.say("I'm already doing something")

	def stop_all(self):
		self.say('Canceling actions')
		self.nav.cancel()
		print('Waiting for threads')
		if self.mutator_thread.isAlive():
			self.mutator_thread.join()
		if self.observer_thread.isAlive():
			self.observer_thread.join()
		self.say('Stopped all Actions')

	def search(self, obj):
		if obj.name == "all":
			self.say("I know: " + ', '.join(self.known_obj))
			
		else:
			found = self.cam.find()
			if found: 
				if obj.name in found:
					if obj.attr[0] in found:
						self.say("Found " + found)
					else:
						self.say("Found similar object: " + found)
				else:
					self.say("Not found, but found: " + found)
			else: 
				self.say("Couldn't find object")
				return
	
	def go(self, obj):	
		if obj.name == "forward": 
			self.mutator_thread = Thread(target=self.nav.drive, args=(0.4,))
		elif obj.name == "right": 
			self.mutator_thread = Thread(target=self.nav.turn, args=(90,))
		elif obj.name == "left": 
			self.mutator_thread = Thread(target=self.nav.turn, args=(-90,))
		elif obj.name == "docking station" and self.nav.enable_dock:
			self.mutator_thread = Thread(target=self.nav.dock)
		elif obj.name == "position": 
			self.mutator_thread = Thread(target=self.nav.go_to, args=(obj.attr[0], obj.attr[1], ))
		else:
			self.say("Unknown place or direction")
			return
		
		self.say('Moving ' + obj.name + " ...")
		self.mutator_thread.start()

	def say(self, txt):
		print('Saying: %s' % txt)
		self.soundhandle.stopAll()
		self.soundhandle.say(txt, self.voice, self.volume)

	def shutdown(self):
		rospy.loginfo("Stopping Controller")
		self.stop_all()
		rospy.sleep(1)
		

if __name__ == "__main__":
	instance = Controller()
	rospy.spin()


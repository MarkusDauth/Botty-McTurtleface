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
		self.obj = ""
		try: 
			rospy.wait_for_service('/action_controller/find_object')
			rospy.loginfo("Camera initialized")
		except:
			rospy.loginfo("Camera failed to initialize")
	
	def found(self):
		return self.obj

	def find(self):
		try:
			self.findObj = rospy.ServiceProxy('/action_controller/find_object', FindObjects)
			self.obj = self.findObj().object
			return True
		except rospy.ServiceException, e:
			self.obj = ""
			return False

class Nav(object):
	def __init__(self):
		self.dock_client = actionlib.SimpleActionClient('dock_drive_action', AutoDockingAction)
		try: 
			rospy.wait_for_service('motor')
		except:
			rospy.loginfo("Motor failed to initialize")

		self.job = rospy.ServiceProxy('motor', call)

		try:
			self.dock_client.wait_for_server(rospy.Duration.from_sec(2))
			rospy.loginfo("Navigator initialized")
		except:
			rospy.loginfo("Navigator failed to initialize")
	
	def dock(self):	
		goal = AutoDockingGoal();
		self.dock_client.send_goal(goal)
		self.dock_client.wait_for_result()
		print(self.dock_client.get_result())

	def cancel(self):
		self.dock_client.cancel_all_goals()
		command=call()
		command.call="stop"
		command.param=None
		self.job(command.call, command.param)



	def drive(self, meter):
		param=[]
		param.append(meter)
		command=call()
		command.call="forwardByMeters"
		command.param=param
		self.job(command.call,command.param)

	def turn(self, angle):
		param=[]
		param.append(angle)
		command=call()
		command.call="turnRigthByAngle"
		command.param=param
		self.job(command.call,command.param)

class Controller(object):
	
	def __init__(self):
		self.known_obj = {"ball", "cup", "object", "arrow"}

		self.thread = Thread()
		# initialize node
		rospy.init_node("control")
		rospy.on_shutdown(self.shutdown)
		#rate = rospy.Rate(10)

		# Initializing publishers

		# Sound handler
		self.soundhandle = SoundClient(blocking=True)

		self.voice = 'voice_kal_diphone'
		self.volume = 1.0

		# Modules
		self.cam = Camera()
		self.nav = Nav()

		# Subscribe to speech commands
		self.sub = rospy.Subscriber("/botty/speech/commands", Command, self.process_command)
		# Subscribe to speech commands
		#self.sub = rospy.Subscriber("/botty/controller/stop", Command, self.stop_all)

		rospy.loginfo("Controller initialized")

	def process_command(self, cmd):
		print("Incoming command ...")
		if not self.thread.isAlive():
			if cmd.action == Action.SEARCH:
				self.thread = Thread(target=self.search, args=(cmd.obj,))
				self.thread.start()
			if cmd.action == Action.GO:
				self.go(None, cmd.obj)
		elif cmd.action == Action.STOP:
			self.stop_all()
		else:
			print("Already doing something")

	def stop_all(self):
		self.say('Canceling actions')
		self.nav.cancel()
		print('Waiting for threads')
		#if self.thread.isAlive():
			#self.thread.join()

	def dock(self):	
		self.nav.dock()

	def search(self, obj):
		if obj.name == "all":
			self.say("I know: " + ', '.join(self.known_obj))
			
		else:
			if self.cam.find(): 
				if obj.name in self.cam.found():
					if obj.attr[0] in self.cam.found():
						self.say("Found " + self.cam.found())
					else:
						self.say("Found similar object: " + self.cam.found())
				else:
					self.say("Not found, but found: " + self.cam.found())
			else: 
				self.say("Couldn't find object")
				return
	
	def go(self, pos, obj):
		print('Going ' + obj.name + " ...")	

		if obj.name == "forward": 
			self.thread = Thread(target=self.nav.drive, args=(1,))
		elif obj.name == "right": 
			self.thread = Thread(target=self.nav.turn, args=(10,))
		#elif obj.name == "left": 
			#self.thread = Thread(target=self.nav.turn, args=(10,))
		elif obj.name == "docking station":
			self.thread = Thread(target=self.dock, args=(cmd.obj,))
		else:
			self.say("Unknown place or direction")
			return
		self.thread.start()

	def say(self, txt):
		print('Saying: %s' % txt)
		self.soundhandle.say(txt, self.voice, self.volume)

	def shutdown(self):
		rospy.loginfo("Stopping Controller")
		self.stop_all()
		rospy.sleep(1)
		

if __name__ == "__main__":
	instance = Controller()
	rospy.spin()


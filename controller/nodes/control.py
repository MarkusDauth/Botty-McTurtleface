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

On an incoming command from 'speech' it starts a new thread with the specified action and its arguments.
This thread can be further divided into sub-routines to acomplish the task.
For example, on "Grab a red ball" botty might first search, then move to a red ball before actually grabbing it. 

While the action is being handled by a thread, the main controller continues to listen for other commands.
If another command comes in, it checks if this new task can be done concurrently.
This might be a command like "what are you doing?".
If it can't run concurrently, it gets thrown out and the user will be informed.
Another option would be to put the new command in a queue.

The new command could also be some type of interrupt, to stop or hold all currently running tasks.
Interrupts will get passed onto all running tasks. 

When a task is done, Botty sends feedback to the User about the result.
It might also give feedback on progress.

Idden/Entwürfe:
Schnittstellen mit actionlib server/client?
Camera.get_objects()
Camera.get_object(obj)
Arm.grab(coord)
Nav.move_to(pos)

class Object:
    String name
    String attr[]
    Position pos
    Coord cam_pos
class Position:
    String name
    Coord coordinates

Aktionen:
go(pos)
    - Nav.move_to(pos)
grab(obj)
    - Nav.move_to(obj.pos)
    - cam_obj = Camera.get_objects()
    - Arm.grab(obj.coord)
bring(obj)
    - current_pos = Nav.get_position()
    - go(pos)
    - grab(obj)
    - go(current_pos)
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

class Controller(object):
	def __init__(self):
		# initialize node
		rospy.init_node("control")
		rospy.on_shutdown(self.shutdown)
		#rate = rospy.Rate(10)

		# Initializing publishers
		#self.pub = rospy.Publisher('/botty/commands', String)
		#self.pub = rospy.Publisher("mobile_base/commands/velocity", Twist)

		# Subscribe to speech commands
		self.sub = rospy.Subscriber("/botty/speech/commands", Command, self.process_command)
		# Subscribe to speech commands
		self.sub = rospy.Subscriber("/botty/controller/stop", Command, self.stop_cmd)

		#arm_client = actionlib.SimpleActionClient('arm', MoveArmAction)
		#nav_client = actionlib.SimpleActionClient('navigation', move_base_msgs/MoveBaseAction)
		self.dock_client = actionlib.SimpleActionClient('dock_drive_action', AutoDockingAction)
		self.dock_client.wait_for_server(rospy.Duration.from_sec(5))
		self.soundhandle = SoundClient()
		rospy.sleep(1)

		self.voice = 'voice_kal_diphone'
		self.volume = 1.0
		print("Initialized Controller")

	def process_command(self, cmd):
		print(Action.name(cmd.action), cmd.obj.name, cmd.obj.attr)
		
		# Eher ein service	
		if cmd.action == Action.STOP:
			self.stop_all()

		elif cmd.action == Action.GO:
			self.go(cmd.obj)
		elif cmd.action == Action.GRAB:
			self.grab(cmd.obj)

	def stop_cmd(self, msg):
		self.stop_all()

	def stop_all(self):
		print('Stopping')
		self.say('Stopping all actions')
		self.dock_client.cancel_all_goals()

	def go(self, obj):
		print('Going to ' + obj.name + " ...")
		#if(obj.name == 'kitchen')
		#elif(obj.name == 'bedroom')	
		if(obj.name == 'docking station'):	
			goal = AutoDockingGoal();
			self.dock_client.send_goal(goal, feedback_cb=self.dock_feedback)
			self.dock_client.wait_for_result()
			print(self.dock_client.get_result())
		
	def dock_feedback(self, fb):
		print 'Feedback: [DockDrive: ' + feedback.state + ']: ' + feedback.text		

	def grab(self, obj):
		print('Grabbing ' + obj.name + " ...")
		'''for p in [0, 10]
			if obj.name == camera.getObject():
				arm.grab(camera.getObject.pos)
			else:
				nav.turnLeft(10)
				# wait for turn
		#obj_list = camera.getObjects()
		'''
	
	def say(self, txt):
		print('Saying: %s' % txt)
		#print 'Voice: %s' % voice
		#print 'Volume: %s' % volume

		self.soundhandle.say(txt, self.voice, self.volume)

	def shutdown(self):
		rospy.loginfo("Stop Controller")
		self.stop_all()
		rospy.sleep(1)


if __name__ == "__main__":
	try:
		instance = Controller()
		rospy.spin()
	except rospy.ROSInterruptException: pass


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


class Node():
    """A node class for A* Pathfinding"""

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position


def astar(maze, start, end):
    """Returns a list of tuples as a path from the given start to the given end in the given maze"""

    # Create start and end node
    start_node = Node(None, start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, end)
    end_node.g = end_node.h = end_node.f = 0

    # Initialize both open and closed list
    open_list = []
    closed_list = []

    # Add the start node
    open_list.append(start_node)

    # Loop until you find the end
    while len(open_list) > 0:

        # Get the current node
        current_node = open_list[0]
        current_index = 0
        for index, item in enumerate(open_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index

        # Pop current off open list, add to closed list
        open_list.pop(current_index)
        closed_list.append(current_node)

        # Found the goal
        if current_node == end_node:
            path = []
            current = current_node
            while current is not None:
                path.append(current.position)
                current = current.parent
            return path[::-1] # Return reversed path

        # Generate children
        children = []
        #for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]: # Adjacent squares
	for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0)]: # Adjacent squares

            # Get node position
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

            # Make sure within range
            if node_position[0] > (len(maze) - 1) or node_position[0] < 0 or node_position[1] > (len(maze[len(maze)-1]) -1) or node_position[1] < 0:
                continue

            # Make sure walkable terrain
            if maze[node_position[0]][node_position[1]] != 0:
                continue

            # Create new node
            new_node = Node(current_node, node_position)

            # Append
            children.append(new_node)

        # Loop through children
        for child in children:

            # Child is on the closed list
            for closed_child in closed_list:
                if child == closed_child:
                    continue

            # Create the f, g, and h values
            child.g = current_node.g + 1
            child.h = ((child.position[0] - end_node.position[0]) ** 2) + ((child.position[1] - end_node.position[1]) ** 2)
            child.f = child.g + child.h

            # Child is already in the open list
            for open_node in open_list:
                if child == open_node and child.g > open_node.g:
                    continue

            # Add the child to the open list
            open_list.append(child)

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
		self.orientation = [0, 1]

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
		return self.job(command.call, command.param)

	def turn(self, angle):
		param=[]
		param.append(angle)
		command=call()
		if angle > 0:
			command.call="turnRigthByAngle"
		else:
			command.call="turnLeftByAngle"
		command.param=param
		return self.job(command.call, command.param)
	
	def go_to(self, x, y):
		maze = [[0, 0, 0, 0],
			[0, 0, 0, 0],
			[0, 0, 0, 0],
			[0, 0, 0, 0],
			[0, 0, 0, 0]]
		
		#coord = self.drive(0)
		#start = (coord.x, coord.y)
		end = (x, y)
		start = (0, 0)

		path = astar(maze, start, end)

		print(path)
		prev_coord = [0,0]
		target_orient = [0,0]
		target_orient[0] = path[0][0]
		target_orient[1] = path[0][1]

		for coord in path:
			#print("Coordinates: " + str(coord))
			#print("Previous: " + str(prev_coord))
			
			target_orient[0] = coord[0] - prev_coord[0]
			target_orient[1] = coord[1] - prev_coord[1]
			prev_coord = coord
			
			#print(self.orientation, target_orient)
			self.turnTo(target_orient)
			self.drive(0.4)
			
			print("\n")			
			
	def turnTo(self, target_orient):
		angle = 0
		
		print(self.orientation, target_orient)	
		add = self.orientation + target_orient

		if target_orient == self.orientation or target_orient == [0,0]:
			print("No turning")
			return			

		elif add == [0,0]:
			angle = 180
		else:
			if self.orientation == [1,0]:
				angle = target_orient[1] * 90
			elif self.orientation == [0,1]:
				angle = -target_orient[0] * 90
			elif self.orientation == [-1,0]:
				angle = -target_orient[1] * 90
			elif self.orientation == [0,-1]:
				angle = target_orient[0] * 90

		'''elif target_orient[0] == 0:
			if self.orientation[0] == 0:
				angle = 180
			else:
				angle = (target_orient[1]*self.orientation[0]) * 90

		elif target_orient[1] == 0:
			if self.orientation[1] == 0:
				angle = 180
			else:
				angle = (target_orient[0]*self.orientation[1]) * 90
		'''

		print("Turning: " + str(angle))
		self.orientation[0] = target_orient[0]
		self.orientation[1] = target_orient[1]
		self.turn(angle)

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
			self.mutator_thread = Thread(target=self.nav.go_to, args=(int(obj.attr[0]), int(obj.attr[1]), ))
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


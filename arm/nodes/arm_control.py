#!/usr/bin/python

'''
Author: David Kostka

Description:
Keyword based arm control
'''

import rospy
from std_msgs.msg import Float64, String
from motor.srv import call,callResponse

class Arm:
	def __init__(self):
		self.pub = { "elbow_pitch" : rospy.Publisher('/phantomx_reactor_controller/elbow_pitch_joint/command', Float64, queue_size=10),
			"shoulder_pitch" : rospy.Publisher('/phantomx_reactor_controller/shoulder_pitch_joint/command', Float64, queue_size=10),
			"shoulder_yaw" : rospy.Publisher('/phantomx_reactor_controller/shoulder_yaw_joint/command', Float64, queue_size=10),
			"wrist_pitch" : rospy.Publisher('/phantomx_reactor_controller/wrist_pitch_joint/command', Float64, queue_size=10),
			"wrist_roll" : rospy.Publisher('/phantomx_reactor_controller/wrist_roll_joint/command', Float64, queue_size=10)
		}
		
	def home(self):
		self.pub["elbow_pitch"].publish(1.5)
		self.pub["shoulder_pitch"].publish(-1.5)
		self.pub["shoulder_yaw"].publish(0.0)
		self.pub["wrist_pitch"].publish(0)
		self.pub["wrist_roll"].publish(0.0)
		rospy.sleep(5)
	

	def push(self):
		self.pub["elbow_pitch"].publish(-0.8)
		self.pub["shoulder_pitch"].publish(1.5)
		self.pub["shoulder_yaw"].publish(1)
		self.pub["wrist_pitch"].publish(0.5)
		self.pub["wrist_roll"].publish(0.0)
		
		rospy.sleep(4)
		self.pub["wrist_pitch"].publish(-0.8)
		self.pub["elbow_pitch"].publish(-1)
		rospy.sleep(2)
		self.pub["shoulder_yaw"].publish(-1)
		rospy.sleep(3)
	

	def callback(self, data):
		response=callResponse()
		if data.call == "home":
			print("Going to Home Position ...")
			self.home()
		elif data.call == "push":
			print("Pushing ...")
			self.push()
		return response

if __name__ == '__main__':
	rospy.init_node('arm')
	arm = Arm()
	rospy.Service('/botty/arm/commands', call, arm.callback)
	#rospy.Subscriber("/botty/arm/commands", String, arm.callback)
	rospy.spin()

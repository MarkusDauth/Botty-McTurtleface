#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
from std_msgs.msg import Float64

#Eine bereits vorhandene Pose soll eingenommen werden
#Pose wird mit dem Setup_Assistant erstellt

class arm_poser:
	def __init__(self):
		self.pub_pinch_rev = rospy.Publisher('/phantomx_reactor_controller/gripper_revolute_joint/command', Float64, queue_size=10)
		self.pub_pinch_pris = rospy.Publisher('/phantomx_reactor_controller/gripper_prismatic_joint/command', Float64, queue_size=10)
		self.robot = moveit_commander.RobotCommander()
		self.scene = moveit_commander.PlanningSceneInterface()

		#Gruppenname wird im Setup-Assistant festgelegt
		self.group_name = "arm"
		self.group = moveit_commander.MoveGroupCommander(self.group_name)

		self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20)



	def callback(self,data):
		if(data.data == "TOPRIGHT"):
			self.group.set_named_target("topRight")
			self.pose_target = geometry_msgs.msg.Pose()
			self.plan = self.group.plan()
			rospy.sleep(1)
			self.group.go()
			rospy.sleep(5)
		elif(data.data == "BOTTOMLEFT"):
			self.group.set_named_target("bottomLeft")
			self.pose_target = geometry_msgs.msg.Pose()
			self.plan = self.group.plan()
			rospy.sleep(1)
			self.group.go()
			rospy.sleep(5)
		elif(data.data == "hold"):
			self.pub_pinch_rev.publish(1)
		elif(data.data == "release"):
			self.pub_pinch_rev.publish(0)


		#Vorsichtshalber nochmal stoppen, falls doch noch eine Bewegung ausgefuehrt wird
		#group.stop()

		#Posen loeschen, ist guter Stil
		#group.clear_pose_targets()

if __name__ == '__main__':
	rospy.init_node("arm_poser")
	arm = arm_poser()
	rospy.Subscriber("botty/arm/command", String, arm.callback)
	rospy.spin()	


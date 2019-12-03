#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from std_msgs.msg import String

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

#Gruppenname wird im Setup-Assistant festgelegt
group_name = "arm"
group = moveit_commander.MoveGroupCommander(group_name)

display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20)

pose_target = geometry_msgs.msg.Pose()


# --- Eigener Code ab hier ---

#Subscriber Code
rospy.Subscriber("chatter", String, callback)

def listener():
	rospy.init_node('listener', anonymous=True)
	rospy.Subscriber("RASCHIED", String, callback)
	rospy.spin()


def callback(data):
#Eine bereits vorhandene Pose soll eingenommen werden
#Pose wird mit dem Setup_Assistant erstellt
	if data == "start":	
		group.set_named_target("start")
		plan1 = group.plan()
		rospy.sleep(1)
		group.go(wait=True)


#Vorsichtshalber nochmal stoppen, falls doch noch eine Bewegung ausgefuehrt wird
#group.stop()

#Posen loeschen, ist guter Stil
#group.clear_pose_targets()

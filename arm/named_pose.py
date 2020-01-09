#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

#Gruppenname wird im Setup-Assistant festgelegt
group_name = "arm"
group = moveit_commander.MoveGroupCommander(group_name)

display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20)

pose_target = geometry_msgs.msg.Pose()


# --- Eigener Code ab hier ---


#Eine bereits vorhandene Pose soll eingenommen werden
#Pose wird mit dem Setup_Assistant erstellt

group.set_named_target("home")

plan = group.plan()

rospy.sleep(1)

group.go(wait=True)


#Vorsichtshalber nochmal stoppen, falls doch noch eine Bewegung ausgefuehrt wird
#group.stop()

#Posen loeschen, ist guter Stil
#group.clear_pose_targets()

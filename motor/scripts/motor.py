#!/usr/bin/env python
# -*- coding: utf-8 -*-
#this document is made within a study project of the 'Hochschule Kaiserslautern'
#made by Felix Mayer
#version 0.5

import rospy
from geometry_msgs.msg import Twist
import actionlib
from actionlib_msgs.msg import *
from motor.msg import Command
from std_msgs.msg import String
PI = 3.1415926535897

global pub
global speed
speed=0.5
global stoppSignal
stoppSignal=False

def setSpeed(speedLocal):
    global speed
    if speedLocal!=None:
        speed=speedLocal

def calculateTurnTwist(angle):
    #initiate the Twist object
    global speed
    twist = Twist()
    angular_speed = speed*2*PI/360
    relative_angle = angle*2*PI/360
    twist.linear.x=0
    twist.linear.y=0
    twist.linear.z=0
    twist.angular.x = 0
    twist.angular.y = 0
    return twist,angular_speed,relative_angle

def moveTurnTwist(twist,angular_speed,relative_angle):
    global pub
    global speed
    startTime = rospy.Time.now().to_sec()
    current_angle = 0
    #turning until the wanted degree is reached
    while(current_angle < relative_angle):
        pub.publish(twist)
        currenTime = rospy.Time.now().to_sec()
        current_angle = angular_speed*(currenTime-startTime)
    #Forcing our robot to stop
    twist.angular.z = 0
    pub.publish(twist)

def turnLeftByAngle(angle,speedLocal=None):
    setSpeed(speedLocal)
    tunrTwist,angular_speed,relative_angle = calculateTurnTwist(angle)
    #turn counter clockwards
    tunrTwist.angular.z = abs(angular_speed)
    moveTurnTwist(tunrTwist,angular_speed,relative_angle)

def turnRigthByAngle(angle,speedLocal=None):
    setSpeed(speedLocal)
    tunrTwist,angular_speed,relative_angle = calculateTurnTwist(angle)
    #turn clockwise
    tunrTwist.angular.z = -abs(angular_speed)
    moveTurnTwist(tunrTwist,angular_speed,relative_angle)

def forward(speedLocal=None):
    #fahre ENDLOS vorwärts bis ein Stoppsignal eintrifft
    global pub
    global speed
    global stoppSignal
    stoppSignal=False
    setSpeed(speedLocal)
    forwardTwist=Twist()
    forwardTwist.linear.x = speed
    forwardTwist.angular.z = 0
    while (not interrupt()):
        pub.publish(forwardTwist)

def forwardByMeters(meter):
    global stoppSignal
    stoppSignal=False
    #tell the action client that we want to spin a thread by default
    movement = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    #wait for the action server to come up (for a duration of 5 seconds)
    movement.wait_for_server(rospy.Duration(5))
    #we'll send a goal to the robot to move the specific ammount of meters forward
    target = MoveBaseGoal()
    target.target_pose.header.frame_id = 'base_link'
    target.target_pose.header.stamp = rospy.Time.now()
    target.target_pose.pose.position.x = meter
    target.target_pose.pose.orientation.w = 1.0 #go forward

	#start his job
    movement.send_goal(target)
	#wait until job is done, with an timout of 60s
    success = movement.wait_for_result(rospy.Duration(60))
    if not success:
        movement.cancel_goal()
        rospy.loginfo("The base failed to move forward "+str(meter)+" meter")
    else:
        state = movement.get_state()
        if state == GoalStatus.SUCCEEDED:
            rospy.loginfo("Heurika! Botty made it.")

def interrupt():
    global stoppSignal
    if (stoppSignal):
        return True
    return False

def stopp():
    global stoppSignal
    stoppSignal=True

def shutdown():
        rospy.loginfo("Ros Shutdown")

def callback(command):
    #command msg shoud be like that:
    #string call (=>Functionname to be called)
    #string[] param (=>all parameters)
    done=False
    if command.call=="stopp":
        if len(command.param)==0:
            print("Call: stopp("+command.param[0]+")")
            stopp()
        else:
            print("Call: stopp; Error: Wrong amount of Parameters! stopp takes 0 parameters")
        done=True
    if command.call=="forwardByMeters":
        if len(command.param)==1:
            print("Call: forwardByMeters("+command.param[0]+")")
            forwardByMeters(int(command.param[0]))
        else:
            print("Call: forwardByMeters; Error: Wrong amount of parameters! forwardByMeters takes 1 parameter =>meters")
        done=True
    if command.call=="forward":
        if len(command.param)<2:
            print("Call: forward("+command.param[0]+")")
            if len(command.param)==0:
                forward()
            else:
                forward(int(command.param[0]))
        else:
            print("Call: forward; Error: Wrong amount of Parameters! stopp takes 1 optional parameter =>speed=None")
        done=True
    if command.call=="turnRigthByAngle":
        if len(command.param)==2:
            print("Call: turnRigthByAngle(angle="+command.param[0]+", speed="+command.param[1]+")")
            turnRigthByAngle(int(command.param[0]),int(command.param[1]))
        elif len(command.param)==1:
            print("Call: turnRigthByAngle(angle="+command.param[0]+")")
            turnRigthByAngle(int(command.param[0]))
        else:
            print("Call: turnRigthByAngle; Error: Wrong amount of Parameters! turnRigthByAngle takes 2 parameters; 1 is optional =>angle,speed=None")
        done=True
    if command.call=="turnLeftByAngle":
        if len(command.param)==2:
            print("Call: turnLeftByAngle(angle="+command.param[0]+", speed="+command.param[1]+")")
            turnLeftByAngle(int(command.param[0]),int(command.param[1]))
        elif len(command.param)==1:
            print("Call: turnLeftByAngle(angle="+command.param[0]+")")
            turnLeftByAngle(int(command.param[0]))
        else:
            print("Call: turnLeftByAngle; Error: Wrong amount of Parameters! turnLeftByAngle takes 2 parameters; 1 is optional =>angle,speed=None")
        done=True
    if command.call=="setSpeed":
        if len(command.param)==1:
            print("Call: setSpeed("+command.param[0]+")")
            setSpeed(int(command.param[0]))
        else:
            print("Call: setSpeed; Error: Wrong amount of Parameters! setSpeed takes 1 parameter =>speed")
        done=True
    if not done:
        print("Error: Call-name is unknown")


def startUp():
    global pub
    #Starts a new node
    rospy.init_node('motor', anonymous=False)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber("botty/controller",String,callback)
    rospy.shutdown(shutdown)
    rospy.spin()

#!/usr/bin/python

'''
Author: David Kostka

Description:
Keyword based movement control

Uses modified Code from Package:
https://github.com/gorinars/ros_voice_control
'''

import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time

class ASRControl(object):
    """Class to handle turtlebot control using voice"""

    def __init__(self):

        # Default values for turtlebot_simulator
        self.speed = 0.2
        # Intializing message type
        self.msg = Twist()
	

        # initialize node
        rospy.init_node("base_control")
        rospy.on_shutdown(self.shutdown)
	self.r = rospy.Rate(10)
        # Initializing publisher with buffer size of 10 messages
        self.pub_ = rospy.Publisher("mobile_base/commands/velocity", Twist, queue_size=10)

        # Subscribe to kws output
        rospy.Subscriber("/botty/speech/commands", String, self.parse_asr_result)
        rospy.spin()

    def parse_asr_result(self, detected_words): #pylint: disable=too-many-branches
        """Function to perform action on detected word"""
        if detected_words.data.find("full speed") > -1:
            if self.speed == 0.2:
                self.msg.linear.x = self.msg.linear.x * 2
                self.msg.angular.z = self.msg.angular.z * 2
                self.speed = 0.4
        elif detected_words.data.find("half speed") > -1:
            if self.speed == 0.4:
                self.msg.linear.x = self.msg.linear.x / 2
                self.msg.angular.z = self.msg.angular.z / 2
                self.speed = 0.2
        elif detected_words.data.find("forward") > -1:
            self.msg.linear.x = self.speed
            self.msg.angular.z = 0
        elif detected_words.data.find("left") > -1:
            self.msg.linear.x = self.speed
            self.msg.angular.z = 1
        elif detected_words.data.find("right") > -1:
            self.msg.linear.x = self.speed
            self.msg.angular.z = -1
        elif detected_words.data.find("back") > -1:
            self.msg.linear.x = -self.speed
            self.msg.angular.z = 0
        elif detected_words.data.find("stop") > -1 or detected_words.data.find("halt") > -1:
            self.msg = Twist()

        # Publish required message
	for x in range(0,10):	
		self.pub_.publish(self.msg)
		self.r.sleep()

    def shutdown(self):
        """
        command executed after Ctrl+C is pressed
        """
        rospy.loginfo("Stop base_control")
        self.pub_.publish(Twist())
        rospy.sleep(1)


if __name__ == "__main__":
    ASRControl()

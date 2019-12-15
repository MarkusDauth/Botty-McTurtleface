#!/usr/bin/python

'''
Author: David Kostka

Description:
Keyword based arm control
'''

import rospy
from std_msgs.msg import String

class Controller:
	def __init__(self):
	    self.pub = rospy.Publisher('botty/arm/command', String, queue_size=10)
	    self.position = ""

	def kws_callback(self, detected_words):
	    detected_words.data = detected_words.data.lower()
	    rospy.loginfo(detected_words.data)
	    if detected_words.data == 'hold ':
		self.pub.publish('hold')
		rospy.loginfo('-> hold')
	    elif detected_words.data == 'release ':
		self.pub.publish('release')
		rospy.loginfo('-> release')
	    elif detected_words.data == 'grab ':
		self.pub.publish(self.position)
		rospy.loginfo('-> grab')

	def camera_callback(self, data):
	    self.position = data

if __name__ == '__main__':
    rospy.init_node('control')
    controller = Controller()

    rospy.Subscriber("/botty/speech/commands", String, controller.kws_callback)
    rospy.Subscriber("botty/camera/object_location", String, controller.camera_callback)

    rospy.spin()

#!/usr/bin/python

import rospy
from std_msgs.msg import String

''' 
Parses/Interprets speech data from recognizer '/kws_data'
and sends list of commands to '/botty/parser/commands'
'''
class Parser:
	def __init__(self):
	    self.pub = rospy.Publisher('/botty/parser/commands', String, queue_size=10)

	def kws_callback(self, detected_words):
	    detected_words.data = detected_words.data.lower()
	    rospy.loginfo(detected_words.data)
	    # Momentan nur direkte uebergabe der keywords 
	    self.pub.publish(detected_words)

if __name__ == '__main__':
    rospy.init_node('parser')
    parser = Parser()

    rospy.Subscriber('/kws_data', String, parser.kws_callback)

    rospy.spin()

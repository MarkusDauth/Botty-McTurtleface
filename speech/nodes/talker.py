#!/usr/bin/env python
''' 
Author: David Kostka
Datum: 17.12.19

Description:
Import and use:
play(num) to play sound; 'num' is the sound id.
say(txt) to play TTS message.

For it to work, execute 'rosrun sound_play soundplay_node.py' first
'''

import rospy
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

class Talker:
    def __init__(self):
	self.soundhandle = SoundClient()
	rospy.sleep(1)

	self.voice = 'voice_kal_diphone'
	self.volume = 1.0

    def say(self, txt):
	print('Saying: %s' % txt)
	#print 'Voice: %s' % voice
	#print 'Volume: %s' % volume

	self.soundhandle.say(txt, self.voice, self.volume)
	rospy.sleep(1)

    def play(self, num):
	print('Playing sound nr.: %s' % num)
	self.soundhandle.play(num, self.volume)
	rospy.sleep(1)

#!/usr/bin/python

import os

import rospy
import time
from std_msgs.msg import String
from pocketsphinx.pocketsphinx import *
from sphinxbase.sphinxbase import *


class Recognizer(object):

    def __init__(self):

        # Initializing publisher with buffer size of 10 messages
        self.pub_ = rospy.Publisher("grammar_data", String, queue_size=10)
        
        # initialize node
        rospy.init_node("recognizer")
        # Call custom function on node shutdown
        rospy.on_shutdown(self.shutdown)

        # Params

        # File containing language model
        _hmm_param = "~hmm"
        # Dictionary
        _dict_param = "~dict"

        # List of keywords to detect
        _kws_param = "~kws"
	
	# File containing language model
        _lm_param = "~lm"
        # Gram file contains the rules and grammar
        _gram = "~gram"
        # Name of rule within the grammar
        _rule = "~rule"
        _grammar = "~grammar"
        # check if lm or grammar mode. Default = grammar
        
	self._use_lm = 0
        self.in_speech_bf = False
	
	# Listen for keyword if False
	self.wakeup = False

        # Setting param values
	
	# HMM
        if rospy.has_param(_hmm_param):
            self.class_hmm = rospy.get_param(_hmm_param)
            if rospy.get_param(_hmm_param) == ":default":
                if os.path.isdir("/usr/local/share/pocketsphinx/model"):
                    rospy.loginfo("Loading the default acoustic model")
                    self.class_hmm = "/usr/local/share/pocketsphinx/model/en-us/en-us"
                    rospy.loginfo("Done loading the default acoustic model")
                else:
                    rospy.logerr(
                        "No language model specified. Couldn't find defaut model.")
                    return
        else:
            rospy.loginfo("Couldn't find lm argument")
	
	# DICT
        if rospy.has_param(_dict_param) and rospy.get_param(_dict_param) != ":default":
            self.lexicon = rospy.get_param(_dict_param)
        else:
            rospy.logerr(
                'No dictionary found. Please add an appropriate dictionary argument.')
            return

	# KWS
        if rospy.has_param(_kws_param) and rospy.get_param(_kws_param) != ":default":
            self.kw_list = rospy.get_param(_kws_param)
        else:
            rospy.logerr(
                'kws cant run. Please add an appropriate keyword list.')
            return
	
	# GRAM 	
	if rospy.has_param(_grammar) and rospy.get_param(_grammar) != ":default":
            pass
        else:
            rospy.logerr(
                "No grammar found. Please add an appropriate grammar along with gram file.")
            return
	
	# LM
        if rospy.has_param(_lm_param) and rospy.get_param(_lm_param) != ':default':
            self._use_lm = 1
            self.class_lm = rospy.get_param(_lm_param)
        elif rospy.has_param(_gram) and rospy.has_param(_rule):
            self._use_lm = 0
            self.gram = rospy.get_param(_gram)
            self.rule = rospy.get_param(_rule)
        else:
            rospy.logerr(
                "Couldn't find suitable parameters. Please take a look at the documentation")
            return

        # All params satisfied. Starting recognizer
        self.start_recognizer()

    def start_recognizer(self):
        """Function to handle keyword spotting of audio"""

        config = Decoder.default_config()
        rospy.loginfo("Pocketsphinx initialized")

        # Setting configuration of decoder using provided params
        config.set_string('-hmm', self.class_hmm)
        config.set_string('-dict', self.lexicon)
        config.set_string('-dither', "no")
        config.set_string('-featparams', os.path.join(self.class_hmm, "feat.params"))

        # Keyword list file for keyword searching
        config.set_string('-kws', self.kw_list)

        # Set required configuration for decoder
        #self.decoder = Decoder(config)
	
	# Check if language model to be used or grammar mode
        if self._use_lm:
            rospy.loginfo('Language Model Found.')
            config.set_string('-lm', self.class_lm)
            self.decoder = Decoder(config)
        else:
            rospy.loginfo(
                'language model not found. Using JSGF grammar instead.')
            self.decoder = Decoder(config)

            # Switch to JSGF grammar
            jsgf = Jsgf(self.gram + '.gram')
            rule = jsgf.get_rule(rospy.get_param('~grammar') + '.' + self.rule)
            # Using finite state grammar as mentioned in the rule
            rospy.loginfo(rospy.get_param('~grammar') + '.' + self.rule)
            fsg = jsgf.build_fsg(rule, self.decoder.get_logmath(), 7.5)
            rospy.loginfo("Writing fsg to " +
                          self.gram + '.fsg')
            fsg.writefile(self.gram + '.fsg')

            self.decoder.set_fsg(self.gram, fsg)
            #self.decoder.set_search(self.gram)
	
	# Switch to Keyword spotting mode
	self.decoder.set_kws('kw', self.kw_list)
        self.decoder.set_search('kw')

	# Start processing input audio
        self.decoder.start_utt()
        rospy.loginfo("Decoder started successfully")

        # Subscribe to audio topic
        rospy.Subscriber("sphinx_audio", String, self.process_audio)
        rospy.spin()

    def process_audio(self, data):
        """Audio processing based on decoder config"""

	# Check if keyword detected

	# Actual processing
	self.decoder.process_raw(data.data, False, False)

	# Switch between Keyword and Grammar mode
	if self.decoder.get_search() == 'kw':
		self.keyword_search()
		#time.sleep(1)
		
	else:
		self.grammar_search()

    def grammar_search(self):
	if self.decoder.get_in_speech() != self.in_speech_bf:
		self.in_speech_bf = self.decoder.get_in_speech()
		if not self.in_speech_bf:
			self.decoder.end_utt()
			if self.decoder.hyp() != None:
				rospy.loginfo('OUTPUT: ' + self.decoder.hyp().hypstr)
                    		self.pub_.publish(self.decoder.hyp().hypstr)
				self.decoder.set_search('kw')
			self.decoder.start_utt()

    def keyword_search(self):
	if self.decoder.hyp() != None:
		self.decoder.end_utt()

		rospy.loginfo("Detected keyphrase, listening ...")
		self.decoder.set_search(self.gram)	 
		
		self.decoder.start_utt()
 
    @staticmethod
    def shutdown():
        """This function is executed on node shutdown."""
        # command executed after Ctrl+C is pressed
        rospy.loginfo("Stop Recognizer")
        rospy.sleep(1)


if __name__ == "__main__":
    Recognizer()

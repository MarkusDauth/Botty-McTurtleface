#!/usr/bin/python
# -*- coding: utf-8 -*-

''' 
Author: David Kostka
Datum: 13.12.19

Description:
Parses/Interprets speech data from recognizer 'botty/speech/grammar_data'
and sends list of Actions to '/botty/speech/commands'

Momentan ist es eine einfache Suche auf Woerter mit Semantischer bedeutung im einkommenden Sprachbefehl. 
Danach die zusammenstellung einer Datenstruktur im JSON Format,
welches die gewuenschte Aktion, das Objekt (bzw. Ort) der Aktion, und Attribute des Objekts beinhaltet (bis jetzt nur Farbe)
Eingabe ist der erkannte Text vom Recognizer durch Topic 'botty/speech/grammar_data'
JSON OBjekt wird zu '/botty/speech/commands' gesendet

Der Code ist noch sehr haesslich, für die 2. Demo reichts aber erstmal aus.

Mögliche Verbesserungen und Ziele:
Tokens sollten automatisch mit denen der Grammatik uebereinstimmen, dass keine Fehler entstehen
Eigene Klasse für Aktion-Struktur
Unterscheidung von "the" und "a", ... Prädikatenlogik ...

Mapping von Synonymen auf eindeutige Aktionen:
grab, hold, take -> grab

Spaeter waere es schoen, für Objekte auch Positionen anzugeben:
"bring (me) a red ball (thats) >in the kitchen<"

Und ein Ziel-Ort:
"bring the red ball from the kitchen >to the bedroom<"

Oder das verketten von Befehlen:
"grab a white cup >and< go to the living room"
'''

import rospy
from talker import *
from std_msgs.msg import String
from controller.msg import Command, Entity
#from controller.Botty import *
import re

from enum import Enum
class Action:
	GO = 1
	GRAB = 2
	BRING = 3
	STOP = 4

	SYNONYMS = 	{
			STOP: ['stop', 'halt', 'abort', 'kill', 'panic'], 
			GO: ['go', 'move'], 
			GRAB: ['grab', 'hold', 'take'], 
			BRING: ['bring']
			}

	@classmethod
	def synonym(cls, txt):
		txt = txt.lower()
		for nr, synlist in cls.SYNONYMS.items():
			for syn in synlist:
				if txt == syn:
					return nr
	@classmethod
	def name(cls, num):
		return cls.SYNONYMS[num][0]

class Parser:
	def __init__(self):
		# Evntl. ein eigenen Message-Typ definieren, nicht einfach als String schicken ...
		self.pub = rospy.Publisher('/botty/speech/commands', Command, queue_size=10)
		self.pub_cmd = rospy.Publisher('/botty/controller/stop', Command, queue_size=10)
		
		# Talker für TTS und sounds
		self.talker = Talker()		

		# Liste von Tokens, sollte man evtl. in eigene Klasse packen, per Datei einlesen
		# Muss mit .gram Datei uebereinstimmen
		action_list = {"go", "grab", "bring", "stop", "abort"}
		attr_list = {"blue", "red", "green", "white", "black"}
		obj_list = {"ball", "cup", "object"}
		place_list = {"kitchen", "living room", "bedroom", "garage", "docking station"}

		# Regex Objekte zur Suche 
		self.actions = re.compile("|".join(action_list))
		self.objects = re.compile("|".join(obj_list))
		self.attributes = re.compile("|".join(attr_list))
		self.places = re.compile("|".join(place_list))

		self.is_valid = True

	def callback(self, detected_words):
		self.is_valid = True
		txt = detected_words.data.lower()
		
		cmd = Command()
		obj = Entity()

		try: 	action = Action.synonym(self.actions.search(txt).group(0))
		except: self.sendError("Unknown Action")
		
		act_str = Action.name(action)
		
		if action == Action.STOP:
			self.pub_cmd.publish(cmd)
		# Wenn Aktion "bring" oder "grab" ist, enthaltet sie auch ein Objekt
		elif action == Action.BRING or action == Action.GRAB:
			try:	obj.name = self.objects.search(txt).group(0)
			except: self.sendError("Invalid object for action: " + act_str)
			if self.attributes.search(txt):			
				obj.attr.append(self.attributes.search(txt).group(0))
		# Bei "go" kann das Objekt nur ein Ort sein		
		elif action == Action.GO:
			try: 	obj.name = self.places.search(txt).group(0)
			except: self.sendError("Invalid place for action: " + act_str)
		
		# Command wird in JSON Format gesendet
		if self.is_valid and action != Action.STOP:
			cmd.action = action
			cmd.obj = obj
			rospy.loginfo(cmd)
			self.pub.publish(cmd)
			self.talker.play(2)
			self.talker.say(act_str + 'ing' + txt[len(act_str):])

	# Mögliche Fehler entstehen durch fehlende übereinstimmung der Tokens mit der Grammatik
	def sendError(self, txt):
		self.talker.play(3)
		self.talker.say(txt)
		self.is_valid = False

if __name__ == '__main__':
	rospy.init_node('parser')
	parser = Parser()
	rospy.Subscriber('botty/speech/grammar_data', String, parser.callback)
	rospy.spin()

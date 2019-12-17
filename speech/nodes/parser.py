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
import re
import json

class Parser:
	def __init__(self):
		# Evntl. ein eigenen Message-Typ definieren, nicht einfach als String schicken ...
		self.pub = rospy.Publisher('/botty/speech/commands', String, queue_size=10)
		
		# Talker für TTS und sounds
		self.talker = Talker()		

		# Liste von Tokens, sollte man evtl. in eigene Klasse packen, per Datei einlesen
		# Muss mit .gram Datei uebereinstimmen
		action_list = {"go", "grab", "bring"}
		attr_list = {"blue", "red", "green", "white", "black"}
		obj_list = {"ball", "cup", "object"}
		place_list = {"kitchen", "living room", "bedroom", "garage"}

		# Regex Objekte zur Suche 
		self.actions = re.compile("|".join(action_list))
		self.objects = re.compile("|".join(obj_list))
		self.attributes = re.compile("|".join(attr_list))
		self.places = re.compile("|".join(place_list))

		self.is_valid = True

	def callback(self, detected_words):
		self.is_valid = True
		txt = detected_words.data.lower()

		action = "undef"
		obj = ""
		attr = ""
		
		try: 	action = self.actions.search(txt).group(0)
		except: self.sendError("Unknown Action")
		
		# Wenn Aktion "bring" oder "grab" ist, enthaltet sie auch ein Objekt
		if action in "bring grab":
			try:	obj = self.objects.search(txt).group(0)
			except: self.sendError("Invalid object for action: " + action)
			if self.attributes.search(txt):			
				attr = self.attributes.search(txt).group(0)
		# Bei "go" kann das Objekt nur ein Ort sein		
		elif action in "go":
			try: 	obj = self.places.search(txt).group(0)
			except: self.sendError("Invalid place for action: " + action)

		# Command wird in JSON Format gesendet
		if self.is_valid:
			json_txt = json.dumps({"action": action, "object": {"name": obj, "attr": attr}})
			self.pub.publish(json_txt)
			rospy.loginfo(json_txt)
			self.talker.play(2)
			self.talker.say(action + 'ing' + txt[len(action):])

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

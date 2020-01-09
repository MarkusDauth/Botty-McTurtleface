#!/usr/bin/python

from enum import Enum

class Action:
	GO = 1
	GRAB = 2
	BRING = 3
	STOP = 4
	SEARCH = 5	
	
	SYNONYMS = 	{
			STOP: ['stop', 'halt', 'abort', 'kill', 'panic'], 
			GO: ['go', 'move'], 
			GRAB: ['grab', 'hold', 'take'], 
			BRING: ['bring'],
			SEARCH: ['search', 'find']
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


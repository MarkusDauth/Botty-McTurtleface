#!/usr/bin/env python
# -*- coding: utf-8 -*-
#this document is made within a study project of the 'Hochschule Kaiserslautern'
#made by Felix Mayer
#version 1.0

import rospy
from xml.dom.minidom import *
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from lidar.msg import Hints

rospy.loginfo("-----")

#some globals
global messurements
global pub
deathAngles=[]
global singleCallibrate
singleCallibrate=True
radius=18 #value is rounded. exact value is 17.7 CM
hokuyoRange=240
#Left Messurement Values
global messurementsLeftStart
global messurementsLeftEnd
global messurementsLeftSites
global siteLeftIncrement
#Front Messurement Values
global messurementsFrontStart
global messurementsFrontEnd
global messurementsFrontSites
global siteFrontIncrement
#Rigth Messurement Values
global messurementsRigthStart
global messurementsRigthEnd
global messurementsRigthSites
global siteRigthIncrement

#loading the configs out of the XML
print("Trying to read the configs")
print("-----")
try:
    configs=parse("config.xml")
except:
    raise SystemError('ERROR occurred during reading the HokuyoInterpreter-Configs. File "config.xml" not found. Try "sudo chmod +x config.xml" with its folder')
try:
    angles=configs.getElementsByTagName('angle')
    for angle in angles:
        assert(angle.firstChild.data in ["rigth","left","front"])
        try:
            if (angle.firstChild.data=="left"):
                leftStart=int(angle.getAttribute("start"))
                leftEnd=int(angle.getAttribute("end"))
                leftSites=int(angle.getAttribute("site"))
        except:
            raise SystemError('ERROR occurred during reading the HokuyoInterpreter-Configs. Please check the "left angle" propertys')
        try:
            if (angle.firstChild.data=="rigth"):
                rigthStart=int(angle.getAttribute("start"))
                rigthEnd=int(angle.getAttribute("end"))
                rigthSites=int(angle.getAttribute("site"))
        except:
            raise SystemError('ERROR occurred during reading the HokuyoInterpreter-Configs. Please check the "rigth angle" propertys')
        try:
            if (angle.firstChild.data=="front"):
                frontStart=int(angle.getAttribute("start"))
                frontEnd=int(angle.getAttribute("end"))
                frontSites=int(angle.getAttribute("site"))
        except:
            raise SystemError('ERROR occurred during reading the HokuyoInterpreter-Configs. Please check the "front angle" propertys')
    collisionBlocker=int(configs.getElementsByTagName("collision")[0].getAttribute("range"))
    callibration=float(configs.getElementsByTagName("callibration")[0].getAttribute("range"))
    constantSecureRange=float(configs.getElementsByTagName("constantSecureRange")[0].getAttribute("range"))
except:
    raise SystemError('ERROR occurred during reading the HokuyoInterpreter-Configs. Please check the "config.xml" file')


def checkRanges():
    #testing, if all angles are used and checking for upper and lower limits
    ranges=[[leftStart,leftEnd],[rigthStart,rigthEnd],[frontStart,frontEnd]]
    ranges.sort()
    assert(ranges[0][0]==1)
    checkRange=[]
    for segment in ranges:
        if ((segment[0]>hokuyoRange) or (segment[0]<1) or (segment[1]<1) or (segment[1]>hokuyoRange)):
            raise SystemError('ERROR occurred during checking the Angle-Configs. Start- and Endpoints must be between 1 and hokuyoRange!')
        for element in range(segment[0],segment[1]+1):
            if (element not in checkRange):
                checkRange.append(element)
    if (len(checkRange)!=hokuyoRange):
        print("WARNING - not all Degrees are used! ",len(checkRange)," counted")
        print("-----")
    

def runCallibrate(data):
    #Callibrating the Hokuyo/Lidar
    #Left Messurement Values
    global messurementsLeftStart
    global messurementsLeftEnd
    global messurementsLeftSites
    global siteLeftIncrement
    #Front Messurement Values
    global messurementsFrontStart
    global messurementsFrontEnd
    global messurementsFrontSites
    global siteFrontIncrement
    #Rigth Messurement Values
    global messurementsRigthStart
    global messurementsRigthEnd
    global messurementsRigthSites
    global siteRigthIncrement
    global messurements
    print("-Starting Callibration Session with Callibration-Range "+str(callibration)+"meters -")
    rospy.loginfo("-----")
    messurements=len(data.ranges)
    for counter in range(0,messurements):
        if data.ranges[counter]<=callibration:
            deathAngles.append(counter)
    print("Death Angles are...")
    print(deathAngles)
    rospy.loginfo("Death Angles are...")
    rospy.loginfo(deathAngles)
    rospy.loginfo("-----")



    #perentage conversion of the angles into the messurements range(Hokuyo has 240 degree and is reading ca. 512 values)
    frontSiteProzent=0
    messurementsFrontSites=0
    frontStartProzent=0
    messurementsFrontStart=0
    frontEndProzent=0
    messurementsFrontEnd=0
    siteFrontIncrement=0
    #Front site conversion
    if frontSites!=0:
        frontSiteProzent=float(hokuyoRange)/frontSites
        messurementsFrontSites=int(float(messurements)/frontSiteProzent)
    #Front start conversion
    if frontStart!=0:
        frontStartProzent=float(hokuyoRange)/frontStart
        messurementsFrontStart=int(float(messurements)/frontStartProzent)
    #Front end conversion
    if frontEnd!=0:
        frontEndProzent=float(hokuyoRange)/frontEnd
        messurementsFrontEnd=int(float(messurements)/frontEndProzent)
    #Calculation of the incrementiell site reduction
    if messurementsFrontSites!=0:
        siteFrontIncrement=float(collisionBlocker)/messurementsFrontSites
    #same for left and rigth
    #left
    leftSiteProzent=0
    messurementsLeftSites=0
    leftStartProzent=0
    messurementsLeftStart=0
    leftEndProzent=0
    messurementsLeftEnd=0
    siteLeftIncrement=0
    if leftSites!=0:
        leftSiteProzent=float(hokuyoRange)/leftSites
        messurementsLeftSites=int(float(messurements)/leftSiteProzent)
    if leftStart!=0:
        leftStartProzent=float(hokuyoRange)/leftStart
        messurementsLeftStart=int(float(messurements)/leftStartProzent)
    if leftEnd!=0:
        leftEndProzent=float(hokuyoRange)/leftEnd
        messurementsLeftEnd=int(float(messurements)/leftEndProzent)
    if messurementsLeftSites!=0:
        siteLeftIncrement=float(collisionBlocker)/messurementsLeftSites
    #rigth
    rigthSiteProzent=0
    messurementsRigthSites=0
    rigthStartProzent=0
    messurementsRigthStart=0
    rigthEndProzent=0
    messurementsRigthEnd=0
    siteRigthIncrement=0
    if rigthSites!=0:
        rigthSiteProzent=float(hokuyoRange)/rigthSites
        messurementsRigthSites=int(float(messurements)/rigthSiteProzent)
    if rigthStart!=0:
        rigthStartProzent=float(hokuyoRange)/rigthStart
        messurementsRigthStart=int(float(messurements)/rigthStartProzent)
    if rigthEnd!=0:
        rigthEndProzent=float(hokuyoRange)/rigthEnd
        messurementsRigthEnd=int(float(messurements)/rigthEndProzent)
    if messurementsRigthSites!=0:
        siteRigthIncrement=float(collisionBlocker)/messurementsRigthSites

def shutdown():
    rospy.loginfo("Ros Shutdown")

def hokuyoInterpreter():
    #starts Process
    global pub
    pub=rospy.Publisher('/botty/hokuyoInterpreter',Hints,queue_size=10)
    rospy.init_node('hokuyoInterpreter',anonymous=False)
    rospy.Subscriber("scan",LaserScan,callback)
    rospy.spin()

def proofHints(messurementsStart,messurementsEnd,messurementsSites,siteIncrement,data,hintList):
    #checking for Hints
    siteDistance=0.0 #Reduction of the secure distance to the sites
    for degree in range(messurementsStart,messurementsEnd):
        if siteIncrement!=0:
            if (degree<(messurementsSites+messurementsStart)): #if on the left site auf the messurement area, increase the 'siteIncrement'
                siteDistance+=siteIncrement
            if ((messurementsEnd-messurementsSites)<=(messurementsStart+(degree-messurementsStart))):  #if on the rigth site auf the messurement area, reduce the 'siteIncrement'
                siteDistance-=siteIncrement
        else:
            siteDistance=collisionBlocker
        secureRange=(radius+constantSecureRange+siteDistance)/100 # '/100' => cm to meter
        if (str(data.ranges[degree])!="inf"):
	    angle=int(hokuyoRange/(float(messurements)/degree))
            #if there's no deathangle AND no hints are to near AND this angle value is not already inside the container
            if ((degree not in deathAngles)and((data.ranges[degree])<(secureRange))and(angle not in hintList)):
                hintList.append(int(hokuyoRange/(float(messurements)/degree)))  #angle             
                print("-----")
        else:
            pass
    return hintList

def callback(data):
    #Left Messurement Values
    global messurementsLeftStart
    global messurementsLeftEnd
    global messurementsLeftSites
    global siteLeftIncrement
    #Front Messurement Values
    global messurementsFrontStart
    global messurementsFrontEnd
    global messurementsFrontSites
    global siteFrontIncrement
    #Rigth Messurement Values
    global messurementsRigthStart
    global messurementsRigthEnd
    global messurementsRigthSites
    global siteRigthIncrement
    #/
    global pub
    global messurements
    frontHints=[]
    leftHints=[]
    rigthHints=[]
    #Callibrates ones
    global singleCallibrate
    if singleCallibrate:
        runCallibrate(data)
        singleCallibrate=False
    else: #Work-Process after Callibration Run
        #Proof for all Hints in every area
        frontHints=proofHints(messurementsFrontStart,messurementsFrontEnd,messurementsFrontSites,siteFrontIncrement,data,frontHints)
        leftHints=proofHints(messurementsLeftStart,messurementsLeftEnd,messurementsLeftSites,siteLeftIncrement,data,leftHints)
        rigthHints=proofHints(messurementsRigthStart,messurementsRigthEnd,messurementsRigthSites,siteRigthIncrement,data,rigthHints)
        #send hint informationen
        print("Left: "+str(len(leftHints))+" hints counted")
        print("Front: "+str(len(frontHints))+" hints counted")
        print("Rigth: "+str(len(rigthHints))+" hints counted")
        output=Hints()
        output.front=frontHints
	#Note that the Hokuyo is messing counter-clockwise (ger="entgegen dem Uhrzeigersinn"), because of that are left and rigth 	  #swapped. Left and rigth are in this code orientated on the driving direction
        output.rigth=leftHints
        output.left=rigthHints
        pub.publish(output)
    print("-----")


if __name__=='__main__':
    checkRanges()
    hokuyoInterpreter()
    rospy.loginfo("Node and Process ended | HokuyoInterpreter")

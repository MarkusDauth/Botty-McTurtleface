#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from xml.dom.minidom import *
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from lidar.msg import Hints
from lidar.srv import HintsService,HintsServiceResponse

rospy.loginfo("-----")

#some globals
global lastOutput
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
rospy.loginfo("Trying to read the configs")
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
        rospy.logwarn("WARNING - not all Degrees are used! ",len(checkRange)," counted")
    

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
    rospy.loginfo("-Starting Callibration Session with Callibration-Range "+str(callibration)+"meters -")
    messurements=len(data.ranges)
    for counter in range(0,messurements):
        if data.ranges[counter]<=callibration:
            if counter not in deathAngles:
                deathAngles.append(counter)
            if counter not in deathAngles:
                deathAngles.append(counter-1)
            if counter not in deathAngles:
                deathAngles.append(counter+1)
    rospy.loginfo("Death Angles are...")
    rospy.loginfo(deathAngles)

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

def stopp():
    rospy.loginfo("Ros Shutdown")

def hokuyoInterpreter():
    #starts Process
    global pub
    pub=rospy.Publisher('/botty/hokuyoInterpreter',Hints,queue_size=10)
    rospy.init_node('hokuyoInterpreter',anonymous=False)
    rospy.on_shutdown(stopp)
    rospy.Subscriber("scan",LaserScan,callback)
    service = rospy.Service('lidar', HintsService, info)
    rospy.spin()

def proofHints(messurementsStart,messurementsEnd,messurementsSites,siteIncrement,data):
    hintListAngle=[]
    hintListDistance=[]
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
            #if there's no deathangle AND the hints are to near AND this angle value is not already inside the container
            if (degree not in deathAngles)and((data.ranges[degree])<(secureRange))and(angle not in hintListAngle):
                hintListAngle.append(angle)
                hintListDistance.append(data.ranges[degree])
        else:
            pass
    return hintListAngle,hintListDistance

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
    global lastOutput
    frontHintsAngle=[]
    leftHintsAngle=[]
    rigthHintsAngle=[]
    frontHintsDistance=[]
    leftHintsDistance=[]
    rigthHintsDistance=[]
    #Callibrates ones
    global singleCallibrate
    if singleCallibrate:
        runCallibrate(data)
        singleCallibrate=False
    else:
        #Work-Process after Callibration Run
        #Proof for all Hints in every area
        frontAngle,frontDistance=proofHints(messurementsFrontStart,messurementsFrontEnd,messurementsFrontSites,siteFrontIncrement,data)
        leftAngle,leftDistance=proofHints(messurementsLeftStart,messurementsLeftEnd,messurementsLeftSites,siteLeftIncrement,data)
        rigthAngle,rigthDistance=proofHints(messurementsRigthStart,messurementsRigthEnd,messurementsRigthSites,siteRigthIncrement,data)
        frontHintsAngle=frontAngle
        frontHintsDistance=frontDistance
        leftHintsAngle=leftAngle
        leftHintsDistance=leftDistance
        rigthHintsAngle=rigthAngle
        rigthHintsDistance=rigthDistance
        #send hint informationen
        rospy.loginfo("Left: "+str(len(leftHintsAngle))+" hints counted")
        rospy.loginfo("Front: "+str(len(frontHintsAngle))+" hints counted")
        rospy.loginfo("Rigth: "+str(len(rigthHintsAngle))+" hints counted")
        output=HintsServiceResponse()
        output.frontAngle=frontHintsAngle
        #Note that the Hokuyo is messing counter-clockwise (ger="entgegen dem Uhrzeigersinn"), because of that are left and rigth 	  
        #swapped. Left and rigth are in this code orientated on the driving direction
        output.rigthAngle=rigthHintsAngle
        output.leftAngle=leftHintsAngle
        output.frontDistance=frontHintsDistance
        output.leftDistance=leftHintsDistance
        output.rigthDistance=rigthHintsDistance
        lastOutput=output
        pub.publish(output)

def info(stuff):
    global lastOutput
    return lastOutput

if __name__=='__main__':
    checkRanges()
    hokuyoInterpreter()
    rospy.loginfo("Node and Process ended | HokuyoInterpreter")

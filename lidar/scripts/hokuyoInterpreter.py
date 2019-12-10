#!/usr/bin/env python
import rospy
from xml.dom.minidom import *
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
#import os

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
    #print(messurements)
    for counter in range(0,messurements):
        if data.ranges[counter]<=callibration:
            deathAngles.append(counter)
    print("Death Angles are...")
    print(deathAngles)
    rospy.loginfo("Death Angles are...")
    rospy.loginfo(deathAngles)
    rospy.loginfo("-----")

    #Prozentuale Umrechnung der Winkelgrad Verhaeltnisse zu der Wahrnehmungsgroesze (Hokuyo hat 240 Grad Wahrnehmung und liest ca. 512 Werte)
    #Front Seiten Umrechnung
    frontSiteProzent=float(hokuyoRange)/frontSites
    messurementsFrontSites=int(float(messurements)/frontSiteProzent)
    #Front Start Umrechnung
    frontStartProzent=float(hokuyoRange)/frontStart
    messurementsFrontStart=int(float(messurements)/frontStartProzent)
    #Front End Umrechnung
    frontEndProzent=float(hokuyoRange)/frontEnd
    messurementsFrontEnd=int(float(messurements)/frontEndProzent)
    #Berrechnung der Inkrementiellen Seiten Reduktion
    siteFrontIncrement=float(collisionBlocker)/messurementsFrontSites

    #das selbe für links und rechts
    #links
    leftSiteProzent=float(hokuyoRange)/leftSites
    messurementsLeftSites=int(float(messurements)/leftSiteProzent)
    leftStartProzent=float(hokuyoRange)/leftStart
    messurementsLeftStart=int(float(messurements)/leftStartProzent)
    leftEndProzent=float(hokuyoRange)/leftEnd
    messurementsLeftEnd=int(float(messurements)/leftEndProzent)
    siteLeftIncrement=float(collisionBlocker)/messurementsLeftSites
    #rechts
    rigthSiteProzent=float(hokuyoRange)/rigthSites
    messurementsRigthSites=int(float(messurements)/rigthSiteProzent)
    rigthStartProzent=float(hokuyoRange)/rigthStart
    messurementsRigthStart=int(float(messurements)/rigthStartProzent)
    rigthEndProzent=float(hokuyoRange)/rigthEnd
    messurementsRigthEnd=int(float(messurements)/rigthEndProzent)
    siteRigthIncrement=float(collisionBlocker)/messurementsRigthSites

def hokuyoInterpreter():
    #starts Process
    global pub
    #pub=rospy.Publisher('/mobile_base/commands/velocity',Twist,queue_size=10)
    pub=rospy.Publisher('/botty/hokuyoInterpreter',String,queue_size=10)
    rospy.init_node('hokuyoInterpreter',anonymous=False)
    rospy.Subscriber("scan",LaserScan,callback)
    rospy.spin()

def proofHints(messurementsStart,messurementsEnd,messurementsSites,siteIncrement,data,hintList):
    #checking for Hints
    siteDistance=0.0 #Verringerung des Sicherheitsabstand zur Seite hin
    hintList.clear() #clean up the last Collection of dedected hints
    for degree in range(messurementsStart,messurementsEnd+1):
        if (degree<(messurementsSites+messurementsStart)): #wenn der derzeitige Abschnitt sich aud der linken Site befindet,erhoehe die siteIncrement
            siteDistance+=siteIncrement
        if ((messurementsEnd-messurementsSites)<=(messurementsStart+(degree-messurementsStart))):  #wenn der derzeitige Abschnitt sich aud der rechten RigthSite befindet,verringere die siteRigthIncrement
            siteDistance-=siteIncrement
        secureRange=(radius+constantSecureRange+siteDistance)/100 # '/100' um die Cm in Meter umzurechnen
	    #print("Grad: "+str(degree)+" Value: "+str(data.ranges[degree])+" Problem-Range: "+str(secureRange))
        if (str(data.ranges[degree])!="inf"):
            #wenn kein toter Winkel vorliegt und der Sicherheitsabstand nicht gewaehrleistet ist
            if ((degree not in deathAngles)and((data.ranges[degree])<(secureRange))):
                #twist=Twist()
                hintList.append([str(int(hokuyoRange/(float(messurements)/degree))),str(data.ranges[degree]),str(degree)])
                #print("DANGER! angle: "+hintList[len(hintList)-1][0]+" with distance: "+hintList[len(hintList)-1][1]+" at messurement: "+hintList[len(hintList)-1][2])
                print("-----")
                #pub.publish(twist)
                #break
        else:
            #print("'inf' got called; angle: "+str(degree)+" "+str(data.ranges[degree]))
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
    #Callibrates ones
    global singleCallibrate
    frontHints=[]
    leftHints=[]
    rigthHints=[]
    hints=[leftHints,frontHints,rigthHints]
    if singleCallibrate:
        runCallibrate(data)
        singleCallibrate=False
    else: #Work-Process after Callibration Run
        #Proof for all Hints in every area
        frontHints=proofHints(messurementsFrontStart,messurementsFrontEnd,messurementsFrontSites,siteFrontIncrement,data,frontHints)
        leftHints=proofHints(messurementsLeftStart,messurementsLeftEnd,messurementsLeftSites,siteLeftIncrement,data,leftHints)
        rigthHints=proofHints(messurementsRigthStart,messurementsRigthEnd,messurementsRigthSites,siteRigthIncrement,data,rigthHints)
        #sende die Hint Informationen, falls welche vorliegen
        print("Left: "+str(len(leftHints))+" hints counted")
        print("Front: "+str(len(frontHints))+" hints counted")
        print("Rigth: "+str(len(rigthHints))+" hints counted")
        if ((len(leftHints)+len(frontHints)+len(rigthHints))>0):
            print("Hints dedected!")
            pub.publish(hints)
        else:
            print("no hints dedected")
        print("-----")


if __name__=='__main__':
    try:
        checkRanges()
        hokuyoInterpreter()
        rospy.loginfo("Node and Process ended | HokuyoInterpreter")
        print(deathAngles)
    except:
        raise SystemError('If u end here, u have serious trouble...')

#!/usr/bin/env python
import rospy
from xml.dom.minidom import *
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
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

#loading the configs out of the XML
print("Trying to read the configs")
print("-----")
try:
    configs=parse("config.xml")
    angles=configs.getElementsByTagName('angle')
    for angle in angles:
        assert(angle.firstChild.data in ["rigth","left","front"])
        if (angle.firstChild.data=="left"):
            leftStart=int(angle.getAttribute("start"))
            leftEnd=int(angle.getAttribute("end"))
            leftSites=int(angle.getAttribute("site"))
        if (angle.firstChild.data=="rigth"):
            rigthStart=int(angle.getAttribute("start"))
            rigthEnd=int(angle.getAttribute("end"))
            rigthSites=int(angle.getAttribute("site"))
        if (angle.firstChild.data=="front"):
            frontStart=int(angle.getAttribute("start"))
            frontEnd=int(angle.getAttribute("end"))
            frontSites=int(angle.getAttribute("site"))
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

def hokuyoInterpreter():
    #starts Process
    global pub
    pub=rospy.Publisher('/mobile_base/commands/velocity',Twist,queue_size=10)
    rospy.init_node('hokuyoInterpreter',anonymous=False)
    rospy.Subscriber("scan",LaserScan,callback)
    rospy.spin()

def callback(data):
    global pub
    global messurements
    #Callibrates ones
    global singleCallibrate
    if singleCallibrate:
        runCallibrate(data)
        singleCallibrate=False
    else: #Work-Process after Callibration Run

        siteDistance=0.0 #Verringerung des Sicherheitsabstand zur Seite hin
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
        siteIncrement=float(collisionBlocker)/messurementsFrontSites


        for degree in range(messurementsFrontStart,messurementsFrontEnd+1):
            if (((degree-messurementsFrontStart)+messurementsFrontStart)<(messurementsFrontSites+messurementsFrontStart)): #wenn der derzeitige Abschnitt sich aud der linken FrontSite befindet,erhoehe die SiteIncrement
                siteDistance+=siteIncrement
            if ((messurementsFrontEnd-messurementsFrontSites)<=(messurementsFrontStart+(degree-messurementsFrontStart))):  #wenn der derzeitige Abschnitt sich aud der rechten FrontSite befindet,verringere die SiteIncrement
                siteDistance-=siteIncrement
	    secureRange=(radius+constantSecureRange+siteDistance)/100 # '/100' um die Cm in Meter umzurechnen
	    #print("Grad: "+str(degree)+" Value: "+str(data.ranges[degree])+" Problem-Range: "+str(secureRange))
            if (str(data.ranges[degree])!="inf"):
                #wenn kein toter Winkel vorliegt und der Sicherheitsabstand nicht gewaehrleistet ist
                if ((degree not in deathAngles)and((data.ranges[degree])<(secureRange))):
                    twist=Twist()
                    print("DANGER! angle: "+str(int(hokuyoRange/(float(messurements)/degree)))+" with distance: "+str(data.ranges[degree])+" at messurement: "+str(degree))
                    print("-----")
                    pub.publish(twist)
                    break
            else:
                #print("'inf' got called; angle: "+str(degree)+" "+str(data.ranges[degree]))
                pass
        print("-----")
   

if __name__=='__main__':
    try:
        checkRanges()
        hokuyoInterpreter()
        rospy.loginfo("Node and Process ended | HokuyoInterpreter")
	print(deathAngles)
    except rospy.ROSInterruptException:
        pass

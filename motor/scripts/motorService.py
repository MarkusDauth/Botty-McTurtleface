#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist
from motor.srv import call,callResponse
from lidar.srv import HintsService,HintsServiceResponse
from math import *
from time import sleep
#some globals
global view     #saving botty view (in degree)
global xCoord   #saving botty X Coordinate (in meter)
global yCoord   #saving botty Y Coordinate (in meter)
global pub      #the publisher node for the velocity
global speed    #bottys speed
global interrupt#interrupt vom controller
#Botty/Turtkebot erreicht nicht die angefragte Distanz und stoppt bereits früher. Folgende Tests haben folgende Ergebnisse geliefert:
#Sollwert=50 Cm; Istwert=ca. 42 Cm
#Sollwert=100 Cm; Istwert=ca. 85 Cm
#Sollwert=200 Cm; Istwert=ca. 167 Cm
#Daraus wurde ein fester Abweichungs Wert berrechnet von 1,2.
#Dieser wird im Code dem gewünschten Zielwert auf multipliziert und
#liefert einen exakteren Istwert, der eher dem Sollwert entspricht.
#ACHTUNG, d.h. der Sollwert wird nie ganz genau erreicht. Bitte keine Millimeter Arbeit für botty
#die Tests wurden mit einer Geschwindugkeit von 0.75 erprobt. #Mehrmaliges Wiedeholen hat identische Ergebnisse geliefert
#
#Ähnliches ist bei der Winkel Abweichung für das Drehen der Fall.
#Aufgrund der Beschleunigung würde eine 180 Grad Drehung mit den
#Abweichung Werten der 90 Grad Drehung über 180 hinaus gehen,
#während 2 * 90 die 180 trifft. Daher gibt es 4 verschieden 
#Abweichungskonstanten für 90,180,270 und 360 Grad. 
#Die Konstante entsprechen der benötigte Zeit für eine 360 Grad
#Drehung und wurden manuell fein justiert.
global abweichung
global wAbw
abweichung=1.2
wAbw90=(360/13.735)
wAbw180=(360/12.8125)
wAbw270=(360/12.045)
wAbw360=(360/11.25)
wAbw=[[90,wAbw90],[180,wAbw180],[270,wAbw270],[360,wAbw360]]
speed=0.75
xCoord=0
yCoord=0
view=0
interrupt=False
debug=False

def setSpeed(speedLocal):
    #set the speed, if it's given
    global speed
    if speedLocal!=None:
        speed=speedLocal

def forwardByMeter(meter,hokuyo=False):
    global abweichung
    global speed
    global pub
    global xCoord
    global yCoord
    global interrupt
    interrupt=False
    if hokuyo:
        rospy.wait_for_service('lidar')
        checkHints = rospy.ServiceProxy('lidar', HintsService)
    vel_msg = Twist()
    distance = meter*abweichung
    vel_msg.linear.x = abs(speed)
    #Since we are moving just in x-axis
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0
    #stopp twist
    twistToStopp=Twist()
    #Calculates the time, it needs to drive
    endTime=distance/speed
    drivenTime=0
    #Loop to move the turtle in an specified distance
    while((drivenTime < endTime) and not (rospy.is_shutdown() or interrupt)):
        timeNow=rospy.Time.now().to_sec()
        if hokuyo:
            response=checkHints()
            #Wenn kein Hindernis, dann fahre
            if len(response.frontAngle)==0:
                #Publish the velocity
                pub.publish(vel_msg)
                timeLater=rospy.Time.now().to_sec()
                drivenTime+=timeLater-timeNow
            else:
                pub.publish(twistToStopp)
                #sonst
                sleep(0.2)
                #wenn rechts frei ist...
                if len(response.rigthAngle)<5:
                    #dann drehe dich nach rechts
                    turnRigthByAngle(90)
                    pub.publish(twistToStopp)
                    sleep(0.2)
                    response=checkHints()
                    siteStartTime=rospy.Time.now().to_sec()
                    #und fahre bis links frei ist
                    while ((len(response.leftAngle)>5) and not (rospy.is_shutdown() or interrupt)):
                        #Publish the velocity
                        response=checkHints()
                        pub.publish(vel_msg)
                    #bewege dich noch ein Stück, um die eigene Masse um das Eck zu bekommen
                    forwardByMeter(0.25)
                    pub.publish(twistToStopp)
                    siteEndTime=rospy.Time.now().to_sec()
                    sleep(0.2)
                    siteTime=siteEndTime-siteStartTime
                    #dann drehe dich nach links
                    turnLeftByAngle(90)
                    pub.publish(twistToStopp)
                    sleep(0.2)
                    response=checkHints()
                    #solltest du links noch nichts sehen, fahre vorwärts
                    frontStartTime=rospy.Time.now().to_sec()
                    if len(response.leftAngle)<5:
                        while ((len(response.leftAngle)<5) and not (rospy.is_shutdown() or interrupt)):
                            #Publish the velocity
                            response=checkHints()
                            pub.publish(vel_msg)
                    #wenn direkt von dir links die Wand ist, dann fahre vorwärt (das Hindernis umgehen in ursprünglicher Fahrtrichtung)
                    while ((len(response.leftAngle)>5) and not (rospy.is_shutdown() or interrupt)):
                        #Publish the velocity
                        response=checkHints()
                        pub.publish(vel_msg)
                    #bewege dich noch ein Stück, um die eigene Masse um das Eck zu bekommen
                    forwardByMeter(0.25)
                    pub.publish(twistToStopp)
                    frontEndTime=rospy.Time.now().to_sec()
                    sleep(0.2)
                    #drehe dich nach links mit Sicht zum Ursprungspfad
                    turnLeftByAngle(90)
                    pub.publish(twistToStopp)
                    sleep(0.2)
                    #fahre nach links solange wie du nach rechts gefahren warst
                    backTime=rospy.Time.now().to_sec()
                    backEndTime=backTime+siteTime
                    while (backTime<backEndTime) and not (rospy.is_shutdown() or interrupt):
                        #Publish the velocity
                        backTime=rospy.Time.now().to_sec()
                        pub.publish(vel_msg)
                    pub.publish(twistToStopp)
                    sleep(0.2)
                    #dann drehe dich zurück
                    turnRigthByAngle(90)
                    pub.publish(twistToStopp)
                    sleep(0.2)
                    drivenTime+=frontEndTime-frontStartTime


                #wenn links frei ist...
                elif len(response.leftAngle)<5:
                    #dann drehe dich nach links
                    turnLeftByAngle(90)
                    pub.publish(twistToStopp)
                    sleep(0.2)
                    response=checkHints()
                    siteStartTime=rospy.Time.now().to_sec()
                    #und fahre bis rechts frei ist
                    while ((len(response.rigthAngle)>5) and not (rospy.is_shutdown() or interrupt)):
                        #Publish the velocity
                        response=checkHints()
                        pub.publish(vel_msg)
                    #bewege dich noch ein Stück, um die eigene Masse um das Eck zu bekommen
                    forwardByMeter(0.25)
                    pub.publish(twistToStopp)
                    siteEndTime=rospy.Time.now().to_sec()
                    sleep(0.2)
                    siteTime=siteEndTime-siteStartTime
                    #dann drehe dich nach rechts
                    turnRigthByAngle(90)
                    pub.publish(twistToStopp)
                    sleep(0.2)
                    response=checkHints()
                    #solltest du rechts noch nichts sehen, fahre vorwärts
                    frontStartTime=rospy.Time.now().to_sec()
                    if len(response.rigthAngle)<5:
                        while ((len(response.rigthAngle)<5) and not (rospy.is_shutdown() or interrupt)):
                            #Publish the velocity
                            response=checkHints()
                            pub.publish(vel_msg)
                    #wenn direkt von dir rechts die Wand ist, dann fahre vorwärt (das Hindernis umgehen in ursprünglicher Fahrtrichtung)
                    while ((len(response.rigthAngle)>5) and not (rospy.is_shutdown() or interrupt)):
                        #Publish the velocity
                        response=checkHints()
                        pub.publish(vel_msg)
                    #bewege dich noch ein Stück, um die eigene Masse um das Eck zu bekommen
                    forwardByMeter(0.25)
                    pub.publish(twistToStopp)
                    frontEndTime=rospy.Time.now().to_sec()
                    sleep(0.2)
                    #drehe dich nach rechts mit Sicht zum Ursprungspfad
                    turnRigthByAngle(90)
                    pub.publish(twistToStopp)
                    sleep(0.2)
                    #fahre nach rechts solange wie du nach links gefahren warst
                    backTime=rospy.Time.now().to_sec()
                    backEndTime=backTime+siteTime
                    while (backTime<backEndTime) and not (rospy.is_shutdown() or interrupt):
                        #Publish the velocity
                        backTime=rospy.Time.now().to_sec()
                        pub.publish(vel_msg)
                    pub.publish(twistToStopp)
                    sleep(0.2)
                    #dann drehe dich zurück
                    turnLeftByAngle(90)
                    pub.publish(twistToStopp)
                    sleep(0.2)
                    drivenTime+=frontEndTime-frontStartTime
                else:
                    rospy.logdebug("NO SPACE!")
                    pub.publish(twistToStopp)
                    sleep(0.2)
                    break
        else:        
            #Publish the velocity
            pub.publish(vel_msg)
            #Takes actual time to velocity calculus
            timeLater=rospy.Time.now().to_sec()
            drivenTime+=timeLater-timeNow
    #After the loop, stops the robot
    vel_msg.linear.x = 0
    #Force the robot to stop
    pub.publish(vel_msg)
    #calculating the reached Coords
    #simple Style; supports only front,left,back,rigth or in other words the below given degrees (0,90,180,270)
    if view==0:
        yCoord+=meter
    elif view==90:
        xCoord+=meter
    elif view==180:
        yCoord-=meter
    elif view==270:
        xCoord-=meter
    else:
        #hardcore Style. Calculates the coords by remembring the view and using 'Sinussatz' and 'Pytagoras'
        if nearRigthAngle(view)==0:
            x=meter/sin(radians(90))*sin(radians(view))
            y=sqrt((meter**2)-(x**2))
            if view<91:
                xCoord+=x
            else: 
                xCoord-=x
            yCoord+=y
        elif nearRigthAngle(view)==180:
            x=meter/(sin(radians(90)))*sin(radians(180-view))
            y=sqrt((meter**2)-(x**2))
            if view<181:
                xCoord+=x
            else:
                xCoord-=x
            yCoord-=y


def nearRigthAngle(angle):
    #chacks if the degree 180 or 0 is nearer to the given degree; actualy its expandable to more degrees. (Example: rigthAngles=[0,90,180,270])
    rigthAngles=[0,180]
    smallest=5000
    smallestPosition=0
    for degree in range(0,len(rigthAngles)):
        if abs(rigthAngles[degree]-angle)<smallest:
            smallest=abs(rigthAngles[degree]-angle)
            smallestPosition=degree
    return rigthAngles[smallestPosition]

def calculateTurnTwist(angle):
    #initiate the Twist object
    global speed
    global wAbw
    twist = Twist()
    twist.linear.x=0
    twist.linear.y=0
    twist.linear.z=0
    twist.angular.x = 0
    twist.angular.y = 0
    return twist

def turnLeftByAngle(angle,speedLocal=None):
    global view
    setSpeed(speedLocal)
    tunrTwist= calculateTurnTwist(angle)
    #turn counter clockwards
    tunrTwist.angular.z = abs(speed)
    moveTwist(tunrTwist,angle)
    view-=angle

def turnRigthByAngle(angle,speedLocal=None):
    global view
    setSpeed(speedLocal)
    turnTwist = calculateTurnTwist(angle)
    #turn clockwise
    turnTwist.angular.z = -abs(speed)
    moveTwist(turnTwist,angle)
    view+=angle

def moveTwist(twist,angle):
    global speed
    global pub
    global wAbw
    global interrupt
    interrupt=False
    #searching for the angle Configs/wAbw which fits best to the used
    targetWAbw=nearAngle(angle)
    #calculating turn speed and needed time
    angleSpeed=wAbw[targetWAbw][1]/0.75*speed
    endTime=rospy.Time.now().to_sec()+(angle/angleSpeed)
    currentTime=rospy.Time.now().to_sec()
    #turning until the time end is reached
    while((currentTime<endTime) and not (rospy.is_shutdown() or interrupt)):
        pub.publish(twist)
        currentTime = rospy.Time.now().to_sec()
    #Forcing our robot to stop
    twist.angular.z = 0
    pub.publish(twist)

def nearAngle(angle):
    #seacrh the nearest angle in the given list. (is param 'angle' closer to 90,180,270 or 360?)
    nearAngle=1000
    indexNotice=0
    for index in range(0,len(wAbw)):
        if abs(angle-wAbw[index][0])<nearAngle:
            nearAngle=abs(angle-wAbw[index][0])
            indexNotice=index
    return indexNotice

def stopp():
    #on Interrupt, stopp the motors
    global pub
    global interrupt
    interrupt=True
    pub.publish(Twist())
    rospy.loginfo("Keyboard Interrupt. Stopping Motor.")


def decode(command):
    #check wich command u received and check if it's parameters are correct
    global xCoord
    global yCoord
    done=False
    if command.call=="setSpeed":
        if len(command.param)==1:
            rospy.loginfo("Call: setSpeed("+command.param[0]+")")
            setSpeed(int(command.param[0]))
        else:
            rospy.logdebug("Call: setSpeed; Error: Wrong amount of Parameters! setSpeed takes 1 parameter =>speed")
        done=True
    if command.call=="forwardByMeters":
        if len(command.param)==1:
            rospy.loginfo("Call: forwardByMeter("+str(command.param[0])+")")
            forwardByMeter(command.param[0])
        elif len(command.param)==2:
            if command.param[1]==0:
                rospy.loginfo("Call: forwardByMeter("+str(command.param[0])+", "+str(command.param[1])+")")
                forwardByMeter(command.param[0])
            elif command.param[1]==1:
                rospy.loginfo("Call: forwardByMeter("+str(command.param[0])+", "+str(command.param[1])+")")
                forwardByMeter(command.param[0],True)
            else:
                rospy.logdebug("Call: forwardByMeter("+str(command.param[0])+", "+str(command.param[1])+"); Error: Second Argument only takes 1 or 0 to be converted to boolean!")
        else:
            rospy.logdebug("Call: forwardByMeters; Error: Wrong amount of parameters! forwardByMeters takes 1 parameter and 1 optional =>meters,hokuyo=False (take 0 or 1 to represent the boolean)")
        done=True
    if command.call=="turnRigthByAngle":
        if len(command.param)==2:
            rospy.loginfo("Call: turnRigthByAngle(angle="+str(command.param[0])+", speed="+str(command.param[1])+")")
            turnRigthByAngle(int(command.param[0]),int(command.param[1]))
        elif len(command.param)==1:
            rospy.loginfo("Call: turnRigthByAngle(angle="+str(command.param[0])+")")
            turnRigthByAngle(int(command.param[0]))
        else:
            rospy.logdebug("Call: turnRigthByAngle; Error: Wrong amount of Parameters! turnRigthByAngle takes 2 parameters; 1 is optional =>angle,speed=None")
        done=True
    if command.call=="turnLeftByAngle":
        if len(command.param)==2:
            rospy.loginfo("Call: turnLeftByAngle(angle="+str(command.param[0])+", speed="+command.param[1]+")")
            turnLeftByAngle(int(command.param[0]),int(command.param[1]))
        elif len(command.param)==1:
            rospy.loginfo("Call: turnLeftByAngle(angle="+str(command.param[0])+")")
            turnLeftByAngle(int(command.param[0]))
        else:
            rospy.logdebug("Call: turnLeftByAngle; Error: Wrong amount of Parameters! turnLeftByAngle takes 2 parameters; 1 is optional =>angle,speed=None")
        done=True
    if command.call=="stopp":
        if len(command.param)==0:
            rospy.loginfo("Call: stopp()")
	    stopp()
        else:
            rospy.logdebug("Call: stopp; Error: Wrong amount of Parameters! stopp takes 0 parameters")
        done=True
    if not done:
        rospy.logdebug("Error: Call-name is unknown")
    response=callResponse()
    #returning the current X and Y Coordinates
    response.success = not interrupt
    response.x=xCoord
    response.y=yCoord
    return response

def motor_server():
    #start the Service
    global pub
    rospy.loginfo("initiating motor node")
    rospy.init_node('botty_motor')
    rospy.loginfo("initiating motor Service")
    service = rospy.Service('motor', call, decode)
    pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
    rospy.on_shutdown(stopp)
    rospy.spin()


if __name__ == "__main__":
    motor_server()

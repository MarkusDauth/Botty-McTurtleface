# Botty-McTurtleface main Package

Dieses ROS-Package dient zur Steuerung des Turtlebot2 mit folgenden Komponenten:
- PhantomX Reactor Arm
- Hokuyo URG-04LX-UG01 (Lidar)
- Orbbec Astra (RD-Kamera)
- NUC

Das Botty-Package ist nochmal in mehrere Subpackages aufgeteilt.
- arm
- camera
- lidar
- speech_parser
- controller

# Installation

Dieses Repository dient als eigenes ROS-Package und muss daher in den src Ordner des Catkin_Workspaces geklont werden. Es müssen mehrere Programme über apt-get installiert werden und Git-Repositories in den src Ordner des 

Vor der Installation sollte das ausgeführt werden:
```
rosdep update
sudo apt-get update
sudo apt-get dist-upgrade
sudo apt-get install ros-kinetic-catkin python-catkin-tools
````

## PhantomX Reactor Arm
Dependencies installieren:
```
sudo apt-get install ros-kinetic-arbotix
sudo apt-get install ros-kinetic-dynamixel-controllers
sudo apt install ros-kinetic-moveit
```
Im Ordner “src” des Workspace dieses Repository downloaden:
```
git clone https://github.com/RobotnikAutomation/phantomx_reactor_arm.git
```

## Lidar
Im Ordner “src” des Workspace dieses Repository downloaden bzw. folgenden Instruktionen folgen:
```
git clone https://github.com/ros-drivers/driver_common.git
cd ..
catkin_make
cd src
git clone https://github.com/ros-drivers/hokuyo_node.git
cd ..
catkin_make
```

Troubleshooting: Es ist wichtig driver_common VORHER zu kompilieren, 
bevor hokuyo_node hinzugefügt wird. Sonst scheitert der Prozess!

## Package
Vor der erstmaligen ausführen von Programmen muss das Botty-Package kompiliert werden. Folgenden Befehl im catkin_ws Ordner ausführen:
```
cd ~/catkin_ws
catkin config --extend /opt/ros/kinetic
catkin build (eventuell kommt eine Fehlermeldung, dann kann man den Befehl ignorieren)
catkin_make  
```

# Programme ausführen

Für alle Programme muss vorher der Roscore gestartet werden. Mit dem Turtlebot geht das unter:
```
roslaunch kobuki_node minimal.launch
```

## PhantomX Reactor Arm
Arm-Nodes in eigener Konsole starten:
```
roslaunch phantomx_reactor_arm_controller arbotix_phantomx_reactor_arm_wrist.launch
```
Rviz-Demo starten:
```
roslaunch phantomx_reactor_arm_moveit_config demo.launch 
```

Python Skript für den Arm starten, z.B.:
```
rosrun botty pose_command.py
```

## Lidar
In einem Terminal folgende Instruktionen folgen:
```
sudo chmod a+rw /dev/ttyACM0
rosrun hokuyo_node hokuyo_node
rosrun hokuyoInterpreter hokuyoInterpreter.py
```

# Sonstiges

Teammitglieder:
Markus Dauth, Felix Mayer, David Kostka, Raschied Slet

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
- speech
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

## Speech
Falls 'pip' nicht installiert ist:
```
sudo apt install python-pip
sudo apt install python3-pip
pip install --upgrade pip
```

Dependencies installieren:
```
sudo apt-get install libasound-dev
sudo apt-get install python-pyaudio
Sudo apt-get install swig
```

Pocketsphinx installieren:
```
sudo pip install pocketsphinx
```

Package 'audio_common' für TTS und Sounds:
```
sudo apt-get install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
git clone https://github.com/ros-drivers/audio_common.git
catkin_make
rosdep install sound_play
```

## Controller
Bis jetzt keine Dependencies

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
rosrun lidar hokuyoInterpreter.py
```

## Speech
Recognizer/Parser starten:
```
roslaunch speech parser.launch
```

Nur Recognizer starten:
```
roslaunch speech recognizer.launch
```

Dazu benötigte Nodes 'send_audio.py' und 'soundplay_node.py' werden automatisch mit gestartet.

## Controller
Arm-steuerung per Sprachkommandos:
```
roslaunch controller arm_control.launch
```

Navigation per Sprachkommandos:
```
roslaunch controller base_control.launch
```

Das Speech-Modul wird beim launch automatisch gestartet, muss also nicht davor gestartet werden.

## Kamera
Auf eigenem Rechner starten:

```
roslaunch camera camera.launch
```
Auf dem Turtlebot-Rechner starten:

```
roslaunch astra_launch astra.launch
```

# Sonstiges

Teammitglieder:
Markus Dauth, Felix Mayer, David Kostka, Raschied Slet

# TODO
Es gibt eine "Main-Readme" (dieses Dokument) und in jedem unserer Packages (arm, camera, lidar, speech, controller) jeweils eine "Sub-Readme"

Was noch in der Doku erklärt werden muss:
* Was muss alles installiert werden? Eine große Installations anleitung in der Haupt Readme
* Wo stehen Angaben zu Version, Datum und Author?
* Wie kompiliert unser Projekt?
* Wie sieht die korrekte Ordnerstruktur aus
* Botty feste IP Adresse (10.0.189.60)

Was noch im Code erledigt werden muss:
* Author, Datum und Version an ansprechenden stellen anpassen
* Code muss Dokumentiert werden + Refactoring
* NIcht benutztn Code auskommentieren?

Geplante Struktur der Main-Readme
* Einleitung (z.B. Projektziel, wie liest man dieses Dokument?)
* Architektur
* Installation (Alle Installationen und Konfigurationen aufgeteilt nach den Botty-Packages auflisten)
* Demos (hier erklären, welche Demos ausgeführt werden können, z.B. "finde Blume an X,Y", Raster erklären)
* Aufgetretene Probleme (z.B. Stromprobleme, Lieferung, Kamera wurde umgebaut, Feste IP, Mikrofon und Lautsprecher)

Folgende Demos gibt es und sollten unter der Überschrift (Demos) gemacht werden:
- AN Koordinat fahren
- finde ObjektX an Koord X,Y
- Finde Objekt (Was siehst du)

Grundstruktur der Sub-Readmes (Template):
* Einleitung (Was soll wie gemacht werden? kurz die Funktionsweise erklären, Hardware kurz eklären, Links für Hardware)
* Verwendete Packages (Auflisten, Links, erklären und benutzt Tutorials verlinken)
* Dateien (Alle selbst geschriebenen Dateien erklären, aber nicht auf Code eingehen)
* Konfiguration (z.B. neue Bilder für die Kaemera trainieren, neue Posen für den Arm)
* Lessons Learned (Expereminte, die nicht funktioniert haben)
* Potenzielle Verbesserungen (hier erklären, was nachfolger noch machen könnte oder was besser sein kann)

Liste an Potenziellen Verbesserungen:
* Akku nachbestellen
* Winkel beim Drehen besser
* Weichere Bewegungsansteuerung
* Umgehung von Hinternissen, (A*) ist nicht implementiert im MOtor
* Bessere Kamera Erkennung
* Neue Befehle z.B. “Bewege dich auf Kachel 2,2"


Was berücksicht werden sollte:
- Kein Glossar oder Literaturverzeichnis
- Trello aufräumen
- Nicht erklären was der Roboter ist, sondern nur was gemacht wurde. Entsprechende Webseiten verlinken

Ab hier beginnt unsere Doku:

# Einleitung

Dieses ROS-Package dient zur Steuerung des Turtlebot2 mit folgenden Komponenten:
- PhantomX Reactor Arm
- Hokuyo URG-04LX-UG01 (Lidar)
- Orbbec Astra (RD-Kamera)
- NUC mit Ubuntu 16.04 LTS
- Kobuki Base
- Lautsprecher (mifa)
- Mikrofon (Docooler)

Das Botty-Package ist nochmal in mehrere Subpackages aufgeteilt.
- arm
- camera
- lidar
- speech
- controller
- motor

Jeder dieser Packages hat wiederum eine eigene ReadMe-Datei, welche die Details der einzelnen Packages erklärt.

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

## MotorService
In einem Terminal folgende Instruktionen folgen:
```
rosrun motor motorService.py
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

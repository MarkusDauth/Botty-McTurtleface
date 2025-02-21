# Einleitung
## Aufbau der Dokumentation
Dieses Readme-Datei dient als Einstieg für die Dokumentation dieses Projektes und stellt projektübergreifende Informationen dar. Die Dateien in diesem Repository dienen als [ROS](https://www.ros.org/)-Package zur Steuerung des [TurtleBot2](https://www.turtlebot.com/turtlebot2/).

Das Botty-Package ist in mehrere Subpackages aufgeteilt:
- arm (Steuerung des PhantomX Reactor Arm)
- camera (Objekterkennung der 3D-Kamera)
- lidar (Hinderniserkennung)
- speech (Sprachsteuerung und -ausgabe)
- motor (Steuerung der Motoren)
- controller (Zentrale Kontrolleinheit, welche die anderen Subpackages steuert)

Jeder dieser Subpackages hat wiederum eine eigene ReadMe-Datei, welche die Details der einzelnen Subpackages und den Umgang mit der entsprechenden Hardware erklärt.
Jeder dieser Readme-Dateien ist wie folgt aufgebaut:
- Einleitung
- Verwendete Packages 
- Dateien
- Konfiguration
- Lessons Learned (Tipps und Tricks mit dem Umgang)
- Potenzielle Verbesserungen (Verbesserungsmöglichkeiten)

Im Ordner "documents" befinden sich die mitgelieferte Anleitung des TurtleBots. 

Weitere Inhalte dieser Readme:
- Projektziel und umgesetzte Funktionalitäten
- Architektur des Botty-Packages
- Installation des Botty-Packages
- Starten der Demos (Anleitung, wie man alle Funktionalitäten dieses Packages ausführt)
- Aufgetretene Probleme und Umgang mit dem TurtleBot (dieses Kapitel sollte vor Benutzung des TurtleBots gelesen werden)

## Projektziel und umgesetzte Funktionalitäten
Dieses Repository ist das Ergebnis des Studienprojektes "Raumerfassung und Sprachsteuerung für einen teilautonomen Roboter" im Wintersemester 2019/20 des Studiengangs Angewandte Informatik an der Hochschule Kaiserslautern, Standort Zweibrücken. Unter der Leitung von Prof. Adrian Müller wurder der TurtleBot2 von Markus Dauth, David Kostka, Felix Mayer und Raschied Slet programmiert. Ziel war es, den TurtleBot2 (Spitzname "Botty McTurtleFace", kurz "Botty") mittels Sprachbefehlen in einem Pick-And-Place-Szenario zu steuern.

Folgende Funktionalitäten wurden in diesem Projekt realisiert:
- Mittels dem Sprackerkennungs-Framework "PocketSphinx" kann Botty per Sprachsteuerung gesteuert werden. Botty versteht eine einfache Grammatik und kann mit einem Text-to-Speech-System (TTS) über Lautsprecher antworten. Siehe Subpackage "speech"
- Objekterkennung von mehreren vortrainierten Objekten. Siehe Subpackage "camera".
- In einem vordefinierten Koordinatensystem (im Folgenden "Grid" genannt) kann Botty an eine Position geschickt werden und an der Position ein vorgegebenes Objekt suchen, wobei er sich im Kreis dreht, bis er das gesucht Objekte erkannt hat oder sich um 360° gedreht hat. Die Logik für das Grid befindet sich im Subpackage "controller".
- Bei erfolgreicher Objekterkennung bewegt sich der Arm nach vorne. Die Steuerung des Arms erfolgt über das Subpackage "arm".
- Hindernisserkennung mittels Lidar. Siehe Subpackage "lidar".
- Beim Vorwährtsfahren ist Botty in der Lage Objekte zu umfahren. Die Logik hierfür und alle weiteren Befehle zum Fahren befindet sich im Subpackage "motor".

# Architektur
![Architektur](documents/architecture.jpg)
Das System ist Funktional in mehere Module aufgeteilt, welche den Subpackages entsprechen:  
- Arm
- Camera
- Controller
- Lidar
- Motor
- Speech  

Ein Modul beinhaltet Nodes, die zur erreichung der Funktionalität, miteinander durch Topics/Services kommunizieren.  
Außer Driver-Nodes können dank ROS auch Nodes auf andere Maschinen ausgelagert werden,   
solange sie sich im ROS-Network befinden. (z.B. zur Visualisierung, Debugging)  
Jedes dieser Module ist von einander gekapselt, die einzige Kommunikation findet per Schnittstelle durch den Controller statt.  
Somit kann die Arbeitsweise von ROS im "Blackboard-Prinzip" strukturierter werden, was die Komplexität reduziert.   

Der Controller ist die zentrale Kontrolleinheit des Systems.  
Dort werden Befehle verwaltet, geplant und auf die Module verteilt, es wird also die Zusammenarbeit der Module koordiniert.  
Mehr dazu im Subpackage Controller.  

## Hardware
Botty McTurtleFace t besitzt folgendene Hardware-Komponenten:
- [PhantomX Reactor Arm (mit Wrist)](https://www.roscomponents.com/en/robotic-arms/100-phantomx-reactor.html#/montaje_widowx-yes/reactor_wrist_rotate-yes)
- [Hokuyo URG-04LX-UG01 (Lidar)](https://www.roscomponents.com/en/lidar-laser-scanner/83-urg-04lx-ug01.html)
- [Orbbec Astra (3D-Kamera)](https://www.roscomponents.com/en/cameras/76-orbbec.html)
- NUC mit Ubuntu 16.04 LTS
- [Kobuki Base](https://www.roscomponents.com/en/mobile-robots/97-kobuki.html)
- Lautsprecher (mifa)
- Mikrofon (Docooler)

## Das ROS-Package "botty"
Dieses ROS-Package beinhaltet alle Source-Dateien für die Ausführung der oben beschriebenen Funktionalitäten und wurde für ROS Kinetic unter Ubuntu LTS 16.04 konzipiert.

Die Subpackages arm, lidar, motor, speech und controller wurden alle mit Python-Skripten realisiert, während das Subpackage camera C++ verwendet.

Die Subpackages arm, camera, lidar, motor und speech dienen zur Steuerung der entsprechenden Hardware. Jedes dieser Packages bietet ROS-Services zum Auslesen von Informationen oder zur Steuerung der jeweiligen Hardware an. Jedes dieser Subpackages kommuniziert mit den Hardware-Komponenten über ROS-Packages von Dritten.

# Installation
Im Folgenden werden alle notwendigen Schritte aufgelistet, um dieses ROS-Package (botty) mit allen erforderlichen Abhängigkeiten zu installieren.

Dieses Repository dient als eigenes ROS-Package. Zur Installation müssen mehrere Programme über apt-get installiert werden und zusätzliche Git-Repositories in den "src" Ordner des Catkin-Workspaces geklont werden.

## Vorbereitung
Vor der Installation sollte Folgendes ausgeführt werden:
```
rosdep update
sudo apt-get update
sudo apt-get dist-upgrade
sudo apt-get install ros-kinetic-catkin python-catkin-tools
```

Klonen des botty-Packages in den Ordner “src” des Workspace:
```
git clone https://github.com/MarkusDauth/botty
```

## PhantomX Reactor Arm
Dependencies installieren:
```
sudo apt-get install ros-kinetic-arbotix
sudo apt-get install ros-kinetic-dynamixel-controllers
```
Im Ordner “src” des Workspace dieses Repository klonen und der Installationsanleitung des Repositories für "Arbotix-M" folgen:
```
git clone https://github.com/RobotnikAutomation/phantomx_reactor_arm.git
```

## Lidar
Im Ordner “src” des Workspace dieses Repository downloaden bzw. folgenden Instruktionen in der gegebenen Reihenfolge folgen:
```
git clone https://github.com/ros-drivers/driver_common.git
cd ..
catkin_make
cd src
git clone https://github.com/ros-drivers/hokuyo_node.git
cd ..
catkin_make
```

Troubleshooting: 
Es ist wichtig driver_common VORHER zu kompilieren, 
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
## Orbbec Astra Camera

Dependencies installieren:
```
sudo apt-get install ros-kinetic-find-object-2d
sudo apt install ros-kinetic-rgbd-launch ros-kinetic-libuvc ros-kinetic-libuvc-camera ros-kinetic-libuvc-ros
sudo apt-get install ros-kinetic-robot-localization
cd ~/catkin_ws/src
git clone https://github.com/orbbec/ros_astra_camera
```
Rosbot_ekf installieren:

```
cd ~/catkin_ws
git clone https://github.com/husarion/rosbot_ekf.git
catkin_make
```

## Botty-Package
Vor der erstmaligen ausführen von Programmen muss das Botty-Package kompiliert werden. Folgenden Befehl im catkin_ws Ordner ausführen:
```
cd ~/catkin_ws
catkin config --extend /opt/ros/kinetic
catkin build (falls eine Fehlermeldung kommt, kann man diesen Befehl ignorieren)
catkin_make  
```

# Starten der Komponenten und Demos
Im Folgenden wird erläutert, wie man die Funktionalitäten des Botty-Packages ausführt. Für jede Hardware-Komponente müssen Befehle ausgeführt werden, um deren ROS-Nodes zu starten.
Um eine bessere Übersicht über den aktuellen Status der einzelnen Komponenten (und eventuelles Debugging) zu erhalten, wird empfohlen, dass die jeweiligen Skripte in einzelnen Konsolen (Terminals) ausgeführt werden.

## Starten der Botty-Komponenten
Um die Demos zu starten, müssen zuerst vorher die ROS-Nodes der einzelnen Hardware-Komponenten gestartet werden.

### Roscore
Für alle Programme muss vorher der Roscore in einer eigenen Konsole gestartet werden. Mit dem TurtleBot2 geht das unter:
```
roslaunch kobuki_node minimal.launch
```

### PhantomX Reactor Arm starten
Arm-Nodes in eigener Konsole starten:
```
roslaunch phantomx_reactor_arm_controller arbotix_phantomx_reactor_arm_wrist.launch
```

### Lidar

Zunächst muss der Hokuyo in einem eigenen Terminal gestartet werden:
```
rosrun hokuyo_node hokuyo_node
```
Danach der hokuyoInterpreter auch in seinem eigenem Terminal:
```
rosrun lidar hokuyoInterpreter.py
```

Troubleshooting: 
Falls der Hokuyo nicht gefunden wird, folgendes zuvor ausführen, damit der passende Port angesprochen wird
```
sudo chmod a+rw /dev/ttyACM0
```

### Motor
In einer eigenen Konsole folgende Befehl ausführen:
```
rosrun motor motorService.py
```

### Speech
Recognizer und Parser gleichzeitig starten:
```
roslaunch speech parser.launch
```

Oder den Recognizer und Parser einzeln starten:
```
roslaunch speech recognizer.launch
roslaunch speech parser_only.launch
```

Dazu benötigte Nodes 'send_audio.py' und 'soundplay_node.py' werden automatisch mit gestartet.

### Kamera
Die Kamera kann entweder direkt auf dem TrutleBot ausgeführt werden oder auf einen eigenen Rechner ausgelagert werden. Wenn die Kamera auf einem eigenen Rechner läuft, kann hierdurch Rechenleistung (und somit auch Storm) auf dem TurtleBot gespart werden.

Auf eigenem Rechner starten:

```
roslaunch camera camera.launch
```
Auf dem Turtlebot-Rechner starten:

```
roslaunch astra_launch astra.launch
```

### Controller
Nur eine Node muss gestartet werden.
```
rosrun controller control.py
```
Sobald alle anderen Komponenten gestartet sind, wird auch der Controller Initialisiert.
Dabei sollte Botty "I am Botty McTurtleFace" sagen.

### Hardware
Bei dem Bluetooth Speaker den An/Aus Knopf gedrückt halten.
Eventuell muss Turtlebot/Ubuntu wieder manuell über Bluetooth mit dem Lautsprecher verbunden werden.

## Demos
Wenn alle Komponenten gestartet sind, sollten folgende Demos möglich sein.    
### Sprachbefehle
Für einfacheres Testen kann der Befehl direkt als Text an den Parser gesendet werden.
Dazu per `rostopic pub` an `/botty/speech/grammar_data` den Text veröffentlichen.  

"go forward/left/right"  
Botty fährt 2 Meter vorwärts bzw. dreht sich um 90 Grad nach rechts/links  
Beim vorwärts fahren sollte Botty Objekte umgehen, die im Weg stehen.

"go to the docking station"  
Botty versucht auf die Docking-Station (Ladestation) zu fahren.  
Damit die docking routine ausgeführt werden kann muss im Controller in der Klasse `Nav` `enable_dock = True` gesetzt werden.
Außerdem muss `roslaunch kobuki_auto_docking minimal.launch` gestartet werden.

"grab the object"  
Botty "greift" was vor ihm liegt.  

### Controller Kommandos  
Manche Befehle können noch nicht per Sprachbefehl gestartet werden.
Mit `rostopic pub /botty/speech/commands` kann eine Command-Message an den Controller gesendet werden.  
Die Message muss gefüllt werden mit:  
1. der Action-ID (z.B. "1" für "go", siehe `controller/nodes/botty.py`),  
2. einem Objekt-Namen (z.B. "position"),   
3. je nach Action-ID eine Attribut-Liste (z.B. [2, 3] für die Koordinaten)  

{action: 1, name: "position", attr: [2, 3]}  
Botty fährt zu Position [2, 3] im Grid  

{action: 5, name: "flower", attr: [4, 2]}  
Botty fährt zur Position und dreht sich dort so lange, bis das Objekt "flower" gefunden wurde (oder eine 360° Drehung).  
Dabei gibt er Rückmeldung zum Fortschritt und Ergebnis.  

# Aufgetretene Probleme und Umgang mit dem TurtleBot
## Stromprobleme
Der TurtleBot lässt sich entweder direkt über das Ladegerät aufladen oder man schließt das Ladegerät an die Ladestation an und positioniert den TurtleBot darauf. Die Positionierung des TurtleBots auf dem Ladegerät muss genau zentral sein, da er sonst nicht genug Strom über die Kontakte der Ladestation erhält (am Besten per Hand zentriert auf der Ladestation zurechtrücken). Aus diesem Grund ist es empfehlenswert, den TurtleBot direkt über das Ladekabel zu laden.

Man kann leider nicht die aktuelle Ladung der Akkus auslesen, weswegen der TurtleBot nach gebraucht geladen werden sollte. Haben die Akkus keinen Saft mehr, schaltet sich das NUC komplett aus. Sollte dies geschehen, sollte vor einschalten des NUCs der TurtleBot für 2 bis 3 Minuten geladen werden, da es sonst möglich ist, dass der TurtleBot sich direkt wieder ausschaltet.

Da die Akkus viele Ladezyklen während des Projektes durchgemacht haben, könnte es eventuell sein, dass neue Akkus bestellt werden müssen.

Ebenfalls sollte auch auf den korrekten Gebrauch des USB-Hubs geachtet werden (siehe nächstes Kapitel).

## USB-Anschlüsse
Der Onboard-Rechner des TurtleBots (NUC) hat nur 4 USB-Ports. Greifarm, Lidar und 3D-Kamera belegen bereits 3 davon. Wenn auf dem NUC direkt mit Maus und Tastatur arbeiten will, muss man entweder eines der anderen Geräte abtrennen oder einen USB-Hub verwenden. Lautsprecher und Kopfhörer benötigen auch jeweils einen USB-Anschluss. Wenn der USB-Hub benutzt wird, sollte darauf geachtet werden, dass die großen Stromverbraucher(Greifarm, Lidar, 3D-Kamera) direkt am NUC und nicht über den USB-Hub angeschlossen sind, da es sonst Probleme mit der Stromversorgung geben kann und der Akku sich sehr schnell entlädt.

## IP-Adresse
Um per SSH auf dem TurtleBot zu arbeiten, hat dieser eine feste IP-Adresse im Hochschul-Netz zugewiesen bekommen. Die feste IP-Adresse lautet: 10.0.189.60
 
## Mikrofon und Lautsprecher
Es wurden zusätzlich ein Lautsprecher und ein Mikrofon für den TurtleBot bestellt. Jedoch war es nicht möglich, dieser per Buchse am NUC zu benutzen, weswegen sie auch per USB angeschlossen werden müssen.

## Lieferung
Bei der Lieferung des TurtleBots sind die Stangen für die Halterungsplattformen beschädigt worden, weswegen neue Stangen und Schrauben nachgeliefert wurden. Die obere Plattform ist aus diesem Grund nicht zu 100% befästigt.

Außerdem ist während dem Projektes der Arduino des Greifarms ausgefallen, weswegen hierfür auch ein Ersatz nachgeliefert wurde.
 
## Umbau der Kamera
Um eine bessere Objekterkennung zu ermöglichen, wurde die Halterung für die 3D-Kamera an der Vorderseite des TurtleBot montiert (ursrpünglich befindet sich diese am hinteren Teil). Dadurch sind die Stangen und Halterungsplattformen nicht mehr im Bild der Kamera.

## Entwicklungsumgebung
Für die Entwicklung des TurtleBots eignet sich am besten ein native Ubuntu 16.04 LTS Umgebung, da ROS nur auf Linux läuft. Alternativ ist auf einem Rechner Dual-Boot mit Windows und Linux zu empfehlen.
Während des Projektes wurde eine portable Lösung mittels einer Ubuntu-SSD getestet, was aber auf den meisten Laptops nicht funktionierte. Virtuelle Umgebungen (sowohl auf dem eigenen Laptop, wie auch auf einem Remote-Server) eignen sich nicht für die Entwicklung mit ROS, da es hier zu sehr großen Problemen mit den Grafikttreibern kommt und die Performanz nicht ausreicht.

## Virtuelle Umgebung Gazebo
Ursprünglich war es angedacht, dass Programmcode zunächst virtuell mittels Gazebo getestet werden soll, bevor es auf dem echten TurtleBot ausgeführt wird. Jedoch ist es schwer die virtuelle Umgebung in Gazebo zum Laufen zu bringen und diese Umgebung mit den entsprechenden virtuellen Hardwarekomponenten zu konfigurieren. Es empfiehlt sich also, direkt mit dem TurtleBot der echten Welt zu arbeiten.

# Autoren
Markus Dauth, David Kostka, Felix Mayer und Raschied Slet

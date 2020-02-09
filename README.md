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

Folgende Demos gibt es und sollten unter der Überschrift (Demos) gemacht werden:
- AN Koordinat fahren
- finde ObjektX an Koord X,Y
- Finde Objekt (Was siehst du)

Grundstruktur der Sub-Readmes, sprich Überschriten (Template):
* Einleitung (Was soll wie gemacht werden? kurz die Funktionsweise erklären, Hardware kurz eklären, Links für Hardware)
* Verwendete Packages (Auflisten, Links, erklären und benutzt Tutorials verlinken)
* Dateien (Alle selbst geschriebenen Dateien erklären, aber nicht auf Code eingehen)
* Konfiguration (z.B. neue Bilder für die Kaemera trainieren, neue Posen für den Arm)
* Lessons Learned (Expereminte, die nicht funktioniert haben)
* Potenzielle Verbesserungen (hier erklären, was nachfolger noch machen könnte oder was besser sein kann)

Liste an Potenziellen Verbesserungen:
* Winkel beim Drehen besser
* Weichere Bewegungsansteuerung
* Umgehung von Hinternissen, (A*) ist nicht implementiert im MOtor
* Bessere Kamera Erkennung
* Neue Befehle z.B. “Bewege dich auf Kachel 2,2"


Was berücksicht werden sollte:
- Kein Glossar oder Literaturverzeichnis
- Trello aufräumen
- Nicht erklären was der Roboter ist, sondern nur was gemacht wurde. Entsprechende Webseiten verlinken
- Trello aufräumen

Ab hier beginnt unsere Doku:

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

Das Subpackage controller dient als zentrale Kontrolleinheit. Mittels der ROS-Services der anderen Packages werden Informationen der Hardware ausgelesen und darauf entsprechend reagiert. Die einzelnen Subpackages kommunizieren hierbei nicht miteiander, sondern jegliche Kommunikation und Logik findet über den Controller statt.

TODO David: bau hier das Schaubild über die Komponenten ein. Eventuell dieses Kapitel noch besser erklären???

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

## Botty-Package
Vor der erstmaligen ausführen von Programmen muss das Botty-Package kompiliert werden. Folgenden Befehl im catkin_ws Ordner ausführen:
```
cd ~/catkin_ws
catkin config --extend /opt/ros/kinetic
catkin build (falls eine Fehlermeldung kommt, kann man diesen Befehl ignorieren)
catkin_make  
```

# Starten der Demos
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
TODO Felix: muss hier eine oder zwei eigene Konsole gestartet werden?

Zunächst muss der Hokuyo gestartet werden:
```
rosrun hokuyo_node hokuyo_node
```
Danach der hokuyoInterpreter:
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
TODO David: wann führt man was aus?
Recognizer/Parser starten:
```
roslaunch speech parser.launch
```

Nur Recognizer starten:
```
roslaunch speech recognizer.launch
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

## Demos
TODO Raschied, David, Felix: hier kurz alle ausführbaren Codes eurer Packages auflisten (siehe Beispiel). (hier erklären, welche Demos ausgeführt werden können, z.B. "finde Blume an X,Y", Raster erklären)

Das Botty-Package bietet mehrere ausführbare Programme an. Für jedes dieser Programme, sollten die entsprechenden Komponenten aus dem vorherhigen Kapitel gestartet werden.


### Objekt in Grid suchen (Beispiel)
Botty fährt im Grid zur angegebenen Position und sucht ein Objekt. Findet er das Objekt, bewegt er den Arm in Richtung des Objektes
(TODO: weiter machen)
```
roslaunch controller arm_control.launch
```

### Arms
Das Speech-Modul wird beim launch automatisch gestartet, muss also nicht davor gestartet werden.

Navigation per Sprachkommandos:
```
roslaunch controller base_control.launch
```

# Aufgetretene Probleme und Umgang mit dem TurtleBot
## Stromprobleme
Der TurtleBot lässt sich entweder direkt über das Ladegerät aufladen oder man schließt das Ladegerät an die Ladestation an und positioniert den TurtleBot darauf. Die Positionierung des TurtleBots auf dem Ladegerät muss genau zentral sein, da er sonst nicht genug Strom über die Kontakte der Ladestation erhält (am Besten per Hand zentriert auf der Ladestation zurechtrücken). Aus diesem Grund ist es empfehlenswert, den TurtleBot direkt über das Ladekabel zu laden.

Man kann leider nicht die aktuelle Ladung der Akkus auslesen, weswegen der TurtleBot nach gebraucht geladen werden sollte. Haben die Akkus keinen Saft mehr, schaltet sich das NUC komplett aus. Sollte dies geschehen, sollte vor einschalten des NUCs der TurtleBot für 2 bis 3 Minuten geladen werden, da es sonst möglich ist, dass der TurtleBot sich direkt wieder ausschaltet.

Da die Akkus viele Ladezyklen während des Projektes durchgemacht haben, könnte es eventuell sein, dass neue Akkus bestellt werden müssen.

Ebenfalls sollte auch auf den korrekten Gebrauch des USB-Hubs geachtet werden (siehe nächstes Kapitel).

## USB-Anschlüsse
Der Onboard-Rechner des TurtleBots (NUC) hat nur 4 USB-Ports. Greifarm, Lidar und 3D-Kamera belegen bereits 3 davon. Wenn auf dem NUC direkt mit Maus und Tastatur arbeiten will, muss man entweder eines der anderen Geräte abtrennen oder einen USB-Hub verwenden. Lautsprecher und Kopfhörer benötigen auch jeweils einen USB-Anschluss. Wenn der USB-Hub benutzt wird, sollte darauf geachtet werden, dass die großen Stromverbraucher(Greifarm, Lidar, 3D-Kamera) direkt am NUC und nicht über den USB-Hub angeschlossen sind, da es sonst Probleme mit der Stromversorgung geben kann und der Akku sich sehr schnell entlädt.

## IP-Adresse
Um per SSH auf dem TurtleBot zu arbeiten, hat dieser eine feste IP-Adresse im Hochschul-Netz zugewiesen bekommen. Die feste IP-Adresse lautet: XXXXX (TODO: kennt einer die IP?)
 
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
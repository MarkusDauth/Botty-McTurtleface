# Einleitung

## Ziel

Dieses ROS-Package dient zur Steuerung des Lidar des Turtlebot.

Ziel war es eine Auswertung der Lidar Werte vorzunehmen, um die Umgebung auf Hindernisse überprüfen zu könne.

## Funktionsweise

Dort zu wird zunächst definiert, ab welcher Entfernung ein Objekt als Hindernis betrachtet wird. Zusätzlich werden alle erkannten Hindernisse in drei Richtungsgruppen sortiert - Links,Rechts und Vorne. Diese Ergebniswerte können entweder auf Anfrage (ROS-Service) oder durch ständige Benachrichtigung (ROS-Message) erhalten werden. Das Ergebnis besteht aus mehreren Listen. Für jede Richtungsgruppe gibt es eine Liste, gefüllt mit Gradzahlen, die die exakte Positionierung des Hindernis widerspiegelt, als auch eine Liste mit den Distanzen (in cm) zu den jeweiligen Hindernissen. Zu beachten ist, dass die Zugehörigkeit der Gradzahlen zu den Distanzen durch die identische Indexstelle gegeben ist. 

D.h. wenn an Stelle 7 der Gradzahl Liste für links ein Hindernis hinterlegt ist, ist die zugehörige Distanz ebenso an der Stelle 7 in der Distanz-Liste für links hinterlegt.

## Hardware

Das Lidar ist ein Hokuyo URG-04LX-UG01.
Lidar bedeutet es ist ein Laserscanner zur Berechnung von Distanzen auf einer horizontalen Ebene. Der Wahrnehmungswinkel beträgt 240° und die Wahrnehmungsreichweite bis ca. 4 meter.

Genauere Spezifikationen unter:

https://www.hokuyo-aut.jp/search/single.php?serial=166

## Verwendete Packages

Während das eigene Packgae eine Auswertung der Daten vornimmt, liefert das 'Hokuyo Node' package die benötigten Daten. Dazu ist ebenso das Package 'Driver Common' notwendig, da 'Hokuyo Node' allein nicht auf ROS-Kinetic lauffähig ist.

Genauere Infos unter:

https://github.com/ros-drivers/driver_common.git

http://wiki.ros.org/hokuyo_node

https://idorobotics.com/2018/11/02/integrating-the-hokuyo-urg-lidar-with-ros

# Dateien

## hokuyoInterpreter.py

Nimmt eine Auswertung der Daten des Lidars vor und erfüllt die oben beschriebenen Funktionsweisen.

Als aller erstes werden die externen Konfigurationsdaten geladen. Im Anschluss wird eine Kallibrations-Phases gestartet, da sonst die Halterungen und andere Bauteile des Turtlebots als Hindernisse wahrgenommen werden könnten. 
Anschließend nimmt es die gepublishten Daten des Lidars entgegen. Da diese nur eine Liste von Zahlen sind mit ungenauer Listenlänge, wird zunächst eine Umrechnung des Längenverhältnis vorgenommen, um zu ermitteln, welches Stellen der Liste, welchen Gradzahlen entsprechen. Die Voraussetzung hierfür ist das die Messungen in gleichmäßigen Abständen stattfinden. Ist die Zuordnung zu Gradzahlen erfolgt, wird anschließend auch eine Zuordnung in Links, Rechts und Vorne vorgenommen. Sobald das getan ist wird geprüft welche all der Werte unter den Grenzwert fallen, der den minimalen Sicherheitsabstand darstellt. Alle diese Werte, für die das zutrifft, werden mit Distanz, Gradrichtung und Richtungsgruppe hinterlegt. Nachdem alle Daten überprüft wurden, werden die hinterlegten Daten, die als Hindernis interpretiert werden, veröffentlicht. Nebenbei wird das letzte Ergebnis auch gemerkt und kann auf Befehl nachgefragt werden.

## config.xml

Beinhaltet die Konfigurations Daten, die für hokuyoInterpreter benötigt werden.

## msg/Hints.msg und srv/HintsService.srv

Stellen die für ROS notwendigen Message bzw. Service Definitions dar.

# Konfiguration

Jegliche Konfigurationen werden in der namens gebenden 'config.xml' vorgenommen. Diese können direkt hier geändert werden ohne dass der Code selbst geändert werden muss.

# Lessons Learned

Wird ein Hindernis erkannt, ist zunächst unklar in welcher Gradrichtung es sich überhaupt befindet, weshalb das zuerst berechnet werden muss. Die Praxis hat gezeigt, dass die Berechnung der entsprechenden Gradzahl nicht zu einfach ist, da der Hokuyo mehrere Messwerte für allein eine Gradzahl anfertigt. D.h. auch wenn der Hokuyo einen Messbereich von 240° hat, werden nicht 240 Ergebnisse geliefert (im Test waren es 512). Der Algorithmus wurde soweit angepasst, dass er unabhängig zu den insgesamt gemessenen Werten eine Zuordnung zu Gradzahlen vornehmen kann, vorausgesetzt die Messungen finden in gleichmäßigen Abständen fest. 

Des weiteren sollte unbedingt bei Start darauf geachtet werden, dass nicht mehrere Kabel vom Turtlebot die Sicht des Lidar versperren, weil allein die Kallibrierungs-Phase, dann nahezu alles als Totenwinkel erklären wird.

# Potenzielle Verbesserungen

Von Interesse wäre eine Kartenerstellung mit dem Lidar. Dies könnte auch unabhängig von dem derzeitigen Code erfolgen. ROS stellt hierfür bereits diverse Tools zur Verfügung, aber bei Tests hat sich gezeigt, dass viele dieser Tools nicht mit Turtlebot 2 kompatibel sind. Wenn keine passende Alternative gefunden werden würde, müsste die Kartenerstellung komplett manuell gemacht werden. Insofern das keine zu große Aufgabe für ein Studienprojekt darstellt, könnten als Anfang Änderungen am ‚hokuyoInterpreter‘ vorgenommen werden, damit dieser zusätzlich zu der derzeitigen Hindernis Erkennung ungeachtet der Erkennung eines Hindernis alle Messwerte hinterlegt. Schließlich kann er bereits eine Zuordnung der Messwerten zu Gradzahlen zusammen mit der jeweiligen Entfernung vornehmen.

# Sonstiges

Autoren, die über die Zeit hinweg an diesem Package gearbeitet haben (als Nachfolger hier einfach mit eintragen):
Felix Mayer 

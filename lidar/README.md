# Einleitung
## Ziel
Dieses ROS-Package dient zur Steuerung des Lidar [Hokuyo URG-04LX-UG01](https://www.roscomponents.com/en/lidar-laser-scanner/83-urg-04lx-ug01.html). Es wird eine Auswertung der Lidar-Werte gemacht, um die Umgebung auf Hindernisse zu überprüfen.

## Hardware
Das Lidar ist ein Hokuyo URG-04LX-UG01. Es handelt sich hierbei um einen Laserscanner zur Berechnung von Distanzen auf einer horizontalen Ebene. Der Wahrnehmungswinkel beträgt 240° und die Wahrnehmungsreichweite ist bis zu ca. 4 Meter.

Genauere Spezifikationen unter:
https://www.hokuyo-aut.jp/search/single.php?serial=166

## Funktionsweise
Alle Objekte, die sich in einem vordefinierten Radius des Lidar befinden, werden als Hindernisse erkannt (TODO: was genau ist der Abstand?). Alle erkannten Hindernisse werden in drei Richtungsgruppen eingeordnet - Links, Rechts und Vorne. Ergebniswerte, welche aus mehreren Listen bestehen, können entweder auf Anfrage (ROS-Service) oder durch ständige Benachrichtigung (ROS-Message) erhalten werden. Für jede Richtungsgruppe gibt es zwei Listen: Eine Liste gefüllt mit Gradzahlen, die die exakte Positionierung des Hindernisses darstellen und eine Liste mit den Distanzen (in cm) zu den jeweiligen Hindernissen. Zu beachten ist, dass die Zugehörigkeit der Gradzahlen zu den Distanzen durch die identische Indexstelle gegeben ist. Wenn beispielsweise an Stelle 7 der Gradzahl-Liste für links ein Hindernis hinterlegt ist, ist die zugehörige Distanz ebenso an dem 7. Index in der Distanz-Liste für links hinterlegt.


## Verwendete Packages
Daten werden vom Lidar selbst über das 'Hokuyo Node' package ausgelesen und dann in diesem Botty-Package (lidar) weiter verarbeitet. Dazu ist ebenso das Package 'Driver Common' notwendig, da 'Hokuyo Node' allein nicht auf ROS-Kinetic lauffähig ist.

Genauere Infos unter:
https://github.com/ros-drivers/driver_common.git
http://wiki.ros.org/hokuyo_node
https://idorobotics.com/2018/11/02/integrating-the-hokuyo-urg-lidar-with-ros

# Dateien
## hokuyoInterpreter.py
(TODO ich würde hier noch angeben, mit welcher Frequenz das Lidar liest und die Werte gepublisht werden. Bei manchen Beschreibungen bin ich mir nicht sicher ob das zu 100% richtig ist, aber generell solltest du hier mehr konkrete Zahlen erwähnen.)
Nimmt eine Auswertung der Daten des Lidars vor und erfüllt die oben beschriebenen Funktionsweisen. 

Beim Start werden die externen Konfigurationsdaten geladen. Danach findet eine Kalibrierungsphase statt, wodurch die Halterungen und andere Bauteile des Turtlebots erkannt und gespeichert werden, damit diese im weitern Verlauf ignoriert werden und somit nicht als Hindernisse wahrgenommen werden. Alles was während der Kalibrierungsphase in einer vorgegeben Reichweite ist (TODO hier die Reichweite angeben), wird im Folgenden ignoriert.
Nach der Kalibrierungsphase werden die gepublishten Daten des Lidars entgegengenommen. Da diese nur eine Liste von Zahlen sind mit ungenauer Listenlänge, wird zunächst eine Umrechnung des Längenverhältnis vorgenommen, um zu ermitteln, welche Indizes der Liste welchen Gradzahlen entsprechen. Die Voraussetzung hierfür ist, dass die Messungen in gleichmäßigen Abständen stattfinden (TODO: meinst du in zeitlichen Abständen?). Nach der Zuordnung der Gradzahlen, wird anschließend eine Aufteilung in die Richtungsgruppen Links, Rechts und Vorne vorgenommen. Anschließend wird geprüft welche der Abstandswerte unter einen vordefinierten Grenzwert, welcher den minimalen Sicherheitsabstand entspricht, fallen. Alle Abstandswerte, die unter diese Grenze fallen, werden mit Distanz, Gradrichtung und Richtungsgruppe hinterlegt (TODO Was bedeutet hinterlegt? Wo hinterlegt?) und anschließend als ROS-Message veröffentlicht. 

Das Ergebnis des Lidarzykluses kann per ROS-Service abgefragt werden

## config.xml
Beinhaltet die Konfigurationsdaten, die für hokuyoInterpreter.py benötigt werden.

## msg/Hints.msg und srv/HintsService.srv
Stellen die für ROS notwendigen Message- bzw. Service-Definitions dar.

# Konfiguration
Jegliche Einstellungsmöglichkeiten werde in "config.xml" vorgenommen. Änderungen hier benötigen keine weiteren Code-Anpassungen.

# Lessons Learned
Wird ein Hindernis erkannt, ist zunächst unklar, in welcher Gradrichtung es sich befindet, weshalb zuerst die Grachrichtung berechnet werden muss. Die Praxis hat gezeigt, dass die Berechnung der entsprechenden Gradzahl nicht einfach ist, da das Lidar mehrere Messwerte für eine Gradzahl veröffentlich. Auch wenn das Lidar einen Messbereich von 240° hat, werden nicht 240 Ergebnisse geliefert (im Test waren es 512). Der Algorithmus wurde soweit angepasst, dass er unabhängig zu den gemessenen Werten eine Zuordnung zu Gradzahlen vornehmen kann, vorausgesetzt die Messungen finden in gleichmäßigen Abständen statt (TODO auch hier wieder zeitliche Abstände?). 

Des weiteren sollte unbedingt beim Start des TurtleBots darauf geachtet werden, dass nicht mehrere Kabel vom Turtlebot die Sicht des Lidar versperren, da sonst während der Kalibrierungsphase sehr viele tote Winkel entstehen.

# Potenzielle Verbesserungen
Von Interesse wäre eine Kartenerstellung mit dem Lidar. Dies könnte auch unabhängig von dem derzeitigen Code erfolgen. ROS stellt hierfür bereits diverse Tools zur Verfügung, aber bei unseren Tests hat sich gezeigt, dass viele dieser Tools nicht ohne großen Aufwand mit dem Turtlebot2 kompatibel sind. Wenn keine passende alternativen Tools gefunden werden, müsste die Kartenerstellung komplett manuell gemacht werden.

(TODO: ich hab keine Ahnung was du mit den nächsten zwei Sätzen sagen willst. In so einer Dokumentation würde ich auch nicht von einem Studienprojekt reden. Kann man die beiden Sätze einfach weglassen?)
Insofern das keine zu große Aufgabe für ein Studienprojekt darstellt, könnten als Anfang Änderungen am ‚hokuyoInterpreter‘ vorgenommen werden, damit dieser zusätzlich zu der derzeitigen Hindernis Erkennung ungeachtet der Erkennung eines Hindernis alle Messwerte hinterlegt. Schließlich kann er bereits eine Zuordnung der Messwerten zu Gradzahlen zusammen mit der jeweiligen Entfernung vornehmen. 

# Autoren
Felix Mayer - Ursprüngliche Arbeiten

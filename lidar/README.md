# Einleitung
## Ziel
Dieses ROS-Package dient zur Steuerung des Lidar [Hokuyo URG-04LX-UG01](https://www.roscomponents.com/en/lidar-laser-scanner/83-urg-04lx-ug01.html). Es wird eine Auswertung der Lidar-Werte gemacht, um die Umgebung auf Hindernisse zu überprüfen.

## Hardware
Das Lidar ist ein Hokuyo URG-04LX-UG01. Es handelt sich hierbei um einen Laserscanner zur Berechnung von Distanzen auf einer horizontalen Ebene. Der Wahrnehmungswinkel beträgt 240° und die Wahrnehmungsreichweite ist bis zu ca. 4 Meter.

Genauere Spezifikationen unter:
https://www.hokuyo-aut.jp/search/single.php?serial=166

## Funktionsweise
Alle Objekte, die sich in einem vordefinierten Radius des Lidar befinden, werden als Hindernisse erkannt. Der genaue Abstand ist in den Konfigurationen beschrieben. Alle erkannten Hindernisse werden in drei Richtungsgruppen eingeordnet - Links, Rechts und Vorne. Ergebniswerte, welche aus mehreren Listen bestehen, können entweder auf Anfrage (ROS-Service) oder durch ständige Benachrichtigung (ROS-Message) erhalten werden. Für jede Richtungsgruppe gibt es zwei Listen: Eine Liste gefüllt mit Gradzahlen, die die exakte Positionierung des Hindernisses darstellen und eine Liste mit den Distanzen (in cm) zu den jeweiligen Hindernissen. Zu beachten ist, dass die Zugehörigkeit der Gradzahlen zu den Distanzen durch die identische Indexstelle gegeben ist. Wenn beispielsweise an Stelle 7 der Gradzahl-Liste für links ein Hindernis hinterlegt ist, ist die zugehörige Distanz ebenso an dem 7. Index in der Distanz-Liste für links hinterlegt.

## Verwendete Packages
Daten werden vom Lidar selbst über das 'Hokuyo Node' package ausgelesen und dann in diesem Botty-Package (lidar) weiter verarbeitet. Dazu ist ebenso das Package 'Driver Common' notwendig, da 'Hokuyo Node' allein nicht auf ROS-Kinetic lauffähig ist.

Genauere Infos unter:

https://github.com/ros-drivers/driver_common.git

http://wiki.ros.org/hokuyo_node

https://idorobotics.com/2018/11/02/integrating-the-hokuyo-urg-lidar-with-ros

# Dateien
## hokuyoInterpreter.py
Nimmt eine Auswertung der Daten des Lidars vor und erfüllt die oben beschriebenen Funktionsweisen. 

Beim Start werden die externen Konfigurationsdaten geladen. Danach findet eine Kalibrierungsphase statt, wodurch die Halterungen und andere Bauteile des Turtlebots erkannt und gespeichert werden, damit diese im weitern Verlauf ignoriert werden und somit nicht als Hindernisse wahrgenommen werden. Alles was während der Kalibrierungsphase in einer vorgegeben Reichweite ist, wird im Folgenden ignoriert. Standardmäßig beträgt die Scannreichweite für die Kallibrierung 25 cm.
Nach der Kalibrierungsphase werden die gepublishten Daten des Lidars entgegengenommen. Die Rohdaten, die andauernd von der 'hokuyo node' veröffentlicht werden, werden direkt verarbeitet, sobald diese dem 'hokuyoInterpreter' bekannt werden. Da diese nur eine Liste von Zahlen sind mit ungenauer Listenlänge, wird zunächst eine Umrechnung des Längenverhältnis vorgenommen, um zu ermitteln, welche Indizes der Liste welchen Gradzahlen entsprechen. Die Voraussetzung hierfür ist, dass die Messungen in gleichmäßigen Abständen stattfinden. Nach der Zuordnung der Gradzahlen, wird anschließend eine Aufteilung in die Richtungsgruppen Links, Rechts und Vorne vorgenommen. Anschließend wird geprüft welche der Abstandswerte unter einen vordefinierten Grenzwert, welcher den minimalen Sicherheitsabstand entspricht, fallen. Alle Abstandswerte, die unter diese Grenze fallen, werden mit Distanz, Gradrichtung und Richtungsgruppe anschließend als ROS-Message veröffentlicht. Der Grenzwert selbst ist nicht konstant und ist von den Konfigurationswerten abhängig. Standradmäßig beträgt der Grenzwert mindestens 5 cm und maximal 35 cm. Genaueres in den Konfigurationen.

Das Ergebnis des Lidarzykluses kann per ROS-Service abgefragt werden oder über einen ROS-Message Dienst kontinuierlich überprüft werden.

## config.xml
Beinhaltet die Konfigurationsdaten, die für hokuyoInterpreter.py benötigt werden.

## msg/Hints.msg und srv/HintsService.srv
Stellen die für ROS notwendigen Message- bzw. Service-Definitions dar.

# Konfiguration
Jegliche Einstellungsmöglichkeiten werde in "config.xml" vorgenommen. Änderungen hier benötigen keine weiteren Code-Anpassungen.

Prinzipell gibt es drei Richtungsgruppen (Links, Rechst, Vorne), die durch Start- und Endpunkt definiert sind, wobei die Punkte Gradzahlen sind, die in dem Wahrnehmungbereich des Lidar (240°) liegen. Jede der drei Richtungsgruppen hat auch eine definierte Seitengröße. An den Seiten einer Richtungsgruppe reduziert sich dann der Grenzwert immer weiter, desto näher die Messung am Start- und Endpunkt der Richtungsgruppe ist. Ist die Seitengröße in den Konfigurationen auf 0 gesetzt hat sie keine Auswirkung und der Grenzwert ist konstant. Ist sie größer 0 wird nur die Länge reduziert, die sich aus dem Konfigurationwert 'collision' ergibt. Der Wert 'constantSecureRange' wird nicht verringert. Der eigenzliche Grenzwert ist die Summe aus 'collision' und 'constantSecureRange'.

Die derzeitigen im Projekt vorgenommenen Konfigurationen lauten:

Rechts:			start=1 end=60 site=0

Links:			start=180 end=240 site=0

Vorne:			start=30 end=210 site=45

collision:		range=30

callibration:		range=0.25

constantSecureRange: 	range=5

# Lessons Learned
Wird ein Hindernis erkannt, ist zunächst unklar, in welcher Gradrichtung es sich befindet, weshalb zuerst die Grachrichtung berechnet werden muss. Die Praxis hat gezeigt, dass die Berechnung der entsprechenden Gradzahl nicht einfach ist, da das Lidar mehrere Messwerte für eine Gradzahl veröffentlich. Auch wenn das Lidar einen Messbereich von 240° hat, werden nicht 240 Ergebnisse geliefert (im Test waren es 512). Der Algorithmus wurde soweit angepasst, dass er unabhängig zu den gemessenen Werten eine Zuordnung zu Gradzahlen vornehmen kann, vorausgesetzt die Messungen finden in gleichmäßigen Abständen statt. 

Des weiteren sollte unbedingt beim Start des TurtleBots darauf geachtet werden, dass nicht mehrere Kabel vom Turtlebot die Sicht des Lidar versperren, da sonst während der Kalibrierungsphase sehr viele tote Winkel entstehen.

# Potenzielle Verbesserungen
Von Interesse wäre eine Kartenerstellung mit dem Lidar. Dies könnte auch unabhängig von dem derzeitigen Code erfolgen. ROS stellt hierfür bereits diverse Tools zur Verfügung, aber bei unseren Tests hat sich gezeigt, dass viele dieser Tools nicht ohne großen Aufwand mit dem Turtlebot2 kompatibel sind. Wenn keine passende alternativen Tools gefunden werden, müsste die Kartenerstellung komplett manuell gemacht werden. 

# Autoren
Felix Mayer - Ursprüngliche Arbeiten

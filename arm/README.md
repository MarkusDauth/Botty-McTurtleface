# Einleitung
Dieses ROS-Package dient zur Steuerung des [PhantomX Reactor Arm](https://www.roscomponents.com/en/robotic-arms/100-phantomx-reactor.html#/montaje_widowx-yes/reactor_wrist_rotate-yes). Ziel war es, dass der Arm Objekte, die von der 3D-Kamera erkannt werden, gegriffen werden und diese Objekte wieder an anderer Stelle abgelegt werden. Jedoch traten während des Projektes mehrere schwerwiegende Probleme mit dem Greifarm auf, weswegen der Arm nur fest programmierte Posen annehmen kann (siehe Lessons Learned).

# Hardware
Der PhantomX Reactor Arm ist ein ROS-kompatibler Greifarm, welcher bereits aufgebaut mit dem TurtleBot2 geliefert wurde. Der Greifarm wird über einen Arbotix-M gesteuert, welches ein modifiziertes Arduino-Board zur Ansteuerung von Servo-Motoren ist. Auf dem NUC des TurtleBot kann eine ROS-Node gestartet werden, welche Positionierungsbefehle für den Greifarm annimmt und diese Befehle dann an das Arduino-Board weiterleitet. Hierfür befindet sich ein Arduino-Image auf dem Arbotix-M, welches die ROS-Befehle interpretiert und entsprechend die Servo-Motoren ansteuert.

In diesem Projekt wurde der PhantomX Reactor Arm "mit Wrist" verwendet.

Achtung vor Benutzung:
- Der Greifarm benötigt relativ viel Strom, weswegen das USB-Kabel direkt an den Onboard-Computer des TurtleBots angeschlossen werden muss (nicht über den USB-Hub anschließen)
- Wenn der Arm direkt über die ROS-Schnittstellen positioniert werden soll, muss dies mit großer Vorsicht gemacht werden. Es können Befehle gesendet werden, die zu große Winkel für die Gelenke verursachen und somit sich die Motoren verhaken oder sich Kabel lösen.
- Je nach vorgesehener Armposition, kann sich der Arm an der Platform des TurtleBots oder sogar mit eigenen Armgelenken verhaken oder kollidieren. Es ist wichtig zu überprüfen, in welcher Reihenfolge die einzelnen Gelenke des Arm angesteuert werden, um diese Probleme zu vermeiden. 

# Funktionsweise
Es wird der ROS-Service "/botty/arm/commands" zur Verfügung gestellt, wodurch der Arm zwei vordefinierte Bewegungen durchführen kann:
- Bewegung "home": Der Arm begiebt sich in seine "Ruheposition" und bleibt in dieser.
- Bewegung "push": Der Arm bewegt sich nach vorne und führt eine "Wischbewegung" von Links nach Rechts durch.

# Verwendete Packages
Die Positionierung des Arms erfolgt über das ROS-Package "phantomx_reactor_arm":
https://github.com/RobotnikAutomation/phantomx_reactor_arm

Mit diesem Package ist es möglich, die Gradwinkel für die Gelenke des Arm direkt anzugeben (andere Möglichkeiten zur Ansteuerung gibt hiermit leider nicht). In dem Github-Repository befindet sich außerdem eine Installationsanleitung und es wird gezeigt, wie man den Arm steuert.

Achtung:
Mit diesem Package werden die Positionierungen der einzelnen Gelenke direkt vorgenommen. Es ist möglich, die Gelenke des Arm zu übersteuern oder Kollisionen mit eigenen Armteilen zu verursachen. Generell sollte vor Testen von Code überlegt werden, welche Armgelenke sich zuerst bewegen müssen, damit keine Kollisionen stattfinden. Um Übersteuerungen zu verhindern, sollten nur Werte für die einzelnen Gelenke zwischen -1,5 und +1,5 verwendet werden.

Troubleshooting:
Falls die Ansteuerung mit diesem Package nicht funktioniert, ist es empfehlenswert, die komplette Installationsnaleitung des "phantomx_reactor_arm"-Repositories für Arobitx-M nochmal durchzuführen.

# Dateien
## nodes/arm_control.py
Diese Datei stellt die Node mit den oben genannten Funktionen zur Verfügung. Über das ROS-Package "phantomx_reactor_arm" wird der Arm angesteuert.
Wenn der Arm in einer Bewegung mehrere Positionen ansteuern soll, müssen die sleep-Aufrufe benutzt werden, um Kollisionen zu vermeiden.

## named_pose.py
Diese Datei dient als Beispielvorlage für die potentielle Verwendung des MoveIt-Packages. Es müssen die entsprechenden Posen mit dem MoveIt-Setup-Assistant eingestellt werden. Der Arm lässt sich jedoch nicht mit dem MoveIt-Package steuern (Siehe Lessons Learned / MoveIt).

# Lessons Learned
Der Umgang mit dem PhantomX Reactor Arm gestaltet sich als sehr schwierig. Während des Projektes gab es zahlreiche Probleme, bei denen etliche Lösungen probiert wurden, jedoch war es zum Schluss des Projektes nur möglich, den Arm mittels hardgecodeten Posen per ROS zu steuern. Online-Dokumentation gibt es für den PhantomX Reactor nur wenige und alle verwendbaren ROS-Packages für den Arm sind sehr schlecht dokumentiert.

## Probleme mit der Installation und Arbotix-M
Der Arm konnte nach Anlieferung nicht direkt benutzt werden. Sollte der Arm nicht über das "phantomx_reactor_arm"-Package angesteuert werden können, so sollte am besten nochmal die komplette Installationsanleitung für den Arm vorgenommen werden. Siehe:
https://github.com/RobotnikAutomation/phantomx_reactor_arm

Es ist darauf zu achten, dass das richtige Arduino-Image auf den Arm gespielt wird und dass die richtigen [udev-Regeln](https://wiki.ubuntuusers.de/udev/) auf dem NUC eingestellt sind.

Sollte sich das Arduino-Image nicht auf den Arbotix-M installieren lassen, so könnte es sein, dass das Arduino-Board defekt ist. Dieses kann durch einen neuen Arbotix-M einfach ausgetauscht werden. Zum Testen, ob der Arbotix-M noch richtig funktioniert, dient folgende Anleitung des PhantomX-Herstellers:
https://learn.trossenrobotics.com/arbotix/7-arbotix-quick-start-guide

Das Arbotix-M-Board musste während des Projektes einmal ausgetauscht werden.

## MoveIt
Zur einfacheren Steuerung der Arms wurde probiert, das MoveIt-Package zu verwenden:
http://docs.ros.org/kinetic/api/moveit_tutorials/html/index.html

Das MoveIt-Package benötigt jedoch entsprechende ROS-Controller, die es für diesen Greifarm nicht gibt. Eine manuelle Anpassung des MoveIt-Packages gestaltete sich als zu komplex. Es könnte möglich sein, den Greifarm über MoveIt mittels des Adapters "USB2Dynamixel" zu steuern, jedoch ist dies so gut wie nich nicht dokumentiert.

## Open Source Programm
Es gibt ein Open-Source Java Programm, welche Funtkionen zur einfacheren Ansteuerung des Arms anbietet:
https://learn.trossenrobotics.com/36-demo-code/137-interbotix-arm-link-software.html

Es wurde versucht, Funktionalitäten der InterbotiX Arm Link Software in einem eigenen Java Konsolenprogramm zu verwenden. Die InterbotiX Arm Link Software ist jedoch sehr stark an das "Processing" Framework gebunden. Über das Processing Framework wird sowohl die GUI, wie auch die Steuerung des Arms realisiert. Es ist nicht möglich, Teile der Software ohne Processing zu verwenden, wodurch eine Verwendung dieser Software auf dem TurtleBot nicht als hilfreich zu erachten ist.

# Potenzielle Verbesserungen
Für den Arm könnten mehrere Posen in dem Skript nodes/arm_control.py konfiguriert werden. Außerdem könnte der Arm eventuell so programmiert werden, dass er sich je nach erkannten Objekt andere Posen annimmt.
Ein Greifen von Objekten ist jedoch mit dem aktuell verwendeten Package schwierig umzusetzen, da die Winkel aller Armgelenke manuell programmiert werden müssen.

Es könnte ein USB2Dynamixel gekauft werden, welcher eventuell eine Steuerung des PhantomX mit MoveIt ermöglichen. Dadurch könnte das tatsächliche Greifen von Objekten, die die 3D-Kamera erkennt, ermöglicht werden.

Eine Alternative für den PhantomX Reactor wäre der WidowX Arm:
https://learn.trossenrobotics.com/projects/186-widowx-arm-with-ros-getting-started-guide.html

Für diesen Arm gibt es mehr Dokumentationen und Hilfestellungen online. Er lässt sich außerdem mit dem MoveIt-Package von Haus aus steuern.
Siehe: https://github.com/Interbotix/widowx_arm

# Autoren
Markus Dauth - Ursprüngliche Arbeiten

David Kostka - Implementierung der Posen als ROS-Service

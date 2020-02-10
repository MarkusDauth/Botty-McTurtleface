# Einleitung
## Ziel
Dieses ROS-Package dient zur Steuerung des Motors des TurtleBots2. Ziel war es, dass der Motor von außerhalb aufgefordert werden kann bestimmte Aktionen auszuführen.

## Funktionsweise
Der Motor nutzt einen ROS-Service, durch den Botty diverse Befehle erhalten kann, die er ausführen soll. Botty kann sich eine gewünschte Gradzahl im Kreis drehen oder geradeaus fahren, während er dabei mögliche Hindernisse umfährt. Als Service wird nach jeder Aktion auch eine Rückmeldung gegeben bzw. einen Rückgabewert geliefert, der einerseits Aufschluss darüber gibt, welche Stelle Botty erreicht hat und ob seine Aktion erfolgreich ausgeführt werden konnte. Die Rückgabe der Positionierung erfolgt über X und Y Koordinaten in Metern.

## Verwendete Packages
Für den Motor wurden keine externen Packages verwendet.

# Dateien
## motorService.py
Startet den Motor Service und warte auf Kommandos, die dann ausgeführt werden.

Als ROS-Service wird nach Start des Skripts zuerst lediglich eine Initialisierung vorgenommen. Erst wenn von anderen Komponenten Aufforderungen an den Motor geschickt werden, werden diese verarbeitet und ausgeführt. Für gewöhnlich erhält der Motor die Aufforderungen vom Controller; er kann aber auch unabhängig genutzt werden. Ein Kommando muss in der Form einer ROS-Service-Definition erstellt werden, damit der Motor sie entgegennehmen kann. Sie besteht aus 2 Teilen: 'call': der Name der verwendeten Funktionalität und 'param': eine Floatliste mit Parametern, wie z.B für den Drehwinkel beim Drehen. FÜr jedes Feature wird die selbe Service-Definition benutzt, weshalb das erhaltene Kommando auf mögliche Fehler überprüft werden muss. Ein Beispiel wäre der Name eines Feetures, den es nicht gibt. Infolge dessen würde der Motor Service den Fehler melden: "Error: Call-name is unknown". Anzumerken ist, dass die Fehler aber nicht zum Programmabsturz führen, sondern die jeweiligen Kommandos dann schlicht nicht ausgeführt werden.

Die verschiedenen Funktionalitäten werden im Folgenden einzeln erklärt.

### setSpeed

Nimmt einen Parameter 'speed' entgegen und ändert die Geschwindigkeit entsprechend. Diese Geschwindigkeit gilt für alle zukünftigen Aktionen, solange sie nicht geändert wird. Standardmäßig beträgt sie 0.75.

Mögliche Fehlermeldungen:

"Call: setSpeed; Error: Wrong amount of Parameters! setSpeed takes 1 parameter =>speed"

### forwardByMeters

Nimmt zwei Parameter entgegen, wobei der zweite optional ist. Der erste Parameter 'meter' gibt die Distanz an, die der Turtlebot nach vorne fahren soll. Zunächst wird die Zeit berechnet, die er bei der gegebenen Geschwindigkeit benötigt, um die gewünschte Distanz zurück zu legen. Dann wird eine Bussy-Loop genutzt, um ihn immer weiter fahren zu lassen, bis er die gewünschte Zeit gefahren ist und damit auch sein Ziel erreicht hat. Sollte als zweiter Parameter eine 1 angegeben werden, die als Boolean interpretiert wird, wird Botty auf Hindernisse in seinem Weg achten und diese gegebenenfalls umfahren. Dabei geschieht Folgendes:

Wenn Botty ein Hindernis mittels dem Lidar vor sich sieht, haltet er an und schaut, ob er rechts vorbei fahren kann. Falls ja, fährt er rechts am Hindernis entlang, bis er das Hinderniss nicht mehr erkennt. Botty wird sich die Zeit merken, die er benötigt, um am Hindernis entlang zu fahren, damit er später weiß, wie lange er wieder zurück fahren muss, sobald er am Hindernis vorbei gefahren ist. Dann wird er in ursprünglicher Fahrtrichtung am Hindernis weiterfahren und sich zur zur ursprünglich geplante Route zurückdrehen. Die Zeit, die er sich zuvor gemerkt hat, fährt er nun zurück, in der Konsequenz, dass er hinter dem Hindernis steht auf exakt der Stelle, die er wegen des Hindernis nicht erreichen konnte. Danach wird er seinen anfänglichen Pfad weiter folgen. Sollte er aber ganz zu Beginn nicht nach rechts ausweichen können, da dort ebenso ein Hindernis steht, wird er stattdessen links ausweichen. Sollte auch das nicht gehen, da es sich um eine Sackgasse handelt, scheitert die gesamte Aktion mit der Warnung "No space". Sollte der die Umgehung von Objekten nicht gewollt sein, kann man auf den zweiten Parameter verzichten oder eine 0 übergeben. In folge dessen wird Botty blind nach vorne fahren.

Mögliche Fehlermeldungen:

"Call: forwardByMeter(distanz, hokuyo); Error: Second Argument only takes 1 or 0 to be converted to boolean!"

"Call: forwardByMeters; Error: Wrong amount of parameters! forwardByMeters takes 1 parameter and 1 optional =>meters,hokuyo=False (take 0 or 1 to represent the boolean)"

### turnRigthByAngle; turnLeftByAngle

Nimmt zwei Parameter entgegen, wobei der Zweite optional ist. Der erste Parameter 'angle' gibt die Gradzahl an, um die sich Botty drehen soll, während der zweite noch zusätzlich die Geschwindigkeit beim drehen anpassen kann. Zunächst wird die Zeit berechnet, die er bei der gegebenen Geschwindigkeit benötigt, um die gewünschte Drehung durchzuführen. Dann wird eine Bussy-Loop genutzt, um ihn immer weiter drehen zu lassen, bis er sich die gewünschte Zeit gedreht hat und damit auch seine Zielausrichtung erreicht hat. Je nachdem welche der beiden Drehungs-Funktionen genutzt wurde, wird er sich entweder im Uhrzeigersinn oder entgegen drehen.

Mögliche Fehlermeldungen:

"Call: turnRigth/LeftByAngle; Error: Wrong amount of Parameters! turnRigth/LeftByAngle takes 2 parameters; 1 is optional =>angle,speed=None"

### stopp

Nimmt keinen Parameter entgegen. Bei Aufruf wird jegliche derzeit ausgeführte Aktion abgebrochen. Diese Funktion dient dazu bei ungewolltem Verhalten die Motoren zu stoppen, bevor Botty Schaden nehmen könnte. Auch die Bussy-Loops der anderen Funktionalitäten werden dann abgebrochen. Diese Funktion kann manuell, vom Controller oder auch von ROS selbst aufgerufen werden. Sollte es zu einem Systemfehler kommt und das Programm beendet wird, wird ROS selbstständig diese Funktion als letztes ausführen, um sicherzustellen, dass nach Programmabsturz der Motor auch zum Stoppen konnmt, da sonst dieser immer weiter Gas geben würde.

Mögliche Fehlermeldungen:

"Call: stopp; Error: Wrong amount of Parameters! stopp takes 0 parameters"

Anmerkung: Dieser Fehler kann nicht eintreffen, wenn ROS selbst die Funktion startet.

## srv/call.srv
Stellen die für ROS notwendigen Service-Definitions dar.

# Konfiguration
Jede normale Anpassung seines Verhaltens, wie die Distanz, der Drehwinkel oder die Geschwindigkeit, werden über den Service zur Anwendungszeit über entsprechende Aufrufe angepasst. Detail Anpassungen, wie zum Beispiel die Exaktheit der Drehung, müssen im Code vorgenommen werden.

# Lessons Learned
Der Motor ist eine weitaus größere Herausforderung als man ahnt. Die große Schwierigkeit zeigt sich in der Feinjustierung seiner Aktionen. Parameterangaben zum Fahren, als auch zum Drehen, werden durch einen Wert von 0.0 bis 1.0 repräsentiert. Eine uns normal erscheinende Angabe in Metern oder Winkelgarde ist für ihn nativ nicht möglich. Die Frage war also , wie lange muss sich Botty mit einem Wert von 0.75 drehen bis ich mich um X Grad gedreht habe? Hierzu waren zahlreiche Tests notwendig bis ein Faktor gefunden wurde, der auf die gewünschte Gradzahl oder Distanz multipliziert wird, sodass das Istergebnis nach der Ausführung möglichst ähnlich dem Sollergebnis entspricht. Diese Faktoren sind im Code als Abweichungen hinterlegt und entsprechend kommentiert. Ein weiteres Problem ist, dass das Zurücklegen von zwei mal einem Meter und einmal zwei Meter unterschiedliche Ergebnisse bringt. Grund hierfür ist, dass beim Fahren von zweimal einem Meter auch mehrmals beschleunigt werden muss, was entsprechend Zeit kostet. Zusätzlich empfiehlt es sich bei Tests sich die Möglichkeit des Abbrechen bereit zu halten, da allein kleine Fehler große Auswirkungen in seinem Fahrverhalten haben können, wodurch Botty unberechenbar werden könnte und schnell gegen eine Wand fährt, wenn nicht aufgepasst wird. 

# Potenzielle Verbesserungen
Die derzeitigen Beschleunigungen und Bremsvorgänge sind allesamt sehr plötzlich, was dazu führt, dass wenn Botty bei größeren Geschwindigkeiten auf null runter bremst, er noch ein kleines Stück rutscht. In der Folge würde auch seine interne Verwaltung der Positionierung über die Zeit kleinere Abweichungen haben. Ein weitere Verbesserung wäre es, die Werte der Abweichung anzupassen. Diese wurden erstellt und getestet in einer Phase, als der PhantomX Reactor Arm wegen eines Hardware Schadens abmontiert war. Nun wo er wieder aufgesetzt ist bringt er einiges an Zusatzgewicht mit, wodurch Botty langsamer beschleunigt und fährt. Dadurch erreicht er nicht mehr die gewünschte Distanz beim fahren oder dreht sich zu kurz.

# Autoren
Felix Mayer - Ursprüngliche Arbeiten

# Einleitung
## Ziel
Dieses ROS-Package dient zur Steuerung des Motors des Turtlebots2. Ziel war es, dass der Motor von außerhalb aufgefordert werden kann bestimmte Aktionen auszuführen.

## Funktionsweise
Der Motor nutzt einen ROS-Service, durch den er diverse Befehle erhalten kann, die er ausführen soll. Nach Aufforderung kann er eine gewünschte Distanz sich drehen oder nach vorne fahren und gegebenfalls dabei ebenso auf mögliche Hindernisse achten. Als ein Service wird er auch nach jeder Aktion auch eine Rückmeldung geben bzw. einen Rückgabewert liefern, die einerseits Aufschluss darüber gibt, welche Stelle er erreicht hat und ob seine Aktion überhaupt erfolgreich ausgeführt werden konnte. Die Rückgabe der Positionierung erfolgt über X und Y Koordinaten in metern.

## Verwendete Packages
Für den Motor wurden keine externen Packages verwendet.

# Dateien
## motorService.py
Startet den Motor Service und warte auf Kommandos, die es dann auszuführen gilt.

Als ROS-Service wird nach Start des Scripts zunächst nichts weiteres passieren, außer dass der Service initialisiert wird. Erst sobald von anderen Komponenten Auforderung an den Motor geschickt werden, wird dieser die Aufforderung verarbeiten und ausführen. Für gewöhnlich erhält der Motor die Aufforderungen vom Controller, kann aber auch unabhängig genutzt werden. Ein Kommando muss in der Form einer ROS-Service Definition erstellt werden, damit der Motor sie entgegennehmen kann. Sie besteht aus 2 Teilen: 'call': der Name der verwendeten Funktionalität und 'param': eine Floatliste mit Parametern, wie z.B für den Drehwinkel beim Drehen. Angemerkt nutzt er für jedes Feeture die selbe Service Definition, weshalb er selbst das erhaltene Kommando auf mögliche Fehler überprüft. Ein Beispiel wäre der Name eines Feetures, den es nicht gibt. In Folge dessen würde der Motor Service den Fehler melden: "Error: Call-name is unknown". Anzumerken ist, dass die Fehler aber nicht zum Programmabsturz führen, sondern die jeweiligen Kommandos dann schlicht nicht ausgeführt werden.

Die verschiedenen Funktionalitäten werden im Folgenden einzeln erklärt.

### setSpeed

Nimmt einen Parameter 'speed' entgegen und ändert die Geschwindigkeit entsprechend. Diese Geschwindigkeit gilt für alle zukünftigen Aktionen, solange sie nicht geändert wird. Standartmäßig beträgt sie 0.75.

Mögliche Fehlermeldungen:

"Call: setSpeed; Error: Wrong amount of Parameters! setSpeed takes 1 parameter =>speed"

### forwardByMeters

Nimmt zwei Parameter entgegen, wobei der zweite optional ist. Der erste 'meter' gibt die Distanz an, die der Turtlebot nach vorne fahren soll. Zunäscht wird die Zeit berechnet, die er bei der gegebenen Geschwindigkeit benötigt, um die gewünschte Distanz zurück zu legen. Dann wird eine Bussy-Loop genutzt, um ihn immer weiter fahren zu lassen, bis er die gewünchte Zeit gefahren ist und damit auch sein Ziel erreicht hat. Das tut er im Normalfall. Sollte als zweite Paramterr eine 1 angegeben werden, die als Boolean interpretiert wird, wird Botty auf Hindernisse in seinem Weg achten und diesen gegebenfalls ausweichen. Dabei geht er folgendermaßen vor:

Wenn er ein Hindernis vor sich siehst, haltet er und schaut, ob er rechts vorbei kannt, falls ja, fährt rechts am Hindernis entlang solange das Hindernis im Weg ist. Botty wird sich die Zeit merken, die er benötigt, um am Hindernis entlang zu fahren, damit er später weiß wie lange er wieder zurück fahren muss, sobald er am Hindernis vorbei ist. Dann wird er in ursprünglicher Fahrtrichtung am Hindernis vorbeifahren und sich zur eigentlichen Planstrecken zurückdrehen. Die Zeit, die er sich zuvor gemerkt hat, fährt er nun zurück, in der Konsequenz, dass er hinter dem Hindernis steht auf exakt der Stelle, die er wegen des Hindernis nicht erreichen konnte. Danach wird er seinen anfänglichen Pfad weiter folgen. Sollte aber ganz zu Beginn er nicht nach rechts ausweichen können, da dort ebenso ein Hindernis steht, wird er statdessen links ausweichen. Sollte auch das nicht gehen, da es sich um eine Sackgasse handelt, scheitert die gesamte Aktion mit der Warnung "No space". Sollte der ganze Vorgang nicht gewollt sein, kann man auf den zweiten Parameter verzichten oder eine 0 übergeben. In folge dessen wird Botty blind nach vorne fahren.

Mögliche Fehlermeldungen:

"Call: forwardByMeter(distanz, hokuyo); Error: Second Argument only takes 1 or 0 to be converted to boolean!"

"Call: forwardByMeters; Error: Wrong amount of parameters! forwardByMeters takes 1 parameter and 1 optional =>meters,hokuyo=False (take 0 or 1 to represent the boolean)"

### turnRigthByAngle; turnLeftByAngle

Nimmt zwei Parameter entgegen, wobei der zweite optional ist. Der erste 'angle' gibt die Gradzahl an, um die sich Botty drehen soll, während der zweite noch zusätzlich die Geschwindigkeit beim drehen anpassen kann. Zunäscht wird die Zeit berechnet, die er bei der gegebenen Geschwindigkeit benötigt, um die gewünschte Drehung durchzuführen. Dann wird eine Bussy-Loop genutzt, um ihn immer weiter drhen zu lassen, bis er sich die gewünchte Zeit gedreht hat und damit auch seine Zielausrichtung erreicht hat. Je nachdem welche der beiden Drehungs Funktionen genutzt wurde, wird er sich entwerder im Uhrzeigersinn oder entgegen drehen.

Mögliche Fehlermeldungen:

"Call: turnRigth/LeftByAngle; Error: Wrong amount of Parameters! turnRigth/LeftByAngle takes 2 parameters; 1 is optional =>angle,speed=None"

### stopp

Nimmt keinen Parameter entgegen. Bei Aufruf wird jegliche derzeit ausgeführte Aktion abgebrochen. Diese Funktion dient dazu bei ungewolltem Verhalten die Motoren zu stoppen, bevor Botty Schaden nehmen könnte. Auch die Bussy-Loops der anderen Funktionalitäten werden dann abgebrochen. Diese Funktion kann manuell, vom Controller oder auch von ROS selbst aufgerufen werden. Sollte es zu einem Systemfehler kommen und das Programm beendet werden, wird ROS selbstständig diese Funktion als letztes ausführen, um sicherzustellen, dass nach Programmabsturz der Motor auch zum Stoppen konnmt, da sonst dieser immer weiter Gas geben würde.

Mögliche Fehlermeldungen:

"Call: stopp; Error: Wrong amount of Parameters! stopp takes 0 parameters"

Anmerkung: Dieser Fehler kann nicht eintreffen, wenn ROS selbst die Funktion startet.

## srv/call.srv
Stellen die für ROS notwendigen Service-Definitions dar.

# Konfiguration
Jede normale Anpassung seines Verhaltens, wie die Distanz,der Drehwinkel oder auch die Geschwindigkeit, werden auch über den Service zur Anwendungszeit angepasst über entsprechende Aufrufe. Detail Anpassungen wie zum Beispiel die Exaktheit der Drehung müssen im Code vorgenommen werden.

# Lessons Learned
Der Motor ist eine weit aus größere Herausforderung als man ahnt. Die große Schwierigkeit zeigt sich in der Feinjustierung seiner Aktionen. Der Wert zum Fahren, als auch zum Drehen, wird ducrh einen Wert von 0.0 bis 1.0 repräsentiert. Eine uns normal erscheinende Angabe in Metern oder Winkelgarde ist für ihn nativ nicht möglich. Die Frage war also , wie lange muss ich mich mit einem Wert von 0.75 drehen bis ich mich um X Grad gedreht habe? Hierzu waren zahlreiche Tests notwendig bis ein Faktor gefunden wurde, der auf die gewünschte Gradzahl oder Distanz multipliziert wird, sodass das Istergebnis nach der Ausführung möglicht ähnlich dem Sollergebnis entspricht. Diese Faktoren sind im Code als Abweichungen hinterlegt und entsprechend kommentiert. Ein weiteres Problem ist, dass das Zurücklegen von zwei mal einem Meter und einmal zwei Meter unterschiedliche Ergebnisse bringt. Grund hierfür ist, dass beim Fahren von zweimal einem Meter auch mehrmals beschleunigt werden muss, was entsprechend Zeit kostet. Zusätzlich empfhielt es sich bei Tests sich die Möglichkeit des Abbrechens bereit zu halten, da allein kleine Fehler große Auswirkungen in seinem Fahrverhalten haben können, wodurch Botty unberechenbar werden könnte und schnell gegen eine Wand fährt, wenn nicht aufgepasst wird. 

# Potenzielle Verbesserungen
Die derzeitigen Beschleunigungen und Bremsvorgänge sind allesamt sehr plötzlich, was dazu führt, dass wenn Botty bei größeren Geschwindigkeiten auf null runter bremst, er noch ein kleines Stück rutscht. In der Folge würde auch seine interne Verwaltung der Positionierug über die Zeit kleinere Abweichungen haben. Ein weitere Verbesserung wäre es, die Abweichungswerte anzupassen. Diese wurden erstellt und getestet in einer Phase, als der PhantomX Reactor Arm wegen eines Hardware Schadens abmontiert war. Nun wo er wieder aufgesetzt ist bringt er einiges an Zusatzgewicht mit, wodurch Botty langsamer beschleunigt und fährt. Dadurch erreicht er nicht mehr die gewünschte Distanz beim fahren oder dreht sich zu kurz.

# Autoren
Felix Mayer - Ursprüngliche Arbeiten

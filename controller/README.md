# Controller
## Einleitung
Der Controller dient als Zentrale Einheit für die Kommunikation zwischen den Modulen.

Hier werden einkommende Befehle verwaltet.
Das heißt abstrakte Befehle wie 'go("forward", 2)' werden:
1. Auf konkrete Funktionen und Threads abgebildet,
2. in Teilaufgaben unterteilt,
3. an die Module weitergegeben,
4. auf das Ergebnis gewartet.
Dabei wird auf ein asynchrones Abbruch-Signal reagiert. 

Zurzeit wird auch die Navigation in einem Grid geplant und ausgeführt.

## Verwendete Packages
Nur sound_play für Audio Rückmeldung.

## Dateien
### Nodes
#### Botty.py
Beinhaltet die Klasse "Action", welche die Datenstruktur eines Befehls darstellt.

#### control.py

### Msg
#### Command.msg

#### Entity.msg


### Config


## Lessons Learned  


## Potenzielle Verbesserungen  
 

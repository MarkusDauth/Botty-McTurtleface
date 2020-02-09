# Controller
## Einleitung
Der Controller dient als Zentrale Einheit für die Kommunikation zwischen den Modulen.  

Hier werden einkommende Befehle verwaltet.  
Das heißt abstrakte Befehle wie `go("forward", 2)` werden:  
1. Auf konkrete Funktionen und Threads abgebildet,  
2. in Teilaufgaben unterteilt,  
3. an die Module weitergegeben,  
4. auf das Ergebnis gewartet.  
Dabei wird auf ein asynchrones Abbruch-Signal reagiert.   

Zurzeit wird auch die Navigation in einem Grid geplant und ausgeführt.  

## Verwendete Packages
Nur `sound_play` für Audio Rückmeldung.  

## Dateien
### Nodes
#### Botty.py
Beinhaltet die Klasse `Action`, welche die Datenstruktur eines Befehls darstellt.  

#### control.py
Für den Controller gibt es folgende Wrapper-Klassen  
- `Arm`: Steuerung des Arms  
- `Camera`: Abfrage der Bilderkennung  
- `Nav`: Navigation, Steurerung des Motors  
Die Wrapper-Klassen bilden eine Schnittstelle für die Kommunikation mit den Services/Topics der entsprechenden Module.  
Dabei wird das senden von Befehlen, für welche der Message-Typ und Service/Topic-Name bekannt sein muss, als Methoden abstrahiert.  

Des weiteren gibt es die `Node`-Klasse, die für den A*-Algorithmus verwendet wird.   
A* wird wiederrum in `Nav` für die Navigation verwendet.    

Die Klasse `Controller` fast schließlich alles in einer Steuerungs-Einheit zusammen.  
Es werden die Wrapper-Objekte und `sound_play` initialisiert, dabei wird geg. auf die Services gewartet.  

Für das ausführen von Befehlen gibt es zwei Threads:  
- `mutator_thread`: Für Befehle die Botty's Zustand verändern. (z.B. Base, Arm bewegungen)  
- `observer_thread`: Befehle, die parallel zu Mutator-Befehlen laufen können. (z.B. Antworten, Objekterkennung)  
Es kann also nur max. 2 Befehle gleichzeitig laufen.   

Kommt ein Befehl an den Controller an, wird ausgewertet, um welches Befehl es sich handelt (Action-ID).  
Je nach Aktionstyp wird entweder ein mutator oder observer Thread gestartet.  

Wenn der Thread nicht bereits belegt ist, wird der Befehl mit entsprechenden Parametern asynchron ausgeführt.  
Wenn er aber belegt ist, gibt es keine Warteschlange oder dergleichen,   
das heißt der neue Befehl wird (mit Fehlerrückmeldung) verworfen.  

Bei einem Stop-Befehl wird der Signal an die Wrapper-Objekte weitergegeben und auf das Terminieren der Threads gewartet.  

### Msg
#### Command.msg
Bildet mit Action-ID und Entity eine Struktur, die ein Befehl darstellt.  
 
#### Entity.msg
Besteht aus Name und Liste von Attributen.  

### Config
Es gibt noch nichts zum Konfigurieren.  

## Lessons Learned  
Man sollte sich früher auf Schnittstellen der Module einigen, um abgekapselte Struktur zu ermöglichen.  
Damit kann man mit Dummy-Befehlen arbeiten, die nicht auf eine (fehlende) Implementierung abhängig ist.  

## Potenzielle Verbesserungen  
`control.py` in mehere Dateien aufteilen, eine für jede Klasse.
Bessere Nebenläufigkeit, bis jetzt können nur zwei Threads gleichzeitig laufen.

# Speech
## Einleitung
Dieses Modul dient zur Sprachsteuerung des Roboters.  
Gedacht ist dabei ein Command-and-Control Prinzip.  
Im Grunde besteht es aus 3 Teilaufgaben:  
- Spracherkennung  
- Interpretation der erkannten Phrasen  
- Absenden der Befehle zum Controller  
In diesem Abschnitt wird kurz erläutert, wie das Modul grob funktioniert.  
im Abschnitt "Dateien" wird näher auf die Implementierung eingegangen.  

### Spracherkennung
Für die Spracherkennung wird erst im Keyword-Mode auf ein Wake-Up Phrase gewartet.  
Danach wechselt die Erkennung in den Grammar-Mode, in der auf das eigentliche Sprachkommando abgehört wird.  
Der erkannte Text wird danach an den Parser gesendet.  

### Interpretation / Parsing
Mit dem erkannten Text als Eingabe interpretiert der Parser den semantischen Inhalt des Texts.  
Ein gültiger Befehl wird als Datenstruktur in Form einer art Prädikat an den Controller übergeben.  

## Verwendete Packages
### Pocketsphinx
Webseite mit Tutorial: https://cmusphinx.github.io/wiki/tutorial/  
PocketSphinx bietet ein Framework zur Spracherkennung für "low-resource Platforms" und kann als Package in ROS heruntergeladen werden.  
Daher bietet es sich gut für dieses Projekt an.  
Ein wichtiger Aspekt ist, das die Erkennung in verschiedenen Modi geschehen kann.  
- Keyword-Mode  
  - Es wird auf eine vorbestimmte Phrase im Audiostream gehört.  
  - Der Phrase wird ein Schwellwert für die Trefferwarscheinlichkeit gesetzt.  
- Grammar-Mode  
  - Der Recognizer versucht, die Phrase einer validen Grammatik zuzuorden  
  - Die Grammatik für diesen Vorgang ist Benutzerdefiniert.  
- Language-Model-Mode  
  - Anstatt einer Grammar, wird ein probabilistisches Language-Model genutzt.  
  - Das LM ist ebenfalls Benutzerdefiniert  

### Festival TTS
Dieses Package ermöglicht eine Rückmeldung von Botty durch Töne und Text-To-Speech.  
Nach dem starten der sound_play Node können folgende Befehle ausgeführt werden:  
- Sound abspielen (Eigene Sound Datei oder per ID)  
- Text Ausprechen (Synthese)  

## Dateien
### Nodes
#### send_audio.py
Kapselt die Audio Eingabe und Konfiguration.  
Damit muss man für den Audio Stream nur noch dem Topic sphinx_audio subscriben.  

#### recognizer.py
Mit dem Audio-Stream als Eingabe wird nach Sprachbefehlen gesucht.  
Wurde eins gefunden wird dieser in Text-Form an den Parser weiter gegeben.  
Der Ablauf besteht aus folgenden Schritten:  
1. Im Keyword-Modus wird für das Wake-up-Keyword wie z.B. "Hey Botty" gesucht.  
2. Wake-up gehört?  
   - Ja:    zu Schritt 3  
   - Nein:  weiter hören  
3. Schalte in Grammar-Mode  
4. Höre auf Phrase.  
5. Warte auf das Ende der Phrase.  
6. Suche nach möglicher Interpretation der Grammatik  
6. Befehl gefunden?  
   - Ja:    Sende Text an Parser weiter und gehe zu Schritt 1.  
   - Nein:  Gehe zu Schritt 4  
   
Der Recognizer benötigt zusätzlich zum start bestimmte Konfigurations-Dateien:  
- Grammatik  
- Keyword List  
- Dictionary  
- Acoustic Model  
Diese sind im Ordner config enthalten und werden beim Start des Nodes als Parameter übergeben.    

#### parser.py
Im Parser wird der Text mit primitiven Low-Level-Parsing auf bestimmte Keywords überprüft.  
Es wird nur nach Keywords mit semantischen Informationen überprüft. (Also Wörter wie "go" oder "blue", nicht "the", "please", usw.)  
Zusätzlich wird überprüft, ob der Befehl semantisch, bzw. syntaktisch gültig ist.  

Jede Aktion hat eine ID mit zugehörigen Synonymen, die in der Klasse "Actions" definiert sind.  
Der (gültige) Sprachbefehl wird in eine Datenstruktur gepackt, welche die Form "Action(Objekt(Attribute))" hat.  
Die Datenstruktur ist eine Message namens "Command", welche in controller/msg definiert ist.  

Ablauf der Verarbeitung einer angekommenen Phrase:  
1. Suche nach Aktion  
2. Je nach Aktions-Typ, suche nach zugehörigen Objekten bzw. Orten  
3. Suche nach Liste von Attributen  
4. Fülle Command Objekt mit entsprechenden Werten  
5. Wenn Befehl gültig ist, sende Command-Message an Controller  
(1-3) Bei ungültigem Befehl sende Fehlermeldung  

#### talker.py
Wrapper für die sound_play Node.  

### Config
Grammars: In .gram Dateien sind JSGF Grammars enthalten, die für die Spracherkennung von PocketSphinx verwendet werden.  
Dictionaries: .dic Dateien enthalten ein Lexikon mit phonetischen Representationen von Wörtern, auch für PocketSphinx  
Keyword-list: In .kwlist sind Phrasen mit Schwellwert für Erkennung im Keyword-Modus definiert  
Acoustic Model: cmusphinx-en-us-5.2 ist ein Ordner, welcher ein Acoustic Model für Englisch enthält.  

Für den Recognizer werden folgende Dateien verwendet:  
- botty.gram  
- botty.kwlist  
- cmudict.dic  
- cmusphinx-en-us-5.2  

## Konfiguration  
Wenn mehr/andere Befehle ermöglicht werden sollen, muss  
- config/botty.gram  
- config/cmudict.dic  
- "Action"-Klasse, Token-Listen in parser.py  
eventuell geändert werden.  

Bei einem anderen Acoutic Model z.B. Deutsch anstatt Englisch muss die entsprechende Datei heruntergeladen  
und der Dateipfad als Parameter in der recognizer.launch Datei geändert werden.  
Das selbe gilt natürlich auch bei den anderen Dateien, die in recognizer.launch als Parameter übergeben werden.  

## Lessons Learned  
Suche nach geeignetem Framework:  
PocketSphinx wurde gewählt, weil es in ROS als package integriert ist.  
Im nachhinein ist die Integration von non-ROS-Frameworks in ROS kein großer Aufwand.  
Daher hätte auch ein bekannteres/besseres Framework gewählt werden können. (Einfachere Handhabung, mehr/bessere Tutorials)  

Der Parser hat für eine primitive Analyse gar kein Framework gebraucht.  
Erst war geplant Features von PocketSphinx bezüglich der JSGF-Grammatik für das semantische Parsing zu nutzen.  
So könnte man die selbe Grammatik, die für die Spracherkennung verwendet wird, auch für das Parsing benutzen.  
Diese Features wurden aber leider in neuen Versionen von PocketSphinx entfernt.  
Es benutzt auch leider kein anderes Framework außer PocketSphinx JSGF-Grammatik.  

## Potenzielle Verbesserungen  
Spracherkennung und Parsing durch https://snips.ai/ ersetzten.  
Siehe https://github.com/CMU-TBD/snips_nlu_ros  

Wenn nicht, dann ...:  
- Bis jetzt muss man bei hinzufügen von möglichen Befehlen an vielen stellen Code ändern.  
  Wäre gelöst, wenn die Grammatik für die Spracherkennung auch für das Parsing genutzt werden könnte.  

- Das enum für Aktion ID's kann in "Command"-Message implementiert werden.  

- Passendes Wake-Up-Keyword mit angepassten Schwellwert.  

- Grammatik erweitern, um folgendes zu ermöglichen:  
  - Positionsangabe (z.B. go to position 1 3)  
  - Konjunktion von Befehlen (z.B. find the cup and grab it)  

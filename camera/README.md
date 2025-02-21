# Einleitung

Dieses Modul kommuniziert mit dem Controller und gibt diesem Informationen darüber, ob und welche Objekte erkannt werden.

Dies wurde auf zwei Arten realisiert:
- Der Controller gibt einen Auftrag.
- Wenn Objekte erkannt werden, veröffentlicht das Kamera-Modul auf einem ROS-Topic was gerade gesehen wird.


Im Projekt wurde die Orbbecc Astra 3D Kamera verwendet.
Die Spezifikationen der Kamera sind zu finden unter: https://orbbec3d.com/product-astra-pro/


# Verwendete Packages

- astra_camera:

Hierbei handelt es sich um das Package, welches es erlaubt, die Kamera in ROS zu verwenden.
Folgendes Tutorial wurde befolgt: https://github.com/orbbec/ros_astra_camera

- find_object_2d:

Dieses Package wurde für die Bilderkennung verwendet. In Kombination mit diesem Package wurde folgendes Tutorial verwendet:
https://husarion.com/tutorials/ros-tutorials/4-visual-object-recognition/

# Dateien

## camera_controller.cpp:

Hier findet die Auswertung der Bilderkennung statt. Es wurden zwei ROS-Kommunikationen realisiert:
  - Anfrage zur Bildauswertung als ROS-Service. Dies wurde für den Controller verwendet.
  - Wenn das das Modul ein Objekt erkennt, veröffentlicht es die Objektinformationen als ROS-Topic.
  
## camera.launch:

Hier werden alle nötigen Nodes zur Bilderkennung gestartet.

## FindObjects.srv:

in dieser Datei wird der Rückgabetyp des Services definiert. 
Es wird eine Liste von Strings mit den erkannten Objekten an den Aufrufenden zurückgegeben.

## Ordner image_rec:

In diesem Ordner werden die antrainierten Objekte im .png format gespeichert.

Es wurden folgende Objekte antrainiert:

- Cola Flasche
- Roter Pfeil
- Kreideschachtel
- Blume
- Zeitschrift


## Ordner auswertung:

In diesem Ordner findet sich eine Auswertung in Form einer Konfusionsmatrix, bei der es darum geht, wie gut die Bilderkennung funktioniert. Die Auswertung ist im .pdf und .xlsx Format im Ordner.

# Konfiguration

Um neue Objekte anzutrainieren:

1. Gehe zu "Edit" -> "Add object...",
2. Objekt vor die Kamera halten.
3. Auf "Take Picture" klicken.
4. Das Objekt im Bild markieren und bestätigen.
5. Um die object_id zum Namen des Objektes zu mappen, muss in die Datei camera_controller.cpp über der Funktion set_object_ids eine neue Funktion erstellt werden. Diese Methode soll true zurückliefern, wenn die übergebene object_id, der object_id des gewünschten Objekts entspricht.
Den Wert der object_id kann man über den image_rec Ordner herausfinden. Der Dateiname des zu erkennenden Bildes entspricht der object_id. 
6. In der Methode object_id_to_string muss ein if-else hinzugefügt werden, indem die vorher erstellte Funktion mit der object_id aufgerufen wird.

# Lessons Learned

Während des Projektes habe ich mit vielen Frameworks experimentiert, mit dem Ziel das Handling der Objekterkennung zu erleichtern.
Die Probleme gingen jedoch in den meisten Fällen darauf zurück dass entweder weitere Hardware benötigt wird, oder dass die Nutzung auf Grund der Komplexität der Packages viel zu umständlich ist.
Das größte Problem lag daran, dass die Packages zu viel Funktionalität mit sich mitbrachten, welche für das Projekt nicht nötig sind und dadurch viel Overhead verursachten.

Folgende Frameworks wurden getestet:

- Orbbec API:
  - https://orbbec3d.com/develop/
  - Aufgrund der mangelnden Dokument ist es uns nicht gelungen, die Orbbec-Software auf dem TurtleBot zu installieren.

- joffmann:
  - https://github.com/joffman/ros_object_recognition
  - Bei unseren Tests wurden mehrere Objekte mit diesem Framework antrainiert, jedoch war die Erkennungsquote in allen Fällen sehr niedrig.

- Tensorflow:
  - https://www.tensorflow.org/
  - Tensorflow konnte nicht genutzt werden, da die Trainingsdatensätze sehr groß sind und aufgrund dessen aus Sicht der gegebenen Rechnerleistung nicht anwendbar ist.

- Darknet Yolo:
  - https://pjreddie.com/darknet/yolo/
  - Wäre nutzbar jedoch wird hier eine Cuda konforme Nvidia GPU benötigt, die der TurtleBot jedoch nicht besitzt.


Die weitere Nutzung des find_object_2d packages ist unter Anderem aufgrund der Erkennrate nicht zu empfehlen. Es wurden während des Projektes viele Objekte eingespielt, welche jedoch aufgrund ihrer Größe und oder Komplexität nicht erkannt wurden. Aufgrunddessen wurde entschieden während des Projektes das gleiche Objekt mit mehreren Bildern zu verknüpfen. Dies ist jedoch nicht empfehlenswert da man bei find_object_2d mehrere Objekte nicht auf mehrere Objekte abbilden kann. Dies muss man dann als Programmierer entsprechend beachten.

# Potenzielle Verbesserungen

Einer Weiterentwicklung dieses camera-Packages ist nicht empfohlen. Die tatsächliche Bilderkennung kann man im derzeitigen Stand nicht gut beeinflussen. Was in diesem Package gemacht wird ist, die von der Bilderkennung erhaltenen Daten zu interpretieren.

Aufgrunddessen empfehle ich die Bilderkennung mithilfe von OpenCV(https://opencv.org/) selbst zu realisieren.

Neben dem Mapping von object_id zu Name des Objektes wurde findet sich im Code eine Positionsbestimmung von Objekten, also ob das Objekt aus Sicht der Kamera links oben, rechts oben, links unten oder rechts unten ist. Da diese Funktion im Rahmen des Projektes jedoch nicht mehr verwendet wurde, wurde diese Funktion ausgeschaltet, ist jedoch im Code noch zu finden. 

# Autoren
Raschied Slet - ursprüngliche Arbeiten

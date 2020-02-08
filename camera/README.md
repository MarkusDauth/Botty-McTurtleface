# Einleitung

Dieses Modul kommuniziert mit dem Controller, und gibt diesem Informationen darüber ob und welche Objekte erkannt werden.

Dies wurde auf zwei Arten realisiert:
- Der Controller gibt einen Auftrag.
- Das Kamera Modul veröffentlicht wenn Objekte erkannt werden auf einem ROS-Topic was gerade gesehen wird.


Im Projekt wurde die Orbbecc Astra 3D Kamera verwendet.
Hier finden Sie die Spezifikationen der Kamera: https://orbbec3d.com/product-astra-pro/


# Verwendete Packages

- astra_camera:

Hierbei handelt es sich um das Package, welches es erlaubt, die Kamera in ROS zu verwenden.
Hierzu wurde folgendes Tutorial befolgt: https://github.com/orbbec/ros_astra_camera

- find_object_2d:

Dieses Package wurde für die Bilderkennung verwendet. In Kombination mit diesem Package wurde folgendes Tutorial verwendet:
https://husarion.com/tutorials/ros-tutorials/4-visual-object-recognition/

# Dateien

### camera_controller.cpp:

- Hier findet die Auswertung der Bilderkennung statt. Es wurden hier zwei ROS-Kommunikationen realisiert:
  - Anfrage zur Bildauswertung als ROS-Service. Dies wurde für den Controller verwendet.
  - Das Modul veröffentlicht falls es etwas sieht Informationen darüber als ROS-Topic.
  
### camera.launch:

- Hier werden alle nötigen Nodes zur Bilderkennung gestartet.

### FindObjects.srv:

- In dieser Datei wird der Rückgabetyp des Services definiert. 
Es wird eine Liste von Strings mit den erkannten Objekten an den aufrufenden zurückgegeben.

### Ordner image_rec:

- In diesem Ordner werden die antrainierten Objekte im .png format gespeichert.

### Ordner auswertung:

- In diesem Ordner findet sich in Form einer Konfusionsmatrix eine Auswertung, bei der es darum geht, wie gut die Bilderkennung funktioniert.
Im Ordner befindet sich die Auswertung im .html und .xlsx Format im Ordner.

# Konfiguration

Um neue Objekte anzutrainieren:

1. Gehe zu "Edit" -> "Add object...",
2. Objekt vor die Kamera halten.
3. Auf "Take Picture" klicken.
4. Das Objekt im Bild makieren und besätigen.
5. Dann muss um die object_id zum Namen des Objektes zu mappen in die Datei camera_controller.cpp über der Funktion set_object_ids eine neue Funktion erstellt werden. Diese Methode soll true zurückliefern wenn die übergebene object_id, der object_id des gewünschten Objekts entspricht.
Den Wert der object_id kann man über den image_rec Ordner herausfinden. Der Dateiname des zu erkennenden Bildes entspricht der object_id. 
6. Dann muss in der object_id_to_string methode ein noch ein if else angegeben werden, indem die vorher erstellte Funktion mit der object_id aufgerufen wird.

# Lessons Learned

Während des Projektes habe ich mit vielen Projekten experimentiert, mit dem Ziel das Handling mit der Objekterkennung zu erleichtern.
Die Probleme gingen jedoch in den meisten Fällen darauf zurück dass entweder weitere Hardware benötigt wird, oder dass die Nutzung auf Grund der Komplexität der Packages viel zu umständlich ist.
Das größte Problem lag jedoch daran dass die Packages zu viel Funktionalität mit sich mitbrachten, welche für das Projekt nicht nötig sind und dadurch viel Overhead verursachten.

# Potenzielle Verbesserungen

Falls Sie an dem Projekt weiterarbeiten möchten und die Bilderkennung weiterentwickeln möchten, empfehle ich das von mir verwendete nicht zu verwenden. Die tatsächliche Bilderkennung kann man im derzeitigen Stand nicht gut beeinflussen. Was in diesem Package gemacht wird ist, die von der Bilderkennung erhaltenen Daten zu interpretieren.

Aufgrunddessen empfehle ich die Bilderkennung mithilfe von OpenCV(https://opencv.org/) selbst zu realisieren.

Neben dem Mapping von object_id zu Name des Objektes wurde findet sich im Code eine Positionsbestimmung von Objekten, also ob das Objekt aus Sicht der Kamera links oben, rechts oben, links unten oder rechts unten ist. Da diese Funktion im Rahmen des Projektes jedoch nicht mehr verwendet wurde, wurde diese Funktion ausgeschaltet, ist jedoch im Code noch zu finden. 



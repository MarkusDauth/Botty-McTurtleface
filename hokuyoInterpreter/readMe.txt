Dependencies:
-> hokuyoNode
	https://github.com/ros-drivers/hokuyo_node.git
-> driverCommon
	https://github.com/ros-drivers/driver_common.git

Anmerkung: Erst OHNE hokuyoNode compilieren, dannach mit. 
=>"It's important to compile driver_common BEFORE you added the hokuyo_node. Else compiling could fail."

Funktionalität:
ordnet den Wahrnehmungsbereich in 3 Segmente, left,rigth,front (derzeit nur front implementiert)
wird in der front ein Hindernis wahrgenommen, bzw. eine Distanz-Grenze unterschritten, wird ein Stop-Signal gesendet.
diverse attribute, die das Verhaltensmuster beeinflussen, werden in der botty/hokuyoInterpreter/scripts/config.xml definiert.
=>
start- und endpunkte der 3 Winkelbereiche, sowie deren Seitenverlauf. ['start','end','site']
den Sicherheitsabstand, der zu den Seiten des Winkelbereichs verringert wird, falls 'site'>0 [collision]
den konstanten Sicherheitsabstand, der nicht von den Winkel Seiten beeinflusst ist ['constantSecureRange']
die Kalibrierungsreichweite ['callibration']

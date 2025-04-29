#DCC to DCC Railcom Cutout (ESP32)

Transformez automatiquement un signal DCC "brut" en un signal DCC compatible Railcom avec insertion de cutout.

Présentation :

Ce projet permet, à partir d’une centrale DCC standard qui n’en est pas équipée, de 
-	Capturer proprement le signal DCC brut.
-	Analyser et reconstruire les paquets DCC.
-	Générer un nouveau flux DCC
-	Un préambule DCC de 16 bits est nécessaires pour la détection Railcom
-	Un cutout Railcom (~464 µs) après chaque trame DCC, indispensable pour permettre le retour d'information Railcom

Pour les détails techniques, voir : https://www.nmra.org/sites/default/files/standards/sandrp/DCC/S/S-9.3.2_2012_12_10.pdf

Le projet est destiné à être embarqué sur un ESP32.

Matériel nécessaire
- 1 ESP32
- Pont en H pour amplifier le signal DCC
- Entrée optocouplée pour isoler l'entrée DCC.

![Schéma optocoupleur](dcc_optocoupleur.png)






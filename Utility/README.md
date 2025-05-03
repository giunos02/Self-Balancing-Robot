# Self-Balancing-Robot

Questo progetto prevede la realizzazione di un prototipo di Self-Balancing Robot e l'implementazione di un controllo PID sullo stesso per ottenere l'auto-bilanciamento e controllarne il movimento.
Ci si è serviti di:
  - Microcontrollore ESP32-WROOM-32;
  - Sensore GY-521 MPU6050;
  - Sensori di distanza VL53L1X;
  - Driver ponte-H L298N;
  - Regolatore di tensione HW-316;
  - Motori DC 370rpm con encoder;
  - Display OLED SSD1306.

Il telaio del prototipo è progettato su Autodesk Fusion 360 e stampato in PLA. 
L'ambiente di svuluppo utilizzato per l'implementazione del codice è Arduino IDE, versione 2.3.6 (download -> https://www.arduino.cc/en/software/) e le librerie utilizzate sono state WiFi.h, math.h, MPU6050.h, WebServer.h, Wire.h, Adafruit_VL53L1X.h, Adafruit_SSD1306.h, Adafruit_GFX.h, PID_v1.h, la maggior parte delle quali non sono presenti di default e dovranno essere installate appositamente (Strumenti -> Gestisci librerie... -> inserisci nome nella barra di ricerca -> Installa).

Le principali funzioni presenti nel codice sono per il calcolo dell'angolo di inclinazione mediante il sensore inerziale IMU installato nell'MPU6050, il controllo PID per l'auto-bilanciamento e la realizzazione di un'applicazione web (si è sfruttato il modulo Wi-Fi del microcontrollore, utilizzandolo come Access Point e Web Server) per la visualizzazione, in tempo reale, delle distanze misurate dai sensori di profondità e di suolo, dell'angolo di inclinazione e della distanza percorsa dal robot (calcolata mediante gli encoder presenti nei motori). Inoltre, figurano la casella per il tuning del PID e il joystick per il controllo del movimento nelle varie direzioni. 

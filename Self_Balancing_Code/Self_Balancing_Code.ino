#include <WiFi.h>
#include <math.h>
#include <MPU6050.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_VL53L1X.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <PID_v1.h>

// Definizione dei pin per i motori
#define Encoder1PinA 34
#define Encoder1PinB 35
#define Encoder2PinA 32
#define Encoder2PinB 33
#define ForwardPin 12
#define BackwardPin 14
#define ForwardPin2 27
#define BackwardPin2 26
#define EnablePin 13
#define EnablePin2 25

#define sampleTime 0.001

// Configurazione Access Point ESP32
const char* ssid = "ESP32_AP";
const char* password = "12345678";

WebServer server(80);

//Indirizzo I2C del display
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(128, 64, &Wire, -1); //display

// Configurazione I2C per i sensori di distanza
Adafruit_VL53L1X sensor1 = Adafruit_VL53L1X();
Adafruit_VL53L1X sensor2 = Adafruit_VL53L1X();

// Creazione oggetto per l'IMU
MPU6050 mpu;

// Creazione di un secondo bus I2C
TwoWire I2C_2 = TwoWire(1);

//Variabili per i dati dell'accelerometro e del giroscopio
int16_t ax, ay, az;
int16_t gx, gy, gz;

//Variabili per calcolare gli angoli
float accAngle, currentAngle=0, prevAngle=0, gyroAngle=0;
float lastTime, deltaTime;
unsigned long currentMillis = 0;  //Tempo corrente per la visualizzazione dei dati

//Coefficiente di smorzamento del filtro
float alpha = 0.985;

// Variabile velocità motore volatile 
volatile int pwmSpeed; 

//Variabili PID
double Kp=15;
double Ki=12;
double Kd=0.06;
double setPoint = 0;   //Valore desiderato
double targetAngle;
double input, output;
int selectedParameter = -1;

PID myPID(&input, &output, &setPoint, Kp, Ki, Kd, DIRECT);

volatile long Encoder1value = 0;
volatile long Encoder2value = 0;

const float impulsi = 11;
const float raggio = 0.06;
const float circonferenza = 2 * PI * raggio;
const float distanzaPerImpulso = circonferenza / impulsi;

bool motoriAttivi = false;
bool ostacoloVicino = false;
bool suoloNonRilevato = false;


// Pagina web
const char webpage[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>ESP32 Robot Control</title>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <script src="https://cdnjs.cloudflare.com/ajax/libs/nipplejs/0.9.0/nipplejs.min.js"></script>
  <style>
    body {
      font-family: Arial, sans-serif;
      background-color: #f4f4f4;
      text-align: center;
      margin: 0;
      padding: 0;
    }
   
    .container {
      position: absolute;
      width: 100%;
      height: 100%;
    }
    .sensor-box,  .motor-box {
      position: absolute;
      width: 140px;
      padding: 5px;
      background: white;
      border-radius: 10px;
      box-shadow: 0 0 8px rgba(30, 60, 150, 0.5); 
      text-align: center;
    }
    .popup {
      display: none;
      position: absolute;
      top: 30%;
      left: 50%;
      transform: translate(-50%, -50%);
      background: red;
      color: white;
      padding: 20px;
      font-size: 20px;
      border-radius: 10px;
      box-shadow: 0 0 8px rgba(0, 0, 0, 0.5);
      z-index: 9999;
    }

    #form-container {
    position: relative;
    top: 20%;
    left: 50%;
    transform: translateX(-50%);
    width: 150px;
    padding: 10px;
    background: white;
    border-radius: 20px;
    box-shadow: 0 0 8px rgba(30, 60, 150, 0.5); 
    text-align: center;
  }


  
    #sensor1 { top: 10px; left: 10px; }
    #sensor2 { top: 10px; right: 10px; }
    #motor1 { bottom: 10px; left: 10px; }
    #imu-box { bottom: 10px; right: 10px; }  

    form { max-width: 150px; top: 20%; }
    input[type='submit'] { width: 100%; padding: 20px; margin: 5px 0; font-size: 1em; }
    input[type='submit'] { background-color: blue; color: white; border: none; weight: 20px; }

    .distance { font-size: 18px; font-weight: bold; transition: color 0.5s ease-in-out; }
    .safe { color: #007BFF; } 
    .alert { color: red; }

     #joystick-container {
      position: absolute; top: 67%; right: 20px;
      transform: translate(-50%, -50%);
      width: 175px; height: 175px; background: white;
      border-radius: 50%;   
      box-shadow: 0 0 8px rgba(30, 60, 150, 0.5); 
    }
  </style>
</head>
<body>
  <div id="popup" class="popup"> </div>
  <div class="container">
    <div class="sensor-box" id="sensor1">
      <h2>Sensore 1</h2>
      <p class="distance safe" id="distance1">-- cm</p>
    </div>
    <div class="sensor-box" id="sensor2">
      <h2>Sensore 2</h2>
      <p class="distance safe" id="distance2">-- cm</p>
    </div>
    <div class="motor-box" id="motor1">
      <h2>Distanza</h2>
      <p id="motorDistance">-- m</p>
    </div>
      <div id="form-container">
        <form action='/set' method='GET'>
          <p>Set Kp: <input type='number' name='Kp' step='0.1' min='0' max='100' value='%.1f'></p>
          <p>Set Ki: <input type='number' name='Ki' step='0.1' min='0' max='200' value='%.1f'></p>
          <p>Set Kd: <input type='number' name='Kd' step='0.01' min='0' max='3' value='%.2f'></p>
          <input type='submit' value='Update'>
        </form>
      </div>
    <div class="motor-box" id="imu-box">
      <h2>Angolo IMU</h2>
    <p id="imu-angle">--°</p>
    </div>
    <div id="joystick-container"></div>

  </div>
  <script>
     function updateData() {
      fetch('/data')
      .then(response => response.json())
      .then(data => {
        let distance1 = document.getElementById("distance1");
        let distance2 = document.getElementById("distance2");
        let mdistance = document.getElementById("motorDistance");
        distance1.innerHTML = data.sensor1 + " cm";
        distance2.innerHTML = data.sensor2 + " cm";
        mdistance.innerHTML = data.motor1 + " m";

        if (data.sensor1 < 30) {
          distance1.classList.add("alert");
          distance1.classList.remove("safe");
        } else {
          distance1.classList.add("safe");
          distance1.classList.remove("alert");
        }

        if (data.sensor2 > 25) {
          distance2.classList.add("alert");
          distance2.classList.remove("safe");
        } else {
          distance2.classList.add("safe");
          distance2.classList.remove("alert");
        }

        let popup = document.getElementById("popup");
        if (data.suolo) {
          popup.textContent = "Suolo non rilevato!";
          popup.style.display = "block";
        } else if(data.ostacolo) {
          popup.textContent = "Ostacolo troppo vicino!";
          popup.style.display = "block";
        } else {
          popup.style.display = "none";
        }
    });
  }

    function updateIMU() {
      fetch('/imu')
      .then(response => response.json())
      .then(data => {
      document.getElementById("imu-angle").innerHTML = data.angle + "°";
      });
    }
    
    function sendJoystickData(x, y) {
      fetch(`/control?x=${x}&y=${y}`);
    }
    
    var joystick = nipplejs.create({
      zone: document.getElementById("joystick-container"),
      mode: "static",
      position: { left: "50%", top: "50%" },
      color: "#0000ff"
    });

    joystick.on("move", function(evt, data) {
      let x = Math.round(data.vector.x * 255);
      let y = Math.round(data.vector.y * 255);
      sendJoystickData(x, y);
    });

    joystick.on("end", function() {
      sendJoystickData(0, 0);
    });


    setInterval(updateData, 100); // Aggiorna i dati ogni secondo
    setInterval(updateIMU, 100);  // Aggiorna l'angolo ogni millisecondo

  </script>
</body>
</html>
)rawliteral";


// Funzione per gestire l'aggiornamento dei valori dal server 
  void handleSet() { 
    if (server.hasArg("Kp")) { 
      Kp = server.arg("Kp").toFloat(); 
    } 
    if (server.hasArg("Ki")) { 
      Ki = server.arg("Ki").toFloat(); 
    } 
    if (server.hasArg("Kd")) { 
      Kd = server.arg("Kd").toFloat(); 
    } 
    
    // Reindirizza alla pagina principale 
    server.sendHeader("Location", "/"); 
    server.send(303); 
  }  

    float getSelectedParameterValue() { 
      switch (selectedParameter) { 
        case 0: return Kp;
        case 1: return Ki; 
        case 2: return Kd; 
        default: return 0; 
      } 
    } 

void updateEncoder1() {
if (digitalRead(Encoder1PinA)> digitalRead(Encoder1PinB))
Encoder1value++;
else 
Encoder1value--;
}

void updateEncoder2() {
if (digitalRead(Encoder2PinA)> digitalRead(Encoder2PinB)) 
Encoder2value++;
else 
Encoder2value--;
}

void fermaMotori() {
  digitalWrite(ForwardPin, LOW);
  digitalWrite(BackwardPin, LOW);
  digitalWrite(ForwardPin2, LOW);
  digitalWrite(BackwardPin2, LOW);
  ledcWrite(EnablePin, 0);
  ledcWrite(EnablePin2, 0);
}

void controllaMotori(int x, int y) {
  int speed = abs(y);
  int turn = abs(x);

  if (y > 50) {
    digitalWrite(ForwardPin, HIGH);
    digitalWrite(BackwardPin, LOW);
    digitalWrite(ForwardPin2, HIGH);
    digitalWrite(BackwardPin2, LOW);
    analogWrite(EnablePin, speed);
    analogWrite(EnablePin2, speed);
  } else if (y < -50) {
    digitalWrite(ForwardPin, LOW);
    digitalWrite(BackwardPin, HIGH);
    digitalWrite(ForwardPin2, LOW);
    digitalWrite(BackwardPin2, HIGH);
    analogWrite(EnablePin, speed);
    analogWrite(EnablePin2, speed);
  } else if (x > 200) {
    digitalWrite(ForwardPin, LOW);
    digitalWrite(BackwardPin, HIGH);
    digitalWrite(ForwardPin2, HIGH);
    digitalWrite(BackwardPin2, LOW);
    analogWrite(EnablePin, turn);
    analogWrite(EnablePin2, turn);
  } else if (x < -200) {
    digitalWrite(ForwardPin, HIGH);
    digitalWrite(BackwardPin, LOW);
    digitalWrite(ForwardPin2, LOW);
    digitalWrite(BackwardPin2, HIGH);
    analogWrite(EnablePin, turn);
    analogWrite(EnablePin2, turn);
  } else if (x > 50) {
    digitalWrite(ForwardPin, HIGH);
    digitalWrite(BackwardPin, LOW);
    digitalWrite(ForwardPin2, HIGH);
    digitalWrite(BackwardPin2, LOW);
    analogWrite(EnablePin, turn-10);
    analogWrite(EnablePin2, turn);
  } else if (x < -50) {
    digitalWrite(ForwardPin, HIGH);
    digitalWrite(BackwardPin, LOW);
    digitalWrite(ForwardPin2, HIGH);
    digitalWrite(BackwardPin2, LOW);
    analogWrite(EnablePin, turn);
    analogWrite(EnablePin2, turn-10);
  }else {
    fermaMotori();
  }
}

void handleControl() {
  if (server.hasArg("x") && server.hasArg("y")) {
    int x = server.arg("x").toInt();
    int y = server.arg("y").toInt();

    if (abs(y) < 50) {
      setPoint = 0;
    } else {
      setPoint = map(y, -255, 255, 5, -5);
    
    }
    controllaMotori(x, y);
    Serial.printf("Joystick X: %d, Y: %d -> SetPoint: %.2f\n", x, y, setPoint);
  }
  server.send(200, "text/plain", "OK");
}

void aggiornaMotori() {
  if (ostacoloVicino || suoloNonRilevato) {
    fermaMotori();
  }
}


void handleRoot() {
  server.send(200, "text/html", webpage);
}

void handleData() {
  float distanza1 = sensor1.distance() / 10.0;
  float distanza2 = sensor2.distance() / 10.0;
  float distanzaMotori = (Encoder1value * distanzaPerImpulso + Encoder2value * distanzaPerImpulso) / 2;

  ostacoloVicino = (distanza1 < 10 || distanza2 < 10);
  suoloNonRilevato = (distanza2 > 30);
  aggiornaMotori();

  String json = "{ \"sensor1\": " + String(distanza1) +
                ", \"sensor2\": " + String(distanza2) +
                ", \"motor1\": " + String(distanzaMotori) +
                ", \"ostacolo\": " + (ostacoloVicino ? "true" : "false") + 
                ", \"suolo\": " + (suoloNonRilevato ? "true" : "false") + " }";

  server.send(200, "application/json", json);
}

//Modificare le funzioni per girare in base a quella che verrà scelta

void handleIMU() {
  String json = "{ \"angle\": " + String(currentAngle, 2) + " }";  
  server.send(200, "application/json", json);
}

void setup() {
  Serial.begin(115200);

  //Comunicazione I2C imu
  Wire.begin(22, 23);  // SDA, SCL per Display, Sensore 1 e IMU
  
  //Comunicazione I2C sensori di distanza
  I2C_2.begin(18, 19); // SDA, SCL per Sensore 2;
  sensor1.begin(0x29, &Wire);
  sensor2.begin(0x29, &I2C_2);
  sensor1.startRanging();
  sensor2.startRanging();

  WiFi.softAP(ssid, password);
  Serial.println("Access Point Creato!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.softAPIP());

  server.on("/", []() { server.send_P(200, "text/html", webpage); });
  server.on("/control", handleControl);
  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/imu", handleIMU);
  server.on("/set", HTTP_GET, handleSet); 

  server.begin();
  Serial.println("Http server started");

  //Inizializzazione motori
pinMode(ForwardPin,OUTPUT);
pinMode(BackwardPin,OUTPUT);
pinMode(EnablePin,OUTPUT);

pinMode(ForwardPin2,OUTPUT);
pinMode(BackwardPin2,OUTPUT);
pinMode(EnablePin2,OUTPUT);


pinMode(Encoder1PinA, INPUT);
pinMode(Encoder1PinB, INPUT);

pinMode(Encoder2PinA, INPUT);
pinMode(Encoder2PinB, INPUT);

attachInterrupt(digitalPinToInterrupt(Encoder1PinA), updateEncoder1, RISING);
attachInterrupt(digitalPinToInterrupt(Encoder2PinA), updateEncoder2, RISING);

fermaMotori();

   // DISPLAY 
      display.begin(SCREEN_ADDRESS, 0x3C); 
      display.clearDisplay(); 
      display.setTextSize(1); 
      display.setTextColor(SSD1306_WHITE); 
      display.setCursor(0, 0);
      display.display(); 

  //MPU
  mpu.initialize();
  //Controllo inizializzazione MPU
  if (!mpu.testConnection()) {
    Serial.println("Errore: MPU6050 non trovato!");
    while (1); // Blocca l'esecuzione
    }

  //Calibrazione dell'accelerometro e del giroscopio
  display.print("Calibrating \naccelerometer...");
  display.display();
  mpu.CalibrateAccel(10);

  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("Calibrating \ngyroscope...");
  display.display();
  mpu.CalibrateGyro(20);

  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("Calibrated!");
  display.println("");
  display.display();
  delay(1000);

  //Imposta il filtro passa-basso per ridurre il rumore
  mpu.setDLPFMode(MPU6050_DLPF_BW_20);

  //Inizializza PID
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-255, 255);
  myPID.SetSampleTime(sampleTime *1000);

  // Inizializza la variabile del tempo 
      lastTime = millis(); 
}

void loop() {

noInterrupts();
long enc1 = Encoder1value;
long enc2 = Encoder2value;
interrupts();

float distanza1 = enc1 * distanzaPerImpulso;
float distanza2 = enc2 * distanzaPerImpulso;


myPID.SetTunings(Kp, Ki, Kd); 


  //Lettura dati al sensore
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  //Calcola l'angolo dell'accelerometro (in gradi)
  accAngle = atan2((float)az, (float)ax)* RAD_TO_DEG -90;

  //Calcola il tempo trascorso dall'ultima lettura
  float CurrentTime = millis();
  deltaTime = (CurrentTime - lastTime) /1000.0;
  lastTime = CurrentTime;

  //Calcola l'angolo del giroscopio (in gradi)
  float gyroRate = (float)gy / 131.0;
  gyroAngle += gyroRate * deltaTime;   //131.0 è la sensibilità del giroscopio (+-250°/s)

  currentAngle = alpha * (prevAngle + (gyroRate * deltaTime)) + (1 - alpha) * (accAngle);   // Filtro complementare

  //Calcola l'input per il PID
  input = currentAngle;
  //Calcola il PID
  myPID.Compute();
  // Controllo dei motori basato sul PID 

pwmSpeed = constrain(abs(output), 0, 150); // Limita la velocità tra 0 e 100 
 if (output > 0) { 
    digitalWrite(ForwardPin, HIGH); 
    digitalWrite(BackwardPin, LOW); 
    analogWrite(EnablePin, abs(pwmSpeed));

    digitalWrite(ForwardPin2, HIGH); 
    digitalWrite(BackwardPin2, LOW); 
    analogWrite(EnablePin2, abs(pwmSpeed));
} else { 
    digitalWrite(ForwardPin, LOW); 
    digitalWrite(BackwardPin, HIGH); 
    analogWrite(EnablePin, abs(pwmSpeed));  

    digitalWrite(ForwardPin2, LOW); 
    digitalWrite(BackwardPin2, HIGH);
    analogWrite(EnablePin2, abs(pwmSpeed));  
} 
 
  server.handleClient();

   //Visualizzazione dei dati su display OLED
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);

  //Stampa dei dati accelerometro e giroscopio
  display.print("Current Angle: "); display.println(currentAngle);
  display.println("");

  //Debug: Stampa i valori di input e output del PID
  display.print("Input: "); display.println(input);
  display.print("Output: "); display.println(output);
  display.print("Kp: "); display.println(Kp);
  display.print("Ki: "); display.println(Ki);
  display.print("Kd: "); display.println(Kd);

  static unsigned long lastDisplayUpdate = 0;
  if(millis() - lastDisplayUpdate > 500) {
    lastDisplayUpdate = millis();
    display.display();   //Aggiorna il display ogni 500ms
  }

  prevAngle = currentAngle;
  Serial.println(-currentAngle);
  delay(deltaTime);


}
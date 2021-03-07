
const int BUTTON_PIN = 0; //10
const int WATER_PIN = 1; //8
const int LED_PIN = 4; //12
const int TOUCH_PIN = 3;
//#define BLYNK_PRINT Serial
#define FIREBASE_URL "esri-eea51.firebaseio.com"
#define FIREBASE_SECRET "qULRQbnuzJ3qkAdYunMasgcLlKtpXic1UQ1FxHvM"
#define WIFI_SSID "Arora-2.4"
#define WIFI_PASSWORD "brockway123"
#define Lat_Long "30.332628 77.998622"

//#include <ESP8266WiFi.h>
//#include <BlynkSimpleEsp8266.h>
#include <FlowMeter.h>

#include <WiFiEspAT.h>

int WATER_DETECTION_DELAY = 10000;

WiFiClient client;
const unsigned long delayPeriod = 2000;
float water_flow_sensor=A0;
const float flow_threshold = 20;
float current_flow = 0;
float previous_flow = 0;

int flowPin = 2;    //This is the input pin on the Arduino
double flowRate;    //This is the value we intend to calculate. 
volatile int count; //This integer needs to be set as volatile to ensure it updates correctly during the interrupt process.  


// connect a flow meter to an interrupt pin (see notes on your Arduino model for pin numbers)
FlowMeter *Meter;
// define an 'interrupt service handler' (ISR) for every interrupt pin you use

boolean waterDetected = false;
boolean alertSent = false;
boolean ledOn = false;

//char auth[] = "FdW9wy8Tcj58iEuYYZVt69_j1_3qnaT2";
//
//
//char ssid[] = "satellitecity";
//char pass[] = "faceface";

void MeterISR() {
    // let our flow meter count the pulses
    Meter->count();
}


void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(WATER_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT);
  pinMode(TOUCH_PIN, INPUT);
  // Serial for debugging
  Serial.begin(115200);
  // Serial for ESP8266 communication
  Serial3.begin(115200);
  WiFi.init(&Serial3);

  setupWifi();  

  Meter = new FlowMeter(digitalPinToInterrupt(2), UncalibratedSensor, MeterISR, RISING);

  setupFlowSensor();

}



void setupWifi() {
    // Connect to WiFi
  Serial.print("Connecting to WiFi - ");
  Serial.println(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
//    Serial.print(".");
  }
  Serial.println();
  Serial.println("Connected to WiFi...");

  printWifiStatus();

}

void setupFlowSensor() {
  // put your setup code here, to run once:
  pinMode(flowPin, INPUT);           //Sets the pin as an input
  attachInterrupt(0, Flow, RISING);  //Configures interrupt 0 (pin 2 on the Arduino Uno) to run the function "Flow"  
}

void Flow()
{
   count++; //Every time this function is called, increment "count" by 1
}

void printWifiStatus() {
  // Print the SSID of the network you're attached to
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // Print your WiFi shield's IP address
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // Print the received signal strength
  long rssi = WiFi.RSSI();
  Serial.print("Signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}


void printWaterFlowStatus(){
  count = 0;      // Reset the counter so we start counting from 0 again
  interrupts();   //Enables interrupts on the Arduino
  delay (100);   //Wait 1 second 
  noInterrupts(); //Disable the interrupts on the Arduino
   
  //Start the math
  flowRate = (count * 2.25);        //Take counted pulses in the last second and multiply by 2.25mL 
  flowRate = flowRate * 60;         //Convert seconds to minutes, giving you mL / Minute
  flowRate = flowRate / 1000;       //Convert mL to Liters, giving you Liters / Minute


  previous_flow = current_flow;
  current_flow = flowRate;

  if (previous_flow <= 0.1 && current_flow > 0.1) {
    Serial.println("Water Flow Detected");         //Print the variable flowRate to Serial
  }

  if (current_flow > 0.1) {
    Serial.println(flowRate);         //Print the variable flowRate to Serial
  }

  if (current_flow <= 0.1 && previous_flow > 0.1) {
    Serial.println("Water Flow Stopped");         //Print the variable flowRate to Serial
  }

}

boolean checkForWaterLeak(boolean addDelay) {
  if (flowRate > 0.1) {
    if (addDelay) {
      Serial.println("checkForWaterLeak water - with Delay");

//k.notify("Your Hardware Has Detected Water Flow For 1 Hour");
      delay(WATER_DETECTION_DELAY);
      Serial.println("delay added");
      if (flowRate > 0.1) {
        waterDetected = true;
        Serial.println("checkForWaterLeak water Detected");
      } else {
        waterDetected = false;
      }
    } else {
      Serial.println("checkForWaterLeak water Detected NO DELAY");
//      Blynk.notify("Your Hardware Has Detected Water Flow");
      waterDetected = true;
    }
  } else {
    waterDetected = false;
  }
  return waterDetected;
}

void sendAlert() {
  if (alertSent) {
      return;
  }
  Serial.println("sending alert");
  digitalWrite(LED_PIN, HIGH);
  alertSent = true;
}

void clearAlertIfNeeded() {
  if (digitalRead(BUTTON_PIN) == LOW) {
    Serial.println("clearAlertIfNeeded - performing system reset");
    digitalWrite(LED_PIN, LOW);
    waterDetected = false;
    alertSent = false;
  }
}

 


void loop() {
//  Blynk.run();

    printWaterFlowStatus();

  clearAlertIfNeeded();
  if (waterDetected == true) {
    sendAlert();
  }

checkForWaterLeak(true);

  if(digitalRead(TOUCH_PIN) == HIGH){
    checkForWaterLeak(true);
  }else{
    checkForWaterLeak(false);
  }

}

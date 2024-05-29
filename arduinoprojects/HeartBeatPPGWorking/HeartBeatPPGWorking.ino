/*
  MAX30105 Breakout: Output all the raw Red/IR/Green readings
  By: Nathan Seidle @ SparkFun Electronics
  Date: October 2nd, 2016
  https://github.com/sparkfun/MAX30105_Breakout

  Outputs all Red/IR/Green values.

  Hardware Connections (Breakoutboard to Arduino):
  -5V = 5V (3.3V is allowed)
  -GND = GND
  -SDA = A4 (or SDA)
  -SCL = A5 (or SCL)
  -INT = Not connected

  The MAX30105 Breakout can handle 5V or 3.3V I2C logic. We recommend powering the board with 5V
  but it will also run at 3.3V.

  This code is released under the [MIT License](http://opensource.org/licenses/MIT).
*/

#include <Wire.h>
#include "MAX30105.h"
#include <PeakDetection.h>
#include "KickFiltersRT.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include "Adafruit_MPU6050.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "ScioSense_ENS160.h"

Adafruit_BME280 bme; // I2C
ScioSense_ENS160 ens160(ENS160_I2CADDR_1);
volatile float BME_T = 0;
volatile float BME_RH = 0;
volatile float ENS_TVOC = 0;
volatile float ENS_CO2 = 0;
const char* ssid = "iPhone";
const char* password = "derin2001";
bool control = false;

const char* mqtt_server = "18.169.68.55";
const int mqtt_port = 1883;
const char* mqtt_user = "da621";
const char* mqtt_password = "derin2001";
const char* mqtt_topic = "control";
WiFiClient espClient;
PubSubClient mqttClient(espClient);

PeakDetection peakDetection;
KickFiltersRT<float> filtersRT;

MAX30105 particleSensor;
const float fs = 100;
const int BUFFER_SIZE = 10;


bool peakDetected = false; // Flag to indicate if a peak has been detected

#define debug Serial //Uncomment this line if you're using an Uno or ESP
//#define debug SerialUSB //Uncomment this line if you're using a SAMD21

void callback(char* topic, byte* payload, unsigned int length) {
//  Serial.print("Message arrived [");
//  Serial.print(topic);
//  Serial.print("] ");
//  for (int i = 0; i < length; i++) {
//    Serial.print((char)payload[i]);
//  }
//  Serial.println();

  String message = "";
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  if (message.equals("Start")) {
    control = true;
    Serial.println("Start signal received.");
  } else if (message.equals("Stop")) {
    control = false;
    Serial.println("Stop signal received.");
  }
  

}

void reconnect() {
  // Loop until we're reconnected
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (mqttClient.connect("ESP32Client", mqtt_user, mqtt_password)) {
      Serial.println("connected");
      mqttClient.subscribe("control"); 

    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup()
{
  debug.begin(9600);
  debug.println("MAX30105 Basic Readings Example");
  peakDetection.begin(48, 3, 0.6);               // sets the lag,F threshold and influence

  // Initialize sensor
  if (particleSensor.begin() == false)
  {
    debug.println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
  }

  particleSensor.setup(); //Configure sensor. Use 6.4mA for LED drive


  
//
//  WiFi.begin(ssid, password);
//
//  Serial.print("Connecting to WiFi");
//  while (WiFi.status() != WL_CONNECTED) {
//    delay(500);
//    Serial.print(".");
//  }
//
//  // Connected to Wi-Fi
//  Serial.println("");
//  Serial.println("WiFi connected");
//  Serial.println("IP address: ");
//  Serial.println(WiFi.localIP());
//
//  mqttClient.setServer(mqtt_server, mqtt_port);
//  mqttClient.setCallback(callback);
//  mqttClient.connect("ESP32Client", mqtt_user, mqtt_password);
//  mqttClient.subscribe("control"); 


//  bme.begin();  
//    if(!bme.begin()){
//     Serial.println("BME Failed");
//     return;
//    }
//
//    bme.setSampling(Adafruit_BME280::MODE_NORMAL,
//                    Adafruit_BME280::SAMPLING_X4,   // temperature
//                    Adafruit_BME280::SAMPLING_NONE, // pressure off
//                    Adafruit_BME280::SAMPLING_X4,   // humidity
//                    Adafruit_BME280::FILTER_OFF,
//                    Adafruit_BME280::STANDBY_MS_0_5 );
//
//  ens160.begin();
//    if(!ens160.begin()){
//     Serial.println("ens160 Failed");
//     return;
//    }
//  ens160.setMode(ENS160_OPMODE_STD);
}
unsigned long lastSensorReadTime = 0;

float last=0;
float last2=0;
float inter =3000;
bool sign= false; 
float lasttime =0;
int count = 0; 
float lastt =0;
double lastbpm = 0;
void loop()
{

//    BME_T = bme.readTemperature();
//    BME_RH = bme.readHumidity();
//    Serial.print("Temp: ");
//    Serial.println(BME_T);

    unsigned long c = millis();

    if (c - lastSensorReadTime >= 3000) {
        lastSensorReadTime = c;
        ens160.measure();
        ENS_TVOC = ens160.getTVOC();
        ENS_CO2 = ens160.geteCO2();
    }
    

        
  double lpfiltered = filtersRT.lowpass(particleSensor.getIR(), 0.5, fs);
  double hpfiltered = filtersRT.highpass(lpfiltered, 4, fs);
  double difference = hpfiltered- last;
  last = hpfiltered;
  double squared = difference*difference;
  peakDetection.add(squared);                     // adds a new data point
  double moving = peakDetection.getFilt();
  int peak = peakDetection.getPeak();
  if(peak ==1 && inter>2000){
     lasttime = millis();
//     debug.println(moving+100);

  }


  
  double difference2 = (moving-last2)*100;
  last2= moving;
  debug.println(moving);
//  debug.print(",");
  inter = millis()- lasttime;

  if (difference2>10 && inter>400){
    double interval = (millis() - lasttime)/(1000*60);
    double bpm = 1 / interval;
    lasttime = millis();
//    debug.println(moving+50);

    if(45 < bpm){
//      debug.print("bpm1: ");
//      debug.println(bpm);
    }
    
    
    

    if ((millis()-lastt)>15000){
//      debug.print("bpm2: ");
//      debug.println(count*4);
      count = 0;
      lastt = millis();
    }else{
      count++;
    }
    

  }

//  if (!mqttClient.connected()) {
//      reconnect();
//    }
//    mqttClient.loop();
  
//  buffer2.push_back(moving);
//  if (buffer2.size()>BUFFER_SIZE){
//    buffer2.erase(buffer2.begin());
//  }
//if (difference2 > 0) {
//    buffer.push_back(1); // Add 1 to buffer if difference is positive
//  } else {
//    buffer.push_back(0); // Add 0 to buffer if difference is non-positive
//  }
//
//  if (buffer.size() > BUFFER_SIZE) {
//    buffer.erase(buffer.begin());
//  }
//
//  if (!peakDetected) { 
//    int sumFirstHalf = 0;
//    int sumSecondHalf = 0;
//
//    for (int i = 0; i < BUFFER_SIZE / 2; i++) {
//      sumFirstHalf += buffer[i];
//    }
//    for (int i = BUFFER_SIZE / 2; i < BUFFER_SIZE; i++) {
//      sumSecondHalf += buffer[i];
//    }
//
//    double max_val = *std::max_element(buffer2.begin(), buffer2.end());
//    double min_val = *std::min_element(buffer2.begin(), buffer2.end());
//    inter = millis()- lasttime;
//
//    if (sumFirstHalf > 4 && sumSecondHalf < 1 && inter>1000) {
//
//      buffer.clear();
//      peakDetected = true; // Set peak detected flag
//      lasttime = millis();
//
//    }
//  }
//
//  // Reset peak detected flag if there are no positive differences in the buffer
//  if (std::find(buffer.begin(), buffer.end(), 1) == buffer.end()) {
//    peakDetected = false;
//  }

//
//  
//  last= bpfiltered;
//  
//  debug.println(bpfiltered);
  
//  debug.println(filtered);

}

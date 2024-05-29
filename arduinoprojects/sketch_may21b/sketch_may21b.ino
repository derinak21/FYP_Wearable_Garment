
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


MAX30105 particleSensor;



bool peakDetected = false; // Flag to indicate if a peak has been detected

#define debug Serial //Uncomment this line if you're using an Uno or ESP


void setup()
{
  Serial.begin(9600);
  Serial.println("MAX30105 Basic Readings Example");

//  if (particleSensor.begin() == false)
//  {
//    debug.println("MAX30105 was not found. Please check wiring/power. ");
//    while (1);
//  }
//
//  particleSensor.setup(); //Configure sensor. Use 6.4mA for LED drive
//

  bme.begin();  
    if(!bme.begin()){
     Serial.println("BME Failed");
     return;
    }
    else{
     Serial.println("BME Success");

    }

    bme.setSampling(Adafruit_BME280::MODE_NORMAL,
                    Adafruit_BME280::SAMPLING_X4,   // temperature
                    Adafruit_BME280::SAMPLING_NONE, // pressure off
                    Adafruit_BME280::SAMPLING_X4,   // humidity
                    Adafruit_BME280::FILTER_OFF,
                    Adafruit_BME280::STANDBY_MS_0_5 );

  ens160.begin();
    if(!ens160.begin()){
     Serial.println("ens160 Failed");
     return;
    }
    else{
     Serial.println("ENS Success");

    }
  ens160.setMode(ENS160_OPMODE_STD);
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

    BME_T = bme.readTemperature();
    BME_RH = bme.readHumidity();


    unsigned long c = millis();

    if (c - lastSensorReadTime >= 3000) {
        lastSensorReadTime = c;
        ens160.measure();
        ENS_TVOC = ens160.getTVOC();
        ENS_CO2 = ens160.geteCO2();
        Serial.print("Temp: ");
        Serial.println(BME_T);
        Serial.print("Humidity: ");
        Serial.println(BME_RH);
        Serial.print("TVOC: ");
        Serial.println(ENS_TVOC);
        Serial.print("CO2: ");
        Serial.println(ENS_CO2);
    }
    


}

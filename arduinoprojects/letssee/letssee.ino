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
MAX30105 particleSensor;
PeakDetection peakDetection;
KickFiltersRT<float> filtersRT;
#define debug Serial //Uncomment this line if you're using an Uno or ESP
//#define debug SerialUSB //Uncomment this line if you're using a SAMD21
const float fs = 100;
const int BUFFER_SIZE = 10;
void setup()
{
  debug.begin(9600);
  debug.println("MAX30105 Basic Readings Example");
  delay(2000);
  Wire1.setPins(SDA1, SCL1);
  Wire1.begin();
  delay(2000);

  particleSensor.begin(Wire1, 400000, 0x57);
  // Initialize sensor
  while (particleSensor.begin(Wire1, 400000, 0x57) == false)
  {
    debug.println("MAX30105 was not found. Please check wiring/power. ");
    delay(1000);
  }
   debug.println("MAX30105 was found.");
  particleSensor.setup(0x1F, 4, 2, 100, 411, 16384); // Example settings

   peakDetection.begin(20, 3, 0.6);               // sets the lag,F threshold and influence
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
 
 double lpfiltered = filtersRT.lowpass(particleSensor.getIR(), 4, fs);
  double hpfiltered = filtersRT.highpass(lpfiltered, 1, fs);
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
}

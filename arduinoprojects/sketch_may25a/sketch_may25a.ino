#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <DFRobot_ENS160.h>
#include "MAX30105.h"

DFRobot_ENS160_I2C ENS160(&Wire1, /*I2CAddr*/ 0x53);

MAX30105 particleSensor;

// Define the secondary Wire object for the secondary I2C port

Adafruit_BME280 bme; // I2C
volatile float BME_T = 0;
volatile float BME_RH = 0;
volatile float ENS_TVOC = 0;
volatile float ENS_CO2 = 0;

void setup() {
  // Initialize serial communication at a baud rate of 9600
  Serial.begin(9600);
  while (!Serial); // Wait for serial port to connect
  Serial.println("QT Py ESP32 BME280 Test"); // Print a message to the serial monitor
  Wire1.setPins(SDA1, SCL1);


  // Initialize the BME280 sensor with the secondary Wire object
  if(!bme.begin(0x77, &Wire1)) {
    Serial.println("BME280 initialization failed!");
    while (1); // halt the program
  } else {
    Serial.println("BME280 initialized successfully!");
  }

  bme.setSampling(Adafruit_BME280::MODE_NORMAL,
                  Adafruit_BME280::SAMPLING_X4,   // temperature
                  Adafruit_BME280::SAMPLING_NONE, // pressure off
                  Adafruit_BME280::SAMPLING_X4,   // humidity
                  Adafruit_BME280::FILTER_OFF,
                  Adafruit_BME280::STANDBY_MS_0_5 );


  while( NO_ERR != ENS160.begin() ){
    Serial.println("Communication with device failed, please check connection");
    delay(3000);
  }
  Serial.println("Begin ok!");
  ENS160.setPWRMode(ENS160_STANDARD_MODE);
  ENS160.setTempAndHum(/*temperature=*/25.0, /*humidity=*/50.0);

    particleSensor.begin(Wire1, 100000, 0x57);
  // Initialize sensor
  while (particleSensor.begin(Wire1, 100000, 0x57) == false)
  {
    debug.println("MAX30105 was not found. Please check wiring/power. ");
    delay(1000);
  }
   debug.println("MAX30105 was found.");
  particleSensor.setup(); //Configure sensor. Use 6.4mA for LED drive
}


void loop() {
  // Read temperature and humidity from the BME280 sensor
  BME_T = bme.readTemperature();
  BME_RH = bme.readHumidity();

  // Print the temperature and humidity readings
  Serial.print("Temperature: ");
  Serial.print(BME_T);
  Serial.println(" °C");
  Serial.print("Humidity: ");
  Serial.print(BME_RH);
  Serial.println(" %");

  uint8_t AQI = ENS160.getAQI();
  Serial.print("Air quality index : ");
  Serial.println(AQI);

  /**
   * Get TVOC concentration
   * Return value range: 0–65000, unit: ppb
   */
  uint16_t TVOC = ENS160.getTVOC();
  Serial.print("Concentration of total volatile organic compounds : ");
  Serial.print(TVOC);
  Serial.println(" ppb");

  /**
   * Get CO2 equivalent concentration calculated according to the detected data of VOCs and hydrogen (eCO2 – Equivalent CO2)
   * Return value range: 400–65000, unit: ppm
   * Five levels: Excellent(400 - 600), Good(600 - 800), Moderate(800 - 1000), 
   *               Poor(1000 - 1500), Unhealthy(> 1500)
   */
  uint16_t ECO2 = ENS160.getECO2();
  Serial.print("Carbon dioxide equivalent concentration : ");
  Serial.print(ECO2);
  Serial.println(" ppm");

  Serial.println();

  
  delay(2000); // Wait for 1 second before reading again
}

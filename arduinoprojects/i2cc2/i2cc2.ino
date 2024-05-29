// ESP32 Frequency Meter
// Code by Kristel Fobelets and Kris Thielemans 11/2/2023
// Adapted from:
// https://blog.eletrogate.com/esp32-frequencimetro-de-precisao
// Rui Viana and Gustavo Murta august/2020
// pins https://github.com/espressif/arduino-esp32/blob/master/variants/esp32s2/pins_arduino.h
// if the board disappears from Tools then follow the steps described in:
// https://blog.espressif.com/arduino-for-esp32-s2-and-esp32-c3-is-coming-f36d79967eb8

/* set pulse counters for the ESP32 WROOM 32D collpit oscillator implementation
//#define PCNT_INPUT_SIG_IO_0     GPIO_NUM_26                               
//#define PCNT_INPUT_CTRL_IO_0    GPIO_NUM_32                               
//#define PCNT_INPUT_SIG_IO_1     GPIO_NUM_27                              
//#define PCNT_INPUT_CTRL_IO_1    GPIO_NUM_33                             
//#define PCNT_H_LIM_VAL          overflow

Wire.begin(); //Join the bus as controller. 
  //By default .begin() will set I2C SCL to Standard Speed mode of 100kHz
  Wire.setClock(400000); //Optional - set I2C SCL to High Speed Mode of 400kHz
*/

#include "stdio.h"                                                        // Library STDIO
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include "driver/pcnt.h"                                                  // Library ESP32 PCNT
#include "soc/pcnt_struct.h"

#include "Adafruit_MPU6050.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "ScioSense_ENS160.h"

void init_cardreader();
void initTimers();
void init_sensors();
void init_PCNT(pcnt_unit_t unit, int pcnt_input_sig_io, int pcnt_input_ctrl_io);
void init_frequencyMeter();
void write_header();
void read_data();
void read_counters();
void start_counters();
void write_data();
void read_PCNT(void *);

Adafruit_BME280 bme; // I2C
ScioSense_ENS160 ens160(ENS160_I2CADDR_1);
Adafruit_MPU6050 mpu;

SPIClass spi = SPIClass(2);
File output;
const char * const SD_filename = "/derinakoutputfile2.csv";

#define PCNT_COUNT_UNIT_0       PCNT_UNIT_0                              // Set Pulse Counter Unit
#define PCNT_COUNT_UNIT_1       PCNT_UNIT_1                              // Set Pulse Counter Unit
#define PCNT_COUNT_CHANNEL      PCNT_CHANNEL_0                           // Set Pulse Counter channel

#define PCNT_INPUT_SIG_IO_0     GPIO_NUM_26                               // Set Pulse Counter input - Freq Meter Input
#define PCNT_INPUT_CTRL_IO_0    GPIO_NUM_32                               // Set Pulse Counter Control GPIO pin - HIGH = count up, LOW = count down  
#define PCNT_INPUT_SIG_IO_1     GPIO_NUM_27                              // Set Pulse Counter input - Freq Meter Input
#define PCNT_INPUT_CTRL_IO_1    GPIO_NUM_33                             // Set Pulse Counter Control GPIO pin - HIGH = count up, LOW = count down  
#define PCNT_H_LIM_VAL          overflow                                 // Overflow of Pulse Counter 

volatile bool   flag            = true;                                  // Flag to enable print frequency reading
const uint32_t  overflow        = 32000;                                 // Max Pulse Counter value 32000
const uint32_t  sample_time     = 10000;                                 // sample time in microseconds to count pulses
int             print_counter   = 0;                                     // count when we store results
const int       print_every     = 1;                                    // how many samples we skip before storing
volatile float  esp_time      = 0;                                     // time elapsed in microseconds since boot
volatile float  esp_time_interval = 0;                                 // actual time between 2 samples (should be close to sample_time)
volatile float  frequency_0   = 0;                                     // frequency value
volatile float  frequency_1   = 0;                                     // frequency value
volatile float BME_T = 0;
volatile float BME_RH = 0;
volatile float ENS_TVOC = 0;
volatile float ENS_CO2 = 0;
volatile float ax_x = 0;
volatile float ax_y = 0;
volatile float ax_z = 0; 

portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;                    // portMUX_TYPE to do synchronisation


//----------------------------------------------------------------------------------------
void setup()
{
Serial.begin(9600);                                                   // Serial Console Arduino 115200 Bps
spi.begin(); 
init_cardreader();
init_frequencyMeter();                                                 // Initialize Frequency Meter
init_sensors();
}

void write_header()
{
 output = SD.open(SD_filename,FILE_APPEND);
output.println("t (esp timer), f0 (Hz), f1 (Hz), T (C), RH (%), VOC (ppb), CO2 (ppm), ax (m/s2), ay (m/s2), az (m/s2)"); //will only work at 1s sampling time
// output.println("t (esp timer), f0 (Hz), f1 (Hz), T (C), RH (%), ax (m/s2), ay (m/s2), az (m/s2)");
 output.close();
}

void init_cardreader()
{
  if (!SD.begin()) {
    Serial.println("Card Mount Failed");
    return;
  }
  output = SD.open(SD_filename, FILE_WRITE);                            // clear output.csv file on SD card reader
  if(!output){
    Serial.println("Failed to open file for writing");
    return;
  }
  output.close();
  write_header();
}

void init_sensors()
{
    bme.begin();  
    if(!bme.begin()){
     Serial.println("BME Failed");
     return;
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
  ens160.setMode(ENS160_OPMODE_STD);

    mpu.begin();  
    if(!mpu.begin()){
     Serial.println("MPU Failed");
     return;
    }

    mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

}

void read_data()
{
 BME_T = bme.readTemperature();
 BME_RH = bme.readHumidity();
 ens160.measure();
 ENS_TVOC = ens160.getTVOC();
 ENS_CO2 = ens160.geteCO2();
 
 sensors_event_t a, g, temp;
 mpu.getEvent(&a, &g, &temp);
 ax_x = a.acceleration.x;
 ax_y = a.acceleration.y;
 ax_z = a.acceleration.z; 
//  Serial.printf("frequency_0: %.2f\n", frequency_0);
// Serial.printf("frequency_1: %.2f\n", frequency_1);
// Serial.printf("temperature: %.2f\n", BME_T);
// Serial.printf("humidity: %.2f\n", BME_RH);
// Serial.printf("Voc: %.2f\n", ENS_TVOC);
// Serial.printf("co2: %.2f\n", ENS_CO2);
// Serial.printf("x: %.2f\n", ax_x);
// Serial.printf("y: %.2f\n", ax_y);
// Serial.printf("z: %.2f\n", ax_z); 
// Serial.printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
//               frequency_0, frequency_1, BME_T, BME_RH, ENS_TVOC, ENS_CO2, ax_x, ax_y, ax_z);

}

//----------------------------------------------------------------------------------
void init_PCNT(pcnt_unit_t unit, int pcnt_input_sig_io, int pcnt_input_ctrl_io)                                                      // Initialize and run PCNT unit
{
  pcnt_config_t pcnt_config = { };                                        // PCNT unit instance

  pcnt_config.pulse_gpio_num = pcnt_input_sig_io;                         // Pulse input GPIO - Freq Meter Input
  pcnt_config.ctrl_gpio_num = pcnt_input_ctrl_io;                         // Control signal input GPIO
  pcnt_config.unit = unit;                                                // Counting unit PCNT - 0
  pcnt_config.channel = PCNT_COUNT_CHANNEL;                               // PCNT unit number - 0
  pcnt_config.counter_h_lim = PCNT_H_LIM_VAL;                             // Maximum counter value
  pcnt_config.pos_mode = PCNT_COUNT_INC;                                  // PCNT positive edge count mode - inc
  pcnt_config.lctrl_mode = PCNT_MODE_DISABLE;                             // PCNT low control mode - disable
  pcnt_config.hctrl_mode = PCNT_MODE_KEEP;                                // PCNT high control mode - won't change counter mode
  pcnt_unit_config(&pcnt_config);                                         // Initialize PCNT unit

  pcnt_counter_pause(unit);                                               // Pause PCNT unit
  pcnt_counter_clear(unit);                                               // Clear PCNT unit

  pcnt_event_enable(unit, PCNT_EVT_H_LIM);                                // Enable event to watch - max count
  pcnt_intr_enable(unit);                                                 // Enable interrupts for PCNT unit
}

//----------------------------------------------------------------------------------
void read_counters()                                                    // Read Pulse Counters
{
  pcnt_counter_pause(PCNT_COUNT_UNIT_0); 
  pcnt_counter_pause(PCNT_COUNT_UNIT_1); 
  const int64_t esp_time_now = esp_timer_get_time();
  int16_t pulses_0 = 0;
  int16_t pulses_1 = 0;
  pcnt_get_counter_value(PCNT_COUNT_UNIT_0, &pulses_0);                   // Read Pulse Counter value
  pcnt_get_counter_value(PCNT_COUNT_UNIT_1, &pulses_1);                   // Read Pulse Counter value
  frequency_0 = pulses_0  ;                                               // Calculation of frequency. 
  pcnt_counter_clear(PCNT_COUNT_UNIT_0);                                  // Clear Pulse Counter
  frequency_1 = pulses_1 ;                                                // Calculation of frequency. 
  pcnt_counter_clear(PCNT_COUNT_UNIT_1);                                  // Clear Pulse Counter
  esp_time_interval = esp_time_now - esp_time;
 }

void start_counters()
{
  esp_time = esp_timer_get_time();
  pcnt_counter_resume(PCNT_COUNT_UNIT_0); 
  pcnt_counter_resume(PCNT_COUNT_UNIT_1); 
}

void write_data()
{
      // Note: esp_time is the time at the end of the read_PCNT
      output = SD.open(SD_filename,FILE_APPEND); 
      read_data();
      output.printf("%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f \n", esp_time, frequency_0, frequency_1, BME_T, BME_RH, ENS_TVOC, ENS_CO2, ax_x, ax_y, ax_z);
      //output.printf("%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f \n", esp_time, frequency_0, frequency_1, BME_T, BME_RH, ax_x, ax_y, ax_z);
      output.close();
}

void read_PCNT(void *)                                                    // Read Pulse Counter callback
{
  read_counters();
  //read_data();
  start_counters();
  flag = true;                                                            // Change flag to enable print
}
//---------------------------------------------------------------------------------
void init_frequencyMeter ()
{
  // Initialize and run PCNT units
  init_PCNT(PCNT_COUNT_UNIT_0, PCNT_INPUT_SIG_IO_0, PCNT_INPUT_CTRL_IO_0);
  init_PCNT(PCNT_COUNT_UNIT_1, PCNT_INPUT_SIG_IO_1, PCNT_INPUT_CTRL_IO_1);

  // create periodic timer for reading-out PCNTs
  esp_timer_create_args_t create_args;
  esp_timer_handle_t timer_handle;
  create_args.callback = read_PCNT;                                       // Set esp-timer argument
  esp_timer_create(&create_args, &timer_handle);                          // Create esp-timer instance
  esp_timer_start_periodic(timer_handle, sample_time);                    // Initialize High resolution timer (dependent on sampling time.
  start_counters();
}

//---------------------------------------------------------------------------------
void loop()
{
   float t = millis()/1000.0; // get program time
    Serial.print(t);           // output time in seconds as first variable
    Serial.print(" ");         // add spacing between variables
    
    float var_sin = sin(t); 
    Serial.print(var_sin);  // output sin(t) variable
    Serial.print(" ");      // add spacing between variables

    float var_cos = cos(t); 
    Serial.print(var_cos);  // output cos(t) variable
    Serial.println();       // this just prints a \n character if you don't provide an argument
  if (flag == true)                                                     // If count has ended
  {
    flag = false;                                                       // change flag to disable print
    print_counter++;
    if (print_counter>=print_every)
    {
      print_counter = 0;
      write_data();
    }
  }
  
  delay(1); // units are millisecs
}


//--------SD card reader functions ---------------------------------------------------------------------

void writeFile(fs::FS &fs, const char * path, const char * message){
  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if(!file){
    Serial.println("Failed to open file for writing");
    return;
  }
  if(file.print(message)){
    Serial.println("File written");
  } else {
    Serial.println("Write failed");
  }
  file.close();
}

void appendFile(fs::FS &fs, const char * path, const char * message){
  Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if(!file){
    Serial.println("Failed to open file for appending");
    return;
  }
  if(file.print(message)){
    Serial.println("Message appended");
  } else {
    Serial.println("Append failed");
  }
  file.close();
}

void renameFile(fs::FS &fs, const char * path1, const char * path2){
  Serial.printf("Renaming file %s to %s\n", path1, path2);
  if (fs.rename(path1, path2)) {
    Serial.println("File renamed");
  } else {
    Serial.println("Rename failed");
  }
}

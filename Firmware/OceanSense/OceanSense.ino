//      ******************************************************************
//      *                                                                *
//      *                             Ocean Sense                        *
//      *                                                                *
//      *         M. Goldwater                            11/10/2020     *
//      *              Copyright (c) Mark Goldwater & Co., 2020          *
//      *                                                                *
//      ******************************************************************

#include "RTClib.h"
#include "SparkFun_Si7021_Breakout_Library.h"
#include "SparkFun_MMA8452Q.h"
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <string.h>

#define DEBUG // define for serial port debugging

#define TEMP_HUMID_BUFFER 12 // number of temperature and humidity samples to store before writing to SD card
#define MINUTES_TO_MILLISECONDS 60000 // multiplication factor to convert minutes to milliseconds
#define SECONDS_TO_MILLISECONDS 1000 // multiplication factor to convert seconds to milliseconds

//
// create real-time clock object
//
RTC_DS3231 rtc;

//
// create temperature and humidity sensor object
//
Weather tempHumid;

//
// create accel object
//
MMA8452Q accel;

//
// pin assignments
//
const int SPI_CS_PIN = 10;
const int LED_PIN = 4;

//
// SD card file object
//
File logFile;

//
// buffers for intermittent termperature and humidity measurements as well as timestamps
//
char timeStamps[TEMP_HUMID_BUFFER][30];
char timeStamp[30];
float temperatures[TEMP_HUMID_BUFFER];
float humidities[TEMP_HUMID_BUFFER];

void setup() {
   
   //
   // start serial monitor
   // wait for it to connect. Needed for nativ USB
   //
   #ifdef DEBUG
   Serial.begin(9600);
   while (!Serial);
   #endif

   //
   // configure pins
   //
   pinMode(LED_PIN, OUTPUT);

   
   digitalWrite(LED_PIN, HIGH);

   //
   // initialize and begin temperature and humidity sensor
   //
   tempHumid.begin();
   delay(200);

   //
   // check accelerometer
   //
   if (accel.begin() == false) {
    #ifdef DEBUG
    Serial.println("Accel not connected");
    #endif
    while(1);
   }
   
   //
   // initialize the SD card
   //
   #ifdef DEBUG
   Serial.println("Initializing SD Card...");
   #endif
   if (!SD.begin(SPI_CS_PIN)) {
      #ifdef DEBUG
      Serial.println("Initialization Failed");
      #endif
      while(1);
   }
   #ifdef DEBUG
   Serial.println("Initialization Done.");
   #endif
   delay(200);

   //
   // check for config file
   //
   if (SD.exists("config.txt")) {
     processConfgurationFile();
   }

   //
   // set up CSV columns
   //
   if (!SD.exists("logth.csv")) {
    setUpTempHumidCsvColumns();
   }
   delay(1000);
   if (!SD.exists("loga.csv")) {
    setUpAccelCsvColumns();
   }
}

void loop() {
  //
  // main function to run the OceanSense platform
  //
  collectData();
}

// ********************************************************************
// |                    DATA COLLECTION FUNCTION                      |
// ********************************************************************

void collectData() {
  //
  // counter to index into buffer
  // 
  unsigned short buffer_counter = 0;

  //
  // setup timers for temperature and humidity data collection
  //
  const unsigned long update_temp_humid_period = 0.5 * MINUTES_TO_MILLISECONDS; // left coefficient [minutes] varies how often teperature and humidity data is collected
  unsigned long update_timer_temp_humid = millis();
  unsigned long elapsed_time_temp_humid;
  
  //
  // setup timers for accelerometer data collection
  //
  const unsigned long update_accelerometer_period = 1 * MINUTES_TO_MILLISECONDS; // left coefficient [minutes] varies how often accelerometer data is collected
  const unsigned long accelerometer_data_collection_period = 5 * SECONDS_TO_MILLISECONDS; // left coefficient [seconds] varies how long accelerometer data is collected for
  unsigned long update_timer_accelerometer_period = millis();
  unsigned long elapsed_time_accelerometer;

  //
  // setup timers to blink LED
  //
  const unsigned long update_LED_period = 2 * SECONDS_TO_MILLISECONDS; // left coefficient [seconds] varies the period of the LED blink
  const unsigned long LED_blink_period = 250; // left coefficient [milliseconds] varies how long the LED stays on for in its blink
  unsigned long update_timer_LED = millis();
  unsigned long elapsed_time_LED;
  
  //
  // flags for data collection
  //
  bool collect_accel = false;
  bool collect_temp_and_humid = false;
  bool LED_on = false;

  //
  // main data colletion loop
  //
  while(1) {
    if (buffer_counter == TEMP_HUMID_BUFFER) {
      //
      // when buffer is filled unload to SD card
      //
      writeTemperatureAndHumidityToSD();

      //
      // reset buffer index counter
      //
      buffer_counter = 0;
    }

    //
    // log temperature and humidity data on timer
    //
    if (collect_temp_and_humid) {
      logTemperatureAndHumidity(buffer_counter);
      #ifdef DEBUG
      Serial.print(temperatures[buffer_counter]);
      Serial.print(" ");
      Serial.println(humidities[buffer_counter]);
      Serial.println(timeStamps[buffer_counter]);
      #endif
      
      //
      // iterate buffer index
      // 
      buffer_counter += 1;

      //
      // stop temp/humidity data collection
      //
      collect_temp_and_humid = false;
    }

    //
    // log accelerometer data on timer
    //
    if (collect_accel) {
      logAndWriteAccelerometerToSD();
    }
    
    //
    // timer to flip flag for collection of temperature an humidity data
    //
    elapsed_time_temp_humid = millis() - update_timer_temp_humid;
    if (elapsed_time_temp_humid >= update_temp_humid_period) {
      collect_temp_and_humid = true;
      update_timer_temp_humid = millis();
    }

    //
    // timer to flip flag ON for collection of accelerometer data 
    //
    if (collect_accel == false) {
      elapsed_time_accelerometer = millis() - update_timer_accelerometer_period;
      if (elapsed_time_accelerometer >= update_accelerometer_period) {
         collect_accel = true;
         update_timer_accelerometer_period = millis();
      }
    } else {
      //
      // timer to flip flag OFF for collection of accelerometer data
      //
      elapsed_time_accelerometer = millis() - update_timer_accelerometer_period;
      if (elapsed_time_accelerometer >= accelerometer_data_collection_period) {
         collect_accel = false;
         update_timer_accelerometer_period = millis();
      }
    }

    //
    // timer to turn ON the LED
    //
    if (LED_on == false) {
      elapsed_time_LED = millis() - update_timer_LED;
      if (elapsed_time_LED >= update_LED_period) {
         digitalWrite(LED_PIN, HIGH);
         LED_on = true;
         update_timer_LED = millis();
      }
    } else {
      //
      // timer to turn OFF the LED
      //
      elapsed_time_LED = millis() - update_timer_LED;
      if (elapsed_time_LED >= LED_blink_period) {
         digitalWrite(LED_PIN, LOW);
         LED_on = false;
         update_timer_LED = millis();
      }
    }
  }
}

// ********************************************************************
// |                          HELPER FUNCTIONS                        |
// ********************************************************************

void writeTemperatureAndHumidityToSD() {
  #ifdef DEBUG
  Serial.println("Writing Temperature and Humidity Data to SD Card.");
  #endif

  //
  // open temperature and humidity log file
  //
  logFile = SD.open("logth.csv", FILE_WRITE);

  //
  // if log file opened properly unload buffers
  //
  if (logFile) {
    for (int i = 0; i < TEMP_HUMID_BUFFER; i++) {
      logFile.print(timeStamps[i]);
      logFile.print(", ");
      logFile.print(temperatures[i]);
      logFile.print(", ");
      logFile.println(humidities[i]);
    }

    //
    // close file
    //
    logFile.close();
    #ifdef DEBUG
    Serial.println("Done.");
    #endif
  } else {
    #ifdef DEBUG
    Serial.println("Error opening log file");
    #endif
  }
}

void logAndWriteAccelerometerToSD() {
  //
  // open temperature and humidity log file
  //
  logFile = SD.open("loga.csv", FILE_WRITE);

  //
  // if log file opened properly take data
  //
  if (logFile) {
    if (accel.available()) {
      getTimeStamp();
      logFile.print(timeStamp);
      logFile.print(", ");
      logFile.print(accel.getCalculatedX(), 3);
      logFile.print(", ");
      logFile.print(accel.getCalculatedY(), 3);
      logFile.print(", ");
      logFile.print(accel.getCalculatedZ(), 3);
      logFile.println();
    }
    
    //
    // close file
    //
    logFile.close();
  } else {
    #ifdef DEBUG
    Serial.println("Error opening log file");
    #endif
  }
}

void getTimeStamp() {
   //
   // get current time from RTC
   //
   DateTime now = rtc.now();

   //
   // format timestamp and store it
   //
   snprintf(timeStamp, sizeof(timeStamp), "%02d/%02d/%04d %02d:%02d:%02d", now.month(), now.day(), now.year(), now.hour(), now.minute(), now.second());
}

void updateTimeStamp(unsigned short buffer_index) {
   //
   // get current time from RTC
   //
   DateTime now = rtc.now();

   //
   // format timestamp and store it
   //
   snprintf(timeStamps[buffer_index], sizeof(timeStamps[buffer_index]), "%02d/%02d/%04d %02d:%02d:%02d", now.month(), now.day(), now.year(), now.hour(), now.minute(), now.second());
}

void logTemperatureAndHumidity(unsigned short buffer_index) {
   //
   // measure humidity and temperature
   //
   float humidity = tempHumid.getRH();
   float temperature = tempHumid.getTempF();

   //
   // add humidity and temperature measurements to respectie buffers
   //
   humidities[buffer_index] = humidity;
   temperatures[buffer_index] = temperature;
   
   //
   // add timestamp to timestamp buffer
   //
   updateTimeStamp(buffer_index);
}

void processConfgurationFile() {
  //
  // configure buffers for reading config file
  //
  char* configBuff;
  char* token;
  char* DATE;
  char* TIME;
  unsigned short apply;
  
     
  logFile = SD.open("config.txt");
  if (logFile) {
    //
    // get the file size
    //
    unsigned int fileSize = logFile.size();

    //
    // create buffer to read the file
    //
    configBuff = (char*) malloc(fileSize + 1);

    //
    // read the file into the buffer
    //
    logFile.read(configBuff, fileSize);

    //
    // add string terminator when done writing to buffer
    //
    configBuff[fileSize] = '\0';

    //
    // close the file
    //
    logFile.close();
  } else {
   #ifdef DEBUG
   Serial.print("Error Opening Configuration File");
   #endif
  }

  //
  // pull first token using deliminators '=' and '\n'
  //
  token = strtok(configBuff, "=\n");

  //
  // while the token exists, parse the config file
  //
  while(token != NULL) {
   //
   // store date setting
   //
   if (!strcmp(token, "DATE")) {
     DATE = strtok(NULL, "=\n");
     token = DATE;
   } 

   //
   // store time setting
   //
   if(!strcmp(token, "TIME")) {
     TIME = strtok(NULL, "=\n");
     token = TIME; 
   }

   //
   // store boolean to determine to apply the config or not
   //
   if(!strcmp(token, "APPLY")) {
     apply = atoi(strtok(NULL, "=\n"));
   }

   //
   // get next setting name
   //
   token = strtok(NULL, "=\n");
 }

 //
 // free the config file buffer
 //
 free(configBuff);

 //
 // process config file if apply flag is '1'
 //
 if (apply) {
  rtc.adjust(DateTime(DATE, TIME));
  #ifdef DEBUG
  Serial.println("Applied Config!");
  #endif
 }
}

void setUpAccelCsvColumns() {
  
  //
  // open acceleration log file
  //
  logFile = SD.open("loga.csv", FILE_WRITE);

  //
  // if log file opened properly write columns
  //
  if (logFile) {
    logFile.print("TimeStamp, ");
    logFile.print("X, ");
    logFile.print("Y, ");
    logFile.print("Z, ");
    logFile.println();
    
    //
    // close file
    //
    logFile.close();
  } else {
    #ifdef DEBUG
    Serial.println("Error opening accel log file");
    #endif
  }
}

void setUpTempHumidCsvColumns() {
  //
  // open acceleration log file
  //
  logFile = SD.open("logth.csv", FILE_WRITE);

  //
  // if log file opened write columns
  //
  if (logFile) {
    logFile.print("TimeStamp, ");
    logFile.print("Temp, ");
    logFile.print("Humidity, ");
    logFile.println();
    
    //
    // close file
    //
    logFile.close();
  } else {
    #ifdef DEBUG
    Serial.println("Error opening th log file");
    #endif
  }
}

//CanOS - CanSat operating system.

//Data4programmer:
//Use a global mode switch delay == to about 3000 millis
//Use redundancy when RAM is reset (store to sd)
//Base the Altitude on GPS if we have funding???
//See SD storage comments later for info about data-recovery


//Includes
#include <Wire.h> //For interfacing, stuff like IsquaredC
#include "i2c.h"

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h> //for reading ye-olde gpsshit

//#include "i2c_BMP280.h" //altimeter library

#include <Adafruit_MPL3115A2.h> //altimeter library

//Important Globals
const int BUZZER_PIN = 5;
const long BUZZER_INTERVAL = 1;

const long ALTPT_INTERVAL = 300;

const int DIODE_PIN = 8;

const long GPS_INTERVAL = 9600;

const float GROUND_PRESS = 1008.81; //!!!!

const float ZONE_ACTIVE = 1.1; //altitude standby <=> activeM
const float ZONE_SAMPLE = 1500; //altitude active <=> microbe & viral biomatter sample collection


//Instantiates
//BMP280 bmp280; //initiate CanSatKit altimeter
//SoftwareSerial GPSserial(GPSTX, GPSRX);

TinyGPSPlus gps; //make yeolde gpssss
Adafruit_MPL3115A2 baro = Adafruit_MPL3115A2(); //initiate altimeter


//Utility Globals
String currentMode = "ACTIVE";
String inboundString = "";
String lastError = "";

bool serialVerbosity = false;

float currentAltitude = 0;
float altHistory = 0;

int currentMillis = 0;

unsigned long buzzerMillis = 0;
int buzzerState = LOW;

unsigned long altptMillis = 0;


//Functions
void readSerial() {
  if (Serial.available() > 0) {
    inboundString = Serial.readString();

    Serial.print("[>] Command recieved: ");
    Serial.println(inboundString);
    
    if (inboundString == "errorec") {
      if (lastError = "") {
        Serial.println("[i] Nothing to report in Error Recall mode.");
      } else {
        Serial.print("[!] Last error was: ");
        Serial.println(lastError);
      }
    }

    if (inboundString == "mode standby") {
      currentMode = "STANDBY";

      Serial.println("[*] Manually entering CanSat STANDBY MODE.");
    }

    if (inboundString == "mode active") {
      currentMode = "ACTIVE";

      Serial.println("[*] Manually entering CanSat ACTIVE MODE.");
    }

    if (inboundString == "log verbose") {
      serialVerbosity = true;

      Serial.println("[*] CanSat debug is now VERBOSE.");
    }

    if (inboundString == "log data") {
      serialVerbosity = false;

      Serial.println("[*] CanSat debug is now raw DATA.");
    }

    if (inboundString == "record alt") {
      Serial.print("[*] Average altitude: ");
      Serial.println(altHistory);

      altHistory = currentAltitude;
    }
  }
}

void emitBuzz(unsigned long currentMillis) {
  if (currentMillis - buzzerMillis >= BUZZER_INTERVAL) {
    buzzerMillis = currentMillis;

    if (buzzerState == LOW) {
      buzzerState = HIGH;
    } else {
      buzzerState = LOW;
    }

    digitalWrite(BUZZER_PIN, buzzerState);
  }
}

void sampleAltitude(unsigned long currentMillis) {
  if (currentMillis - altptMillis >= ALTPT_INTERVAL) {
    altptMillis = currentMillis;

    //here we want to addiitonally record the current readings.
    //surrogated with output for now

    float pascal = baro.getPressure();
    float temperature = baro.getTemperature();
   
    if (serialVerbosity == true) {
      Serial.println("[C] FlightConditions update:");
      Serial.print("  | Pressure in HPA: "); Serial.println(pascal / 100);
      Serial.print("  | Temperature in C: "); Serial.println(temperature);
    }

    //float perceivedAlt = 44330 * (1.0 - pow(pressure_HPA / GROUND_PRESS, 0.1903)); //so gettem maths or whateva iva ksp

    float hypsoAlt = ((pow(GROUND_PRESS / (pascal / 100), 1.0/5.257) - 1) * (temperature + 273.15)) / 0.0065;

    currentAltitude = hypsoAlt;

    altHistory = (9 * altHistory + hypsoAlt) /10;

    if (serialVerbosity == true) {
      //Serial.print("  | Altitude in M: "); Serial.println(metersold);
      Serial.print("  | Hypsometric, precise Altitude in M: "); Serial.println(altHistory);
    } else {
      Serial.println(hypsoAlt);
    }
  }
}

void readGPS() {
  if (Serial1.available() > 0) {
    if (gps.encode(Serial1.read())) {
      if (serialVerbosity) {
        displayInfo();
      }
    }
  } else {
    Serial.println("[!] WARN: NO GPS...");
  }
}

void displayInfo() {
  Serial.print(F("Location: ")); 
  if (gps.location.isValid())
  {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F(" "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.println();
}

void setup() {
  //Configure debug serial, GPSSerial and stuff
  
  Serial.begin(9600);
  Serial1.begin(GPS_INTERVAL);

  Serial.println("[i] CanSat OS Online.");

  //Configure buzzer
  
  pinMode(BUZZER_PIN, OUTPUT);

  //Configure diode

  pinMode(DIODE_PIN, OUTPUT);

  //Configure Altimeter:

  //is it good to retry until found?
  //yes - primary and secondary mission rely on this :)
  //think about this :)

  while (!baro.begin()) {
    Serial.println("[!] Altitude sensor missing! Retrying...");
  
    delay(300);
  }
  Serial.println("[*] Altitude sensor found! :)");
}

void loop() {
  //show ON status
  analogWrite(DIODE_PIN, HIGH);
  
  //get current time
  unsigned long currentMillis = millis();
  
  readSerial();

  if (currentMode == "STANDBY") {
    emitBuzz(currentMillis);
  }

  sampleAltitude(currentMillis);

  if (currentAltitude < ZONE_ACTIVE) {
    currentMode = "STANDBY";
  } else if (currentAltitude > ZONE_ACTIVE and currentAltitude < ZONE_SAMPLE) {
    currentMode = "ACTIVE";
  } else {
    currentMode = "SAMPLE";
  }

  //readGPS(); //replace to a mode ^^
}

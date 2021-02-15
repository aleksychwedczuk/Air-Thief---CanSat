n/////////////////////////////////////////////////////////////////
//                                                             //
//        AirOS - CanSat Operating System for Air Thief.       //
//             Written and badly debugged by @Alto.            //
//                                                             //
/////////////////////////////////////////////////////////////////

// ------------------------- Manual -------------------------- //
/*
    00 Preparing CanSatOS for Launch Day:
      1. Sample the current altitude and set it:
        * In code set DEBUGUSB to True
        * Open Serial Monitor (CTRL+SHIFT+M)
        * Send 'mode verbose' command.
        * Send 'get press' command. Note down avg pressure.
        * Set in code the GROUND_PRESS to that value.
        * Set DEBUGUSB back to False.
        * Also set current MAX_ALT and AIR_MIN
      2. Set nonmutable RFC elements:
        * Check current Lat & Lon.
        * Set the first decimal digits of Lat & Lon.
        * If there is overroll possible, adjust diff value
          by +5 or -5 and do so accordingly on the gstat
          recv module. Then adjust Lat & Lon by that
          same value. If overroll occurs, remote loc fix is
          still possible.
      3. Insert SD card:
        * Power off the CanSat.
        * Insert the preformatted 16GB SD card.
        * Consider sticking it in place with a tiny bit of
          nonconductive Glue-Tac.
        * Now, power on the CanSat.
        * If the Diode is persistantly YELLOW, use RST hole.
          This will reset the POCU.
        * Repeat until the Diode is BLUE. This means that
          the SD has been correctly configured. The SDlogger
          components are now ready to work correctly.
       4. Setup RTC:
         * Launch the SetupRTC.ino program.
         * Reset the board via the RST hole and compile & upload
           the code.
         * Ppen the Serial Monitor (CTRL+SHIFT+M) and just
           let the code run for about 3 seconds.
         * Upload AirOS back onto the board. The RTC is now ok.

    01 Status:
      Via Radio/Serial:
        00 Hex char (OVERALL STATUS):
          A: NULErr - Everything is in order. No errors.
                      This state is a miracle! :D
          B: SDCErr - The SD card is not configured. Refer to
                      point 3. in Manual
          C: RFCErr - Impossible to see on gstat, means radio
                      has failed.
          D: ALTErr - The altimeter has failed, or is
                      giving off-scale low/high. This means
                      the OS is using GPS data for altitude.
          E: GPSErr - The GPS is offline. This means that
                      precise position cannot be gathered.
                      The OS will not send data on the Can's
                      position at all.
          F: TELErr - Critical! Both Altimeter and GPS data
                      is not available. This means that until
                      altitude data is restored, the OS will
                      just base off of the launch time that
                      was set earlier (if available) and run
                      the pump for 1.5 minutes and then enter
                      Active mode to wait for instructions.
          G: RESErr - Reports a recent reset. This flag will
                      automatically become NULErr if no other
                      errors are occuring. It means that if
                      during flight the POCU restarts, we can
                      detect that event.
          H: FFAErr - Reports that the CanSat is in freefall.
                      This means that the rate of descent that
                      is calculated is greater than 8 m/s. It
                      could mean that the parachute has not
                      unfloded properly.
          The errors are assigned and take precedence in
          ascending order (A to H). This means that if we have
          no SD card and are in Freefall, we see H not B. This
          is good.

        01 Hex char (CURRENT MODE):
          A: STBY - The CanSat is standing by for launch.
          B: ACTI - The CanSat is in an active mode.
          C: SMPL - The CanSat is collecting a sample.

        02 Hex number (Sampling Duration Major):
          Shows the 10 in Base-10 positional notation for
          the duration of the function of the pump.
          So 00 to 90 or + if maxed - if mined.

        03 Hex number (Sampling Duration Minor):
          Shows the 1 in Base-10 positional notation for
          the duration of the function of the pump.
          So 0 to 9 or + if maxed - if mined.

        04 Hex number (Altitude Kilo):
          Shows the 1000 in Base-10 positional notation for
          the altitude of the CanSat.
          So 0000 to 9000 or + if maxed - if mined.

        05 Hex number (Altitude Hecto):
          Shows the 100 in Base-10 positional notation for
          the altitude of the CanSat.
          So 000 to 900 or + if maxed - if mined.

        06 Hex number (Altitude Deca):
          Shows the 10 in Base-10 positional notation for
          the altitude of the CanSat.
          So 00 to 90 or + if maxed - if mined.

        07 Hex number (Altitude Minor):
          Shows the 1 in Base-10 positional notation for
          the altitude of the CanSat.
          So 0 to 9 or + if maxed - if mined.

        08 Hex symbol (Speed in M/S from 0 to 35):
          Shows the speed [in m/s] from 0-9 to A-Z.
          Shows ++ if maxed.

        09 Hex symbol (Heading in 10ths of degrees):
          Shows the heading of the CanSat in 360/10 deg.
          Can be from 0-9 to A-Z. (36 chars, 0 to 360)

        10 Hex char (Lat or Lon stream select):
          Shows if the next 7 digits are for lat or lon pos,
          because we cycle between Lat and Lon pos in our
          broadcast. [Stream size considerations.]

        11 Hex digit to 18 Hex digit (Lat/Lon decimal):
          Holds the 8 digits (4 dp = 11.1 m, +/-sign) for a coord
          in Lat or Lon. Depending on the previous char, this
          will either be the Lat or Lon. Will be X if the GPS
          is offline. Cannot be off-scale.

        If any packet is E, this means the code that wrote to
        it on the POCU has screwed up as that is initial for the
        transmitted string. If it is + it is maxed out. If it
        is - it is mined out. If it is X then that sensor
        is not working correctly or offline. The packet length
        is 20: 19 Hex symbols and an empty char = 0.

      Via Diode:
        RED:    RFCErr or TELErr.
        YELLOW: SDCErr, ALTErr or GPSErr.
        BLUE:   NULErr (No errors! :D)
        GREEN:  When CanSat has done Sample mode. This
                takes precedence over any errors. This
                also means that the Buzzer will be on...
                Please turn off Your ears now.
                sudo pactl list ear |egrep -e 'Ear|Mute'
        When the CanSat lands, Green should be shown.

    02 POCU Pins:
      RX0 -> GPS RX
      TX1 -> GPS TX

      SCL -> ALT SCL
      SDA -> ALT SDA

      A1  -> BUZZER SIG

      A2  -> MOSFET

      A3     R Diode Channel
      A4  -> G Diode Channel
      A5     B Diode Channel

    03 Panic-proof quick facts:
      00:
        The onboard GPS has a precision of about 10 m,
        which is also what is broadcast back to the gstat.
      01:
        If the SD card is not working/RF is not working, check
        if they do not communicate at the same time as HIGH CS
        has to be set on the nonused slave.
      02:
        If an onboard error occurs, the LoRa will enter panic
        mode and listen for an incoming command packet every 1s.
        This can be utilised so that, when we see that something
        has screwed itself, sending a packet: 'CMD <command>'
        will allow us to issue debug statements. If the error
        returns to NULErr, the LoRa will exit RX mode.
      03:
        Altitude delta + means going up, - means going down.
        This means that delta < -2 is falling at a rate good
        enough to initiate sampling systems. That is because the
        sensor can sometimes produce variance and this is the
        best, simplest most shovelological way of doing it.

*/

// ------------------------ Includes ------------------------- //
#include <Arduino.h>            //  For awesome STUFF
#include <Adafruit_MPL3115A2.h> //  For ALTIMETER
#include <TinyGPS++.h>          //  For GPS
#include <Wire.h>               //  For SCL SDA devices
#include <SPI.h>                //  For SPI Interface
#include <RH_RF95.h>            //  For RadioHead RF9X
#include <SD.h>                 //  For the SD card
//#include "RTClib.h"             //  RTC Library for LOGTStamps

// ------------------------- Config -------------------------- //
//Buzzer
#define BUZZER_PIN A1
#define BUZZER_INTERVAL 3

//Altitude Pressure & Temperature Sensor [on SDA, SCL]
#define ALTPT_INTERVAL 1000

//RGB Diode Pins
#define DIODE_PIN_R A3
#define DIODE_PIN_G A4
#define DIODE_PIN_B A5

//GPS [on RX, TX]
#define GPS_INTERVAL 1000
#define GPS_BAUDRATE 9600
#define LAT_DEC 5 //  !!! PREPARE ON L-DAY !!!
#define LON_DEC 2 //  !!! PREPARE ON L-DAY !!!
//  Show the 10 in Base-10 of Lat and Lon

//SDStore config
#define SD_PIN 10
#define SD_INTERVAL 1000

//RFComms config
#define RFC_INTERVAL 1000

#define RFC_CS 8
#define RFC_RST 4
#define RFC_INT 3

#define RFC_FREQ 433.0 //  Regulations suggest this FREQ.

//Avionics Config
#define GROUND_PRESS 1090.3 //  !!! PREPARE ON L-DAY !!!
                             //  Shows the ground-level HPA
#define MAX_ALT      3000    //  Max alt reachable in M
#define AIR_MIN      500     //  Cut pump @ this alt (microbeee)

//Debug Channel on Serial
#define DEBUGUSB true

// ---------------------- Instantiates ----------------------- //
RH_RF95 rfc(RFC_CS, RFC_INT);                   //  RFM9X
TinyGPSPlus gps;                                //  GPS
Adafruit_MPL3115A2 baro = Adafruit_MPL3115A2(); //  Altimeter
//RTC_PCF8523 rtc;                              //  Ze timepiese

//SDCard Tech [follow docs] (AKA copypasta)
Sd2Card card;
SdVolume volume;
SdFile root;

// ------------------------- Globals ------------------------- //
String currentMode   = "ACTI";     //  Current OS mode
String inboundString = "";         //  Used for Serial debug
String lastError     = "NULErr";   //  Refer to Manual, hehe

float currentAltitude = 0; //RFC it is anyhow int, with 1m prec
float lastdatAltitude = 0;  //This is last check alt, 1s/sample
                           //thus we can get fall speed. This
                           //is extra simple, however after
                           //various considerations this is the
                           //least failure-prone way of getting
                           //that data. altData stays for backup
float delta1sAltitude = 0; //shows basically fall rate in m/s
                           //yes, they have same length intent.

float altHistory      = 0;

float altData[30];         //stores 30 last altitudes

//RGB Colorstates
byte RColor = 255;
byte GColor = 0;
byte BColor = 0;

//GPS Tech
float Lat    = -999; //shows nonfunctional when set -999
float Lon    = -999; //also
float GPSAlt = 0;    //redundancy router hehe
float GPSVel = 0;    //!!!
float GPSAng = 0;    //!!!

//Timing
unsigned long altptMillis   = 0; //for values for timing
unsigned long buzzerMillis  = 0; //for values for timing
unsigned long logMillis     = 0; //for values for timing
unsigned long radioMillis   = 0; //for values for timing
unsigned long gpsMillis     = 0; //for values for timing

//State-dependant
int    buzzerState     = LOW;   // bzzzzzZZZZzz
int    sampleTime      = 0;     // from 0 s to 120 s anyway
String LogSuffix       = "";    // set if RTC is ok to not
                                // overwrite a logfile acc.

//Flags
bool hasSample       = false;    //  Is sample there?
bool serialVerbosity = false;    //  Toggles verbose mode
bool altOnline       = false;    //  Is altimeter OKOK?
bool SDOnline        = false;    //  Is SDFuxxor runnin'??????
bool radioOnline     = false;    //  Is the radio up?
bool LatSelected     = true;     //  Send lat or lon? (chckMAN)
bool RTCOk           = false;    //  IS RTC OK...?

// ------------------------ Functions ------------------------ //
//Reads the Serial, enables good debuging.
//veryE gooday tastes gooded. **
void readSerial() {
  if (Serial.available() > 0) {
    inboundString = Serial.readString();

    Serial.print("[>] Command recieved: ");
    Serial.println(inboundString);

    if (inboundString == "errorec") {
      if (lastError = "NULErr") {
        Serial.println("[i] No errors.");
      } else {
        Serial.print("[!] Last error was: ");
        Serial.println(lastError);
      }
    }

    if (inboundString == "mode standby") {
      currentMode = "STBY";

      Serial.println("[*] Manually entering STBY.");
    }

    if (inboundString == "mode active") {
      currentMode = "ACTI";

      Serial.println("[*] Manually entering ACTI.");
    }

    if (inboundString == "mode sample") {
      currentMode = "SMPL";

      Serial.println("[*] Manually entering SMPL.");
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

//Controls the buzzer
void emitBuzz() { //emytt buzZZzzz
  if (millis() - buzzerMillis >= BUZZER_INTERVAL) {
    buzzerMillis = millis();

    if (buzzerState == LOW) {
      buzzerState = HIGH;
    } else {
      buzzerState = LOW;
    }

    digitalWrite(BUZZER_PIN, buzzerState);
  }
}

//Init altimeter (async)
void initAltitude() {
  if (!baro.begin()) {
    Serial.println("[!] Ah, shit... Baro failed.");

    lastError = "ALTErr";

    logData("FAIL BARO");
  } else {
    Serial.println("[!] Baro ONLINE.");
    altOnline = true;

    logData("OK BARO");
  }
}

//Controls altitude sampling
void sampleAltitude() {
  if (!altOnline) {
    initAltitude();

    return;
  }

  if (millis() - altptMillis >= ALTPT_INTERVAL) {
    altptMillis = millis();

    //here we want to addiitonally record the current readings.
    //surrogated with output for now

    float pascal = baro.getPressure();
    float temperature = baro.getTemperature();
    logData("PRESS " + String(pascal) + " TEMP " + \
            String(temperature));

    if (serialVerbosity == true) {
      Serial.println("[C] FlightConditions update:");
      Serial.print("  | Pressure in HPA: ");
      Serial.println(pascal / 100);
      Serial.print("  | Temperature in C: ");
      Serial.println(temperature);
    }

    float hypsoAlt = ((pow(GROUND_PRESS / (pascal / 100),
                           1.0 / 5.257) - 1) *
                      (temperature + 273.15)) / 0.0065;

    currentAltitude = hypsoAlt;

    //GPSSave us plz (if need be)
    if (currentAltitude > MAX_ALT || currentAltitude < -200) {
      if (GPSAlt) //GP-Salt? NaCl? Native Client? RIP Chrome Apps
        currentAltitude = GPSAlt; //so yee we are saveddddd

      logData("WARN ALT TELE FUCKED"); //politely remind team
    }
    
    delta1sAltitude = currentAltitude - lastdatAltitude;
                    //^ lower if we fell lower down towards AZL
    lastdatAltitude = hypsoAlt; //and set current again
                    //yes using 2 vars is possible but calcrule

    logData("ALT " + String(currentAltitude) + " DELTA " + \
    String(delta1sAltitude));

    for (int i = 0; i < 30 - 1; i++) {
      altData[i] = altData[i + 1];
    }
    altData[29] = currentAltitude;

//    Serial.println("[ALTCOMPUTE]: ");
//    for (int i = 0; i < 30; i++) {
//      Serial.print(altData[i]); Serial.print(", ");
//    }
//    Serial.println();

//    altHistory = (9 * altHistory + hypsoAlt) / 10;//boomer time
    // ^^ debug only do not use in production !!!

    if (serialVerbosity == true) {
      Serial.print("  | Hypsometric, precise Altitude in M: ");
      //Serial.println(altHistory);
      Serial.println(hypsoAlt);
    } else {
      Serial.println(hypsoAlt);
    }
  }
}

//Reads from RX0 and TX1 the GPS info
void readGPS() {
  if (Serial1.available() > 0) {
    if (gps.encode(Serial1.read())) {
      Serial.println("[i] GPSOK");
      
      if (serialVerbosity) {
        displayInfo();
      }

      if (gps.location.isValid()) {
        Lat = gps.location.lat();
        Lon = gps.location.lng();

        if (gps.speed.isUpdated()) { //add info on velo
          GPSVel = gps.speed.mps();
        }
        if (gps.course.isUpdated()) { //add info on course
          GPSAng = gps.course.deg();
        }
        if (gps.altitude.isUpdated()) { //add info on bakAlt
          GPSAlt = gps.altitude.meters(); //yee
        } else {
          GPSAlt = -1; //unf no cigar on dis
        }
      } else {
        if (!altOnline) {
          Serial.println("[!] CRIT! No TELEMETRY! :("); //*uuu!*
          lastError = "TELErr"; //telemetry.isDead() == true

          logData("FAIL TELEMETRY");
        } else {
          //Serial.println("[*] SLIWARN: GPS DATA BAD!");
          //lastError = "GPSErr";
          //logData("TEMPFAIL GPS");
        }
      }
    }
  } else {
    //Serial.println("[!] WARN: NO GPS...");

    //lastError = "GPSErr";
  }
}

void displayInfo() {
  Serial.print(F("Location: "));
  if (gps.location.isValid()) {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
  }
  else {
    Serial.print(F("INVALID"));
  }

  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid()) {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  }
  else {
    Serial.print(F("INVALID"));
  }

  Serial.print(F(" "));
  if (gps.time.isValid()) {
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
  else {
    Serial.print(F("INVALID"));
  }

  Serial.println();
}

//SDCard Management
void initCard() {
  if (!card.init(SPI_HALF_SPEED, SD_PIN)) {
    Serial.println("[!] Failed to INIT card. Will try again.");

    lastError = "SDCErr";
  } else {
    Serial.println("[*] INIT Success!");

    SDOnline = true;
  }
}

void logData(String dataline) {
  digitalWrite(RFC_CS, HIGH);
  if (!SDOnline) {
    initCard();

    return;
  }

  if (!SD.begin(SD_PIN)) {
    Serial.println("[i] Card failed or is not present");

    SDOnline = false; //reset to fix next 1s interval

    return;
  }

  File LOGFILE = SD.open("AirOS.log", FILE_WRITE); // NOTepad

  if (LOGFILE) {
    LOGFILE.println(dataline);
    LOGFILE.close();

    Serial.print("[i] Logging: ");
    Serial.println(dataline);
  }
  else {
    Serial.println("error opening datalog.txt");

    lastError = "SDCErr";

    SDOnline = false; //logging error can be fixed w/ reboot
  }

  digitalWrite(RFC_CS, LOW);
}

void initRadio() { //has delay because it is a setup

  Serial.println("[i] LoRa radio init...");

  //reset -> see docs
  digitalWrite(RFC_RST, LOW);
  delay(10);
  digitalWrite(RFC_RST, HIGH);
  delay(10);

  if (!rfc.init()) {
    lastError = "RFCErr";
    radioOnline = false;

    logData("FAIL LORA CRIT");
    return;

  }
  Serial.println("[i] LoRa radio init OK!");

  if (!rfc.setFrequency(RFC_FREQ)) {
    Serial.println("setFrequency failed");
    logData("FAIL LORA SETFREQ");
    radioOnline = false;
    Serial.println("[i] Frequency failed to set.");
    return;
  }
  Serial.print("Set Freq to: "); Serial.println(RFC_FREQ);

  rfc.setTxPower(23, false); //ramp up transmitter power

  radioOnline = true;
  Serial.println("[i] RADIO OK :D");
  logData("OK LORA");

  digitalWrite(8, HIGH); //flush SPI again so SD is ok
}

void transmitData() {
  if (!radioOnline) {
    initRadio();

    return;
  }

  Serial.println("Transmitting data...");

  char RFPacket[20] = "EEEEEEEEEEEEEEEEEEE"; //E -> notset
  RFPacket[19] = 0; //and set terminator up
  //not the robot but like null char

  //here is data addition
  //mode
  if (lastError == "NULErr") {RFPacket[0] = 'A';}
  if (lastError == "SDCErr") {RFPacket[0] = 'B';}
  if (lastError == "RFCErr") {RFPacket[0] = 'C';}
  if (lastError == "ALTErr") {RFPacket[0] = 'D';}
  if (lastError == "GPSErr") {RFPacket[0] = 'E';}
  if (lastError == "TELErr") {RFPacket[0] = 'F';}
  if (lastError == "RESErr") {RFPacket[0] = 'G';}
  if (lastError == "FFAErr") {RFPacket[0] = 'H';}
  //sampling
  if (currentMode == "STBY") {RFPacket[1] = 'A';}
  if (currentMode == "ACTI") {RFPacket[1] = 'B';}
  if (currentMode == "SMPL") {RFPacket[1] = 'C';}
  //sample time
  if (sampleTime > 99) {
    RFPacket[2] = '+'; RFPacket[3] = '+';
  } else if (sampleTime < 0) {
    RFPacket[2] = '-'; RFPacket[3] = '-';
  } else {
    RFPacket[2] = 48 + (int) (abs(sampleTime) / 10) % 10;
    RFPacket[3] = 48 + (int) (abs(sampleTime) / 1)  % 10;
  }
  //altitude value
  if (currentAltitude > 9999) {
    RFPacket[4] = '+'; RFPacket[5] = '+';
    RFPacket[6] = '+'; RFPacket[7] = '+';
  } else if (currentAltitude < 0) {
    RFPacket[4] = '-'; RFPacket[5] = '-';
    RFPacket[6] = '-'; RFPacket[7] = '-';
  } else {
    RFPacket[4] = 48 + (int) (abs(currentAltitude) / 1000) % 10;
    RFPacket[5] = 48 + (int) (abs(currentAltitude) / 100)  % 10;
    RFPacket[6] = 48 + (int) (abs(currentAltitude) / 10)   % 10;
    RFPacket[7] = 48 + (int) (abs(currentAltitude) / 1)    % 10;
  }
  //hex symbol for speed
  if (GPSVel < 10) {
    itoa((int) sampleTime, RFPacket+8, 10); //1pos int
  } else if (GPSVel < 36) {
    RFPacket[8] = 55 + (int) GPSVel; //set to char, A = 65 ASCII
  } else {
    RFPacket[8] = '+'; //V has maxed out (oshiit feel de sped)
  }
  //hex symbol for heading
  if (GPSAng < 100) {
    itoa((int) (GPSAng / 10), RFPacket + 9, 10); //set by 10
  } else {
    RFPacket[9] = 55 + (int) GPSAng / 10; //set to char EQUIV
  }
  //send latselect char
  if (LatSelected) { //so is it de lat or long dissa tiem?
    RFPacket[10] = 'Y'; //fuxxin confusion 100
    if (Lat < 0) {
      RFPacket[11] = '-'; //is neg
    } else {
      RFPacket[11] = '+'; //is pos
    }

    RFPacket[12] = 48 + (int) (abs(Lat) / 100)    % 10;
    RFPacket[13] = 48 + (int) (abs(Lat) / 10)     % 10;
    RFPacket[14] = 48 + (int) (abs(Lat) / 1)      % 10;
    RFPacket[15] = 48 + (int) (abs(Lat) * 10)     % 10;
    RFPacket[16] = 48 + (int) (abs(Lat) * 100)    % 10;
    RFPacket[17] = 48 + (int) (abs(Lat) * 1000)   % 10;
    RFPacket[18] = 48 + (int) (abs(Lat) * 10000)  % 10;
  } else {
    RFPacket[10] = 'X'; //equally evil
    if (Lon < 0) {
      RFPacket[11] = '-'; //is neg
    } else {
      RFPacket[11] = '+'; //is pos
    }

    //itoa replaced with this abomination - easier to debug
    //for loops are dangerous :D
    RFPacket[12] = 48 + (int) (abs(Lon) / 100)    % 10;
    RFPacket[13] = 48 + (int) (abs(Lon) / 10)     % 10;
    RFPacket[14] = 48 + (int) (abs(Lon) / 1)      % 10;
    RFPacket[15] = 48 + (int) (abs(Lon) * 10)     % 10;
    RFPacket[16] = 48 + (int) (abs(Lon) * 100)    % 10;
    RFPacket[17] = 48 + (int) (abs(Lon) * 1000)   % 10;
    RFPacket[18] = 48 + (int) (abs(Lon) * 10000)  % 10;
  }

  //!!!! sample


  Serial.print("Sending ");
  for (int i = 0; i < 20; i++) {
    Serial.print((char) RFPacket[i]);
  }
  Serial.println();

  //last!!!
  LatSelected = !LatSelected; //and toggle

  //Serial.print("Sending "); Serial.println(RFPacket);


  //issue zone
  digitalWrite(8, LOW); //let radio work

  rfc.send((uint8_t *)RFPacket, 20);
  delay(10); //as in docs, dutycycle is ok anyway
  rfc.waitPacketSent(1000); //it takes uno momento hfully
                            //max timeout is 1s which is ok
                            //we send 20.1 chars/s so perfecto
                            //total dutycycle is ok :D

  digitalWrite(8, HIGH); //let SD talk

  Serial.println("Packet sent!");
  logData("LORA SEND");
}

//Start the RTC for creating the logfiles
//void initRTC() {
//  if (!rtc.begin()) {
//    RTCOk = false;
//    return; //failed to init, no matter screwit
//  }
//
//  rtc.start(); //set STOP bit to 0
//}

void setup() {
  //maths stuff
  for (int i = 0; i < 30; i++) {
    altData[i] = -999; //reset all
  }
  
  //Serial INIT
  Serial.begin(9600);
  Serial1.begin(GPS_BAUDRATE);

  Serial.println("[i] AirOS Online.");

  //Configure All
  pinMode(BUZZER_PIN,  OUTPUT);
  pinMode(DIODE_PIN_R, OUTPUT);
  pinMode(DIODE_PIN_G, OUTPUT);
  pinMode(DIODE_PIN_B, OUTPUT);
  pinMode(RFC_RST,     OUTPUT);

  initRadio();    //uses delay, which is ok
  initAltitude(); //altimeter tech
  initCard();     //and card
  //
  //  DateTime now = rtc.now();
  //  if (RTCOk) {
  //    LogSuffix += now.year();  LogSuffix += "/";
  //    LogSuffix += now.month(); LogSuffix += "/";
  //    LogSuffix += now.day();   LogSuffix += " ";
  //    LogSuffix += now.hour(); LogSuffix += ":";
  //    LogSuffix += now.minute(); LogSuffix += ".";
  //    LogSuffix += now.second();
  //
  //    Serial.print("[i] Log suffix as: ");
  //    Serial.println(LogSuffix);
  //  }
  digitalWrite(RFC_CS, HIGH); //  stfu, RAD.io PLZ SHUT UPPPPP


  logData("AirOS UP");
}

void setModes() { //setMode is reserved FYI !!
  if (currentAltitude < 5) { //minimum sensor alt for start !!!
    currentMode = "STBY";
    logData("MODE STBY");
    return;
  }
  if (currentAltitude < AIR_MIN) {
    currentMode = "ACTI"; //always in active anyhow
    logData("MODE ACTI");
    return;
  }
  if (delta1sAltitude < -2) {
    currentMode = "SMPL"; //means we are falling fast enough
                          //descent rate exp >> 2 m/s
    logData("MODE SMPL");
  } else {
    currentMode = "ACTI";

    //descent rate is still too low
    logData("MODE ACTI APO"); //probably apoapsis? idk man
  }
}

void loop() {
  //Push correct colors -> number efficacy RGB (byte hehe)
  analogWrite(DIODE_PIN_R, HIGH);
  analogWrite(DIODE_PIN_G, HIGH);
  analogWrite(DIODE_PIN_B, HIGH);

  if (DEBUGUSB) { //init readSerial call to talk to CanPOCU
    readSerial();
  }

  //dumped 4 now !!! buzzzzzzzzaa
  //emitBuzz();
  sampleAltitude(); // sample

  if (currentMode == "STBY") {
    delay(5000); //force-sleep POCU for 5sec for power efficacy
  } else {
    readGPS(); //only not in STBY do we GPS
  }
  
  if (millis() - logMillis >= SD_INTERVAL) {
    logMillis = millis();


    //////DEBUGGGGG//////
    //GPSAng = int (GPSAng + 1) % 360;
    if (currentMode == "SMPL") {
      sampleTime = int (sampleTime + 1) % 100;

      logData("SMPL GET");
    }
  }
  if (millis() - radioMillis >= RFC_INTERVAL) {
    setModes();
    
    transmitData();

    radioMillis = millis();
  }

  if (currentMode == "SMPL") {hasSample = true;}

  if (hasSample && currentMode == "STBY") {
    emitBuzz();
    /// !!! rework this will not work
    /// !!! we need to delay 5000 using buzzer as timing :D
    /// basically fancy-sleep (snore?) for 5000 millis bikos
    /// elec power and stuff
    for (int i = 0; i < 2500; i++) {
      analogWrite(BUZZER_PIN, HIGH);
      delay(1);
      analogWrite(BUZZER_PIN, LOW);
      delay(1);
    }
  }

  lastError = "NULErr"; //clear damn errors @ end of ze loopak
}

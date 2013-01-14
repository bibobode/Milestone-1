#include <Wire.h>
#include "IntersemaBaro.h"
#include "Adafruit_GPS.h"
#include "SimpleTimer.h"
//#include <NewSoftSerial.h>
#include <SoftwareSerial.h>  //Include the NewSoftSerial library to send serial commands to the cellular module.
#include <string.h>         //Used for string manipulations

SoftwareSerial cell(2,3);  //Create a 'fake' serial port. Pin 2 is the Rx pin, pin 3 is the Tx pin.
SoftwareSerial gps(4, 5);
Adafruit_GPS GPS(&gps);

SimpleTimer timer;
Intersema::BaroPressure_MS5607B altimeter(true);

// different commands to set the update rate from once a second (1 Hz) to 10 times a second (10Hz)
#define PMTK_SET_NMEA_UPDATE_1HZ  "$PMTK220,1000*1F"
#define PMTK_SET_NMEA_UPDATE_5HZ  "$PMTK220,200*2C"
#define PMTK_SET_NMEA_UPDATE_10HZ "$PMTK220,100*2F"
// turn on only the second sentence (GPRMC)
#define PMTK_SET_NMEA_OUTPUT_RMCONLY "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"
// turn on ALL THE DATA
#define PMTK_SET_NMEA_OUTPUT_ALLDATA "$PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28"


//AT+CNMI=3,3,0,0


void setup() {
 
  // used to reset the whole module 
  digitalWrite(13, HIGH);
  pinMode(13, OUTPUT);

  Serial.begin(9600);
  GPS.begin(9600);
  cell.begin(9600);
  
  timer.setInterval(1, readGPS);


  // uncomment this line to turn on only the "minimum recommended" data for high update rates!
  //gps.println(PMTK_SET_NMEA_OUTPUT_RMCONLY);

  Serial.println("Initiating GPS...");
  // uncomment this line to turn on all the available data - for 9600 baud you'll want 1 Hz rate
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_ALLDATA);

  // 1 Hz update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  // 5 Hz update rate- for 9600 baud you'll have to set the output to RMC only (see above)
  //gps.println(PMTK_SET_NMEA_UPDATE_5HZ);
  // 10 Hz update rate - for 9600 baud you'll have to set the output to RMC only (see above)
  //gps.println(PMTK_SET_NMEA_UPDATE_10HZ);

  Serial.println("Initiating altimeter...");
  altimeter.init();

  Serial.println("Initiating cell...");
  
  Serial.println("Initiating ultrasonic sensor...");
  pinMode(5, INPUT);
}


char incomingChar=0;      //Will hold the incoming character from the Serial Port.

//String noSMSMessage = "+CMS ERROR: 321";
String regStatus = "+SIND: 11";
String regStatusBegun = "+SIND: 7";
String lostSigStatus1 = "+SIND: 8";
String lostSigStatus2 = "+SIND: 1";
String readyStatus = "+SIND: 4";
String smsRead = "+CMT: \"";

String sindStatus = "+SIND:";
String cmgsStatus = "+CMGS: ";
String cmsErrorStatus = "+CMS ERROR: ";

#define cReturn 0x0D  // carriage return
#define ctrlZ 0x1A

boolean registered = false;
boolean ready = false;

boolean smsAvailableNext = false;
boolean messageAvailable = false;
boolean smsSentMsgAvailableNext = false;
boolean waitingForSIND8Response = false;
boolean searchNextBand = true;

int smsStatus = 0;  // 0 is available, 1 is busy setting up sms, 2 is busy sending sms message, 3 is waiting for response

boolean allow = false;
boolean triedRestarting = false;

String sms = "";
String message = "";

unsigned long smsDelay = 1000;
unsigned long prevTime = 0;

int currentBand = 2;

unsigned long balloonDiamInches;


void loop() {

  // --------------------------------------------------------------------------------------------------
  // --------------------------------------------------------------------------------------------------
  // GPS CODE
  // --------------------------------------------------------------------------------------------------
  // --------------------------------------------------------------------------------------------------
  
  float decimalLatitude = -1;
  float decimalLongitude = -1;
  
  timer.run();
  if (GPS.newNMEAreceived())
  {
    GPS.parse(GPS.lastNMEA());

    int degreesLatitude = (int) (GPS.latitude/100);
    int degreesLongitude = (int) (GPS.longitude/100);

    decimalLatitude = (float) ((((GPS.latitude/100)-degreesLatitude)*100)/60)+degreesLatitude;
    decimalLongitude = (float) ((((GPS.longitude/100)-degreesLongitude)*100)/60)+degreesLongitude;

    if (GPS.lat == 'S' || GPS.lat == 'W') { 
      decimalLatitude *= -1; 
    }
    if (GPS.lon == 'S' || GPS.lon == 'W') { 
      decimalLongitude *= -1; 
    }

    //    Serial.print("Location: ");
    //    Serial.print(decLat, 6);
    //    Serial.print(", ");
    //    Serial.println(decLon, 6);
    //    
    //    Serial.print("Time: "); Serial.print((int)GPS.hour); Serial.print(":");
    //    Serial.print((int)GPS.minute); Serial.print(":");
    //    Serial.println((int)GPS.seconds);
    //    
    //    println("Altitude", GPS.altitude);
    //    println("Speed", GPS.speed);
    //    println("Angle", GPS.angle);
    //    println("Magvariation", GPS.magvariation);
    //    println("Fix", GPS.fix);
    //    println("Fix Quality", GPS.fixquality);
    //    println("Satellites", GPS.satellites);
    //    println("HDOP", GPS.HDOP);
    //    println("GeoIDHeight", GPS.geoidheight);
    //    Serial.println("");
  }

  int altitudeInM= altimeter.getHeightCentiMeters()/100;
  int altimeterTemperature = altimeter.getTemperature();

  //  Serial.print("Centimeters: ");
  //  Serial.print((float)(altitudeInCm));
  //  Serial.print(", Feet: ");
  //  Serial.println((float)(altitudeInCm) / 30.48);
  //  Serial.print("Temperature: ");
  //  Serial.println(altimeterTemperature);
  //  delay(400);



  // --------------------------------------------------------------------------------------------------
  // --------------------------------------------------------------------------------------------------
  // ULTRASONIC CODE
  // --------------------------------------------------------------------------------------------------
  // --------------------------------------------------------------------------------------------------

  balloonDiamInches = (pulseIn(3, HIGH)/147);


  // --------------------------------------------------------------------------------------------------
  // --------------------------------------------------------------------------------------------------
  // CELL CODE
  // --------------------------------------------------------------------------------------------------
  // --------------------------------------------------------------------------------------------------

  if (millis() > 60000*2) {
    Serial.println("Resetting cell after 2 minutes...");
    resetCell();
    return;
  }

  if (!registered || !ready) {
    smsStatus = 0;
    //smsAvailableNext = false;
    smsSentMsgAvailableNext = false;
  }

  if (cell.available() > 0) {
    incomingChar = cell.read();    //Get the character from the cellular serial port.
    if (incomingChar == cReturn || incomingChar == '\n')
    {
      if (message.length() > 0) {
        Serial.print("Message from cell: ");
        Serial.println(message);
        messageAvailable = true;
      }
    }
    else {
      message += incomingChar;
    }
  }

  if (message.length() > 0 && messageAvailable) {

    if (message.indexOf(sindStatus) >= 0) {

      // +SIND: message received

      if (message.indexOf(regStatus) >= 0) {
        Serial.println("Registered to network");
        registered = true;
        triedRestarting = false;
        searchNextBand = true;
        waitingForSIND8Response = false;
        sendMessage("Ready and just registered.");
      }
      else if (message.indexOf(readyStatus) >= 0) {
        Serial.println("Cell is ready");
        ready = true;
        prevTime = millis();
        sendMessage("Registered and now ready.");
      }
      else if (message.indexOf(lostSigStatus1) >= 0) {
        Serial.println("Lost connection");
        registered = false;
        if (waitingForSIND8Response) {
          searchNextBand = true;
          waitingForSIND8Response = false;
        }
      }
      else if (message.indexOf(lostSigStatus2) >= 0) {
        //Serial.println("Starting up...");
        ready = false;
        registered = false;
      }
    }

    else {
      if (smsAvailableNext) {
        sms = message;
        smsAvailableNext = false;
        //cell.println("AT+CMGD=1,4");
        
        sendMessage("Received");
      }
      else if (message.indexOf(smsRead) >= 0) {
        smsAvailableNext = true;
      }

      if (smsSentMsgAvailableNext && message.indexOf("OK") >= 0) {
        smsStatus = 0;
        smsSentMsgAvailableNext = false;
      }
      else if (message.indexOf(cmgsStatus) >= 0 && smsStatus != 0) {
        smsSentMsgAvailableNext = true;
      }
      else if (message.indexOf(cmsErrorStatus) >= 0 && smsStatus != 0) {
        smsStatus = 0;
        smsSentMsgAvailableNext = false;
      }
    }
  }

  if (!registered && ready)
  {
    //device is ready but we're not registered
    if (!triedRestarting) {
      waitForResponse(30000);
      if (allow) {
        resetCell();
        triedRestarting = true; 
      }
    }
    else {
      searchBands();
    }
  }
  else if (registered && ready)
  {
    String msgToSend = "Lat: "+(decimalLatitude, 6);
    msgToSend += " Long: "+(decimalLongitude, 6);
    msgToSend += " Alt: "+altitudeInM;
    msgToSend += " Freefall: ";
    msgToSend += " Speed: "+(GPS.speed, 6);
    msgToSend += " Diamtr: "+balloonDiamInches;
    
    
    msgToSend += " TempIn: ";
    msgToSend += " TempOut: ";
    msgToSend += " Angle: "+(GPS.angle, 6);
    msgToSend += " AltTemp: "+altimeterTemperature;
    msgToSend += " GPSAlt: "+(GPS.altitude, 6);
    msgToSend += " X: ";
    msgToSend += " Y: ";
    msgToSend += " Z: ";
    
    
    
    //    Serial.print("Time: "); Serial.print((int)GPS.hour); Serial.print(":");
    //    Serial.print((int)GPS.minute); Serial.print(":");
    //    Serial.println((int)GPS.seconds);
    //    
    //    println("Altitude", GPS.altitude);
    //    println("Speed", GPS.speed);
    //    println("Angle", GPS.angle);
    //    println("Magvariation", GPS.magvariation);
    //    println("Fix", GPS.fix);
    //    println("Fix Quality", GPS.fixquality);
    //    println("Satellites", GPS.satellites);
    //    println("HDOP", GPS.HDOP);
    //    println("GeoIDHeight", GPS.geoidheight);
    //    Serial.println("");
    
    sendMessage(msgToSend);
  }

  //If a character is coming from the terminal to the Arduino...
  if (Serial.available() > 0)
  {
    Serial.println("Received character");
    incomingChar=Serial.read();  //Get the character coming from the terminal

    if( incomingChar == '~' ) // If it’s a tilde…
      incomingChar = 0x0D;      // ...convert to a carriage return  
    else if( incomingChar == '^' ) // If it’s an up caret…
      incomingChar = 0x1A;    // ...convert to ctrl-Z  
    cell.print(incomingChar);    //Send the character to the cellular module.

    Serial.print(incomingChar); // Echo it back to the terminal
  }

  if (messageAvailable) {
    resetMessage();
  }





}

void readGPS()
{
  GPS.read();
}


void postSetup() {
  cell.println("AT+CMGF=1");
  cell.println("AT+CMGD=1,4");
  //cell.println("AT+CNMI=3,3,0,0");
}

void resetMessage() {
  message = "";
  messageAvailable = false;
}

void sendMessage(String text) {

  if (!ready || !registered) return;

  if (smsStatus == 0) {

    waitForResponse(smsDelay);
    if (allow) {
      postSetup();
      smsStatus = 1;
    }
  }
  else if (smsStatus == 1) {

    waitForResponse(smsDelay);
    if (allow) {
      smsStatus = 2;
      //cell.println("AT+CMGS=\"5197817862\"");  // now send message...
      //cell.println("AT+CMGS=\"3233910406\"");  // now send message...
      cell.println("AT+CMGS=\"5197812700\"");  // now send message...
    }
  }
  else if (smsStatus == 2) {

    waitForResponse(smsDelay);
    if (allow) {
      smsStatus = 3;

      cell.print(text);   // our message to send
      cell.write(26);  // ASCII equivalent of Ctrl-Z

      Serial.print ("Sending message: ");
      Serial.println(text);
    }
  }
  else if (smsStatus == 3) {

    waitForResponse(30000);
    if (allow) {
      // sms timeout
      Serial.println ("SMS timeout!");
      resetCell();
    } 
  }
}

void resetCell () {
  Serial.println("Resetting...");
  //cell.println("AT+CFUN=0,1");
  //cell.println("AT+CFUN=1,1");
  registered = false;
  ready = false;
  
  digitalWrite(13, LOW);
  //pinMode(4, OUTPUT);
  //analogWrite(13, 0);
  //digitalWrite(13, LOW);
  //delay(2000);
  //digitalWrite(13, HIGH);
  
  //Serial.println("Done resetting");
}

void searchBands () {
  if (!searchNextBand) return;
  waitingForSIND8Response = true;

  currentBand++;
  if (currentBand > 10) currentBand = 0;
  String bandNum = String(currentBand);
  String bandComm = "AT+SBAND=";
  bandComm += bandNum;
  cell.println(bandComm);

  Serial.println(bandComm);

  resetCell();
}

void waitForResponse (unsigned long responseDelay) {

  unsigned long curTime = millis();
  if (curTime < prevTime) {
    // overflow occurred
    prevTime = 0;
    Serial.println ("Overflow occurred");
  }

  if (curTime-prevTime < responseDelay) {
    allow = false;
    return;  
  }
  prevTime = curTime;
  allow = true;
}



//void println (String title, float value)
//{
//  Serial.print(title);
//  Serial.print(": ");
//  Serial.println(value, 4);
//}



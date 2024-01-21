#include <MPU9250.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

#include <Wire.h>
#include <Math.h>

MPU9250 IMU (Wire, 0x68);

// Code for Arduino Mega
static const int RXPin = 11, TXPin = 10;
static const uint32_t GPSBaud = 9600;

double latitude = 0;
double longitude = 0;

float x,y,z;
float azimuth;

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

void setup() {
  Serial.begin(115200);
  ss.begin(GPSBaud);
  IMU.begin();

}

void loop() {

  // Data from GPS
  if (ss.available() > 0){
    if (gps.encode(ss.read())){
      if (gps.location.isValid())
      {
        Serial.print(gps.location.lat(), 6);
        Serial.print(F(","));
        Serial.print(gps.location.lng(), 6);
        Serial.print(F(","));
        Serial.print(gps.altitude.meters(), 2);
        Serial.print("      ");
        // Data from Magnetometer
        IMU.readSensor();
        x = IMU.getMagX_uT();
        y = IMU.getMagY_uT();
        z = IMU.getMagZ_uT();
        azimuth = atan2(y,x);
        
        if(azimuth < 0){
          Serial.print(azimuth*180.0/PI+360.0);
        }
        else{
          Serial.print(azimuth*180.0/PI);
        }
        Serial.println();
        delay(100);
      }
      else
      {
        Serial.println(F("INVALID"));
      }
    }
    
  }
}

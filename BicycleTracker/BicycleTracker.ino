#include <Wire.h>
#include <rn2xx3.h>
#include "LSM303.h"
#include "Sodaq_UBlox_GPS.h"

LSM303 lsm303;
rn2xx3 myLora(Serial1);

const int MAGNETO_THRESHOLD = 200;

float lastAccelX = 0;
float lastAccelY = 0;
float lastAccelZ = 0;

float lastMagnetoX = 0;
float lastMagnetoY = 0;
float lastMagnetoZ = 0;

float accelXdif = 0;
float accelYdif = 0;
float accelZdif = 0;

float magnetoXdif = 0;
float magnetoYdif = 0;
float magnetoZdif = 0;

unsigned long unixTimestamp(int year, int month, int day,
              int hour, int min, int sec)
{
  const short days_since_beginning_of_year[12] = {0,31,59,90,120,151,181,212,243,273,304,334};
 
  int leap_years = ((year-1)-1968)/4
                  - ((year-1)-1900)/100
                  + ((year-1)-1600)/400;
 
  long days_since_1970 = (year-1970)*365 + leap_years
                      + days_since_beginning_of_year[month-1] + day-1;
 
  if ( (month>2) && (year%4==0 && (year%100!=0 || year%400==0)) )
    days_since_1970 += 1; /* +leap day, if year is a leap year */
 
  return sec + 60 * ( min + 60 * (hour + 24*days_since_1970) );
}



void sendData() {
  //create a new 10-bytes buffer
  uint8_t txBuffer[10];
  
  //get new gps information
  sodaq_gps.scan();

  //get timestamp
  uint32_t timestamp = unixTimestamp(sodaq_gps.getYear(), sodaq_gps.getMonth(), sodaq_gps.getDay(),
    sodaq_gps.getHour(), sodaq_gps.getMinute(), sodaq_gps.getSecond());

  //get gps latitude and longitude
  uint32_t latitude = ((sodaq_gps.getLat() + 90) / 180) * 16777215;
  uint32_t longitude = ((sodaq_gps.getLon() + 180) / 360) * 16777215;

  //4 bytes timestamp
  txBuffer[0] = ( timestamp >> 24) & 0xFF;
  txBuffer[1] = ( timestamp >> 16) & 0xFF;
  txBuffer[2] = ( timestamp >> 8) & 0xFF;
  txBuffer[3] = ( timestamp) & 0xFF;

  //3 bytes lat
  txBuffer[4] = ( latitude >> 16 ) & 0xFF;
  txBuffer[5] = ( latitude >> 8 ) & 0xFF;
  txBuffer[6] = latitude & 0xFF;

  //3 bytes lon
  txBuffer[7] = ( longitude >> 16 ) & 0xFF;
  txBuffer[8] = ( longitude >> 8 ) & 0xFF;
  txBuffer[9] = longitude & 0xFF;

  //send
  myLora.txBytes(txBuffer, sizeof(txBuffer));
}



bool isTurning() {
  //magneto meter turning threshold: 200
 return abs(magnetoXdif) > MAGNETO_THRESHOLD ||
        abs(magnetoYdif) > MAGNETO_THRESHOLD ||
        abs(magnetoZdif) > MAGNETO_THRESHOLD ;
}


void setLsm303Active(bool on)
{
  if (on) {
    if (!lsm303.init(LSM303::device_D, LSM303::sa0_low)) {
      SerialUSB.println("Initialization of the LSM303 failed!");
      return;
    }
    
    lsm303.enableDefault();
    lsm303.writeReg(LSM303::CTRL5, lsm303.readReg(LSM303::CTRL5) | 0b10001000); // enable temp and 12.5Hz ODR
    lsm303.writeReg(LSM303::CTRL1, 0b01110111); // 100Hz sampling rate, BDU on, all axes on
    lsm303.writeReg(LSM303::CTRL2, 0b01101100); // 50Hz anti-aliasing filter, +-8g scale

    delay(100);
  }
  else {
    // disable accelerometer, power-down mode
    lsm303.writeReg(LSM303::CTRL1, 0);
    
    // zero CTRL5 (including turn off TEMP sensor)
    lsm303.writeReg(LSM303::CTRL5, 0);

    // disable magnetometer, power-down mode
    lsm303.writeReg(LSM303::CTRL7, 0b00000010);
  }
}

void setup() {
  
  Wire.begin(); // enable 12c for accelero meter communication
  SerialUSB.begin(9600); // enable usb serial debugging to computer

  while ((!SerialUSB) && (millis() < 10000)); // wait for the usb serial to be connected, maximum 10 seconds
  SerialUSB.println("Startup");

  setLsm303Active(true);

  SerialUSB.print("Accelerometer type: LSM303");
  switch(lsm303.getDeviceType())
  {
    case lsm303.device_DLH:
      SerialUSB.println("DLH");
      break;
    case lsm303.device_DLM:
      SerialUSB.println("DLM");
      break;
    case lsm303.device_DLHC:
      SerialUSB.println("DLHC");
      break;
    case lsm303.device_D:
      SerialUSB.println("D");
      break;
    case lsm303.device_auto:
    default:
      SerialUSB.println(" - generic or unknown");
      break;
  }
}
void loop() {
  // put your main code here, to run repeatedly:
  lsm303.read();

  //read and update accelerometer values (milli-g-force)
  float accelX = lsm303.a.x*0.244;
  float accelY = lsm303.a.y*0.244;
  float accelZ = lsm303.a.z*0.244;

  SerialUSB.print("accelXdif = ");
  SerialUSB.println(accelXdif = accelX - lastAccelX);
  SerialUSB.print("accelYdif = ");
  SerialUSB.println(accelYdif = accelY - lastAccelY);
  SerialUSB.print("accelZdif = ");
  SerialUSB.println(accelZdif = accelZ - lastAccelZ);

  lastAccelX = accelX;
  lastAccelY = accelY;
  lastAccelZ = accelZ;

  SerialUSB.println("");

  //read and update magnetometer values (milli-gauss)
  float magnetoX = lsm303.m.x*0.160;
  float magnetoY = lsm303.m.y*0.160;
  float magnetoZ = lsm303.m.z*0.160;

  SerialUSB.print("magnetoXdif = ");
  SerialUSB.println(magnetoXdif = magnetoX - lastMagnetoX);
  SerialUSB.print("magnetoYdif = ");
  SerialUSB.println(magnetoYdif = magnetoY - lastMagnetoY);
  SerialUSB.print("magnetoZdif = ");
  SerialUSB.println(magnetoZdif = magnetoZ - lastMagnetoZ);

  lastMagnetoX = magnetoX;
  lastMagnetoY = magnetoY;
  lastMagnetoZ = magnetoZ;

  SerialUSB.println("");

  if (isTurning()) {
    SerialUSB.println("isTurning");
    sendData();
  }

  delay(1000);
}

#include <Wire.h>
#include <rn2xx3.h>
#include "LSM303.h"
#include "Sodaq_UBlox_GPS.h"

//we assume 50Hz, 2s=100 samples per axis
#define BUFFER_SIZE 100

// timestamp=4, lat=3, lon=3, max_x=2, max_y=2, max_z=2
uint8_t txBuffer[16];
uint32_t LatitudeBinary, LongitudeBinary;

int16_t max_x = 0;
int16_t max_y = 0;
int16_t max_z = 0;

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

// =========================================================================================

/** 
 *  FROM Sodaq universal tracker:
 * Initializes the LSM303 or puts it in power-down mode.
 */
void setLsm303Active(bool on)
{
    if (on) {
        if (!lsm303.init(LSM303::device_D, LSM303::sa0_low)) {
            SerialUSB.println("Initialization of the LSM303 failed!");
            return;
        }

        lsm303.enableDefault();
        // lsm303.writeReg(LSM303::CTRL5, lsm303.readReg(LSM303::CTRL5) | 0b10001000); // enable temp and 12.5Hz ODR 
        lsm303.writeReg(LSM303::CTRL1, 0b01110111); // 100Hz sampling rate, BDU on, all axes on
        lsm303.writeReg(LSM303::CTRL2, 0b11011000); // 50Hz anti-aliasing filter, +-8g scale
        
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

void initialize_radio()
{
  delay(100); //wait for the RN2xx3's startup message
  while(Serial1.available()){
    Serial1.read();
  }
  
  //print out the HWEUI so that we can register it via ttnctl
  String hweui = myLora.hweui();
  while(hweui.length() != 16)
  {
    SerialUSB.println("Communication with RN2xx3 unsuccesful. Power cycle the Sodaq One board.");
    delay(10000);
    hweui = myLora.hweui();
  }
  SerialUSB.println("Device EUI: ");
  SerialUSB.println(hweui);
  SerialUSB.println("RN2xx3 firmware version:");
  SerialUSB.println(myLora.sysver());

  //configure your keys and join the network
  SerialUSB.println("Trying to join LoRaWAN network");
  bool join_result = false;
  
  //ABP: initABP(String addr, String AppSKey, String NwkSKey);
  //join_result = myLora.initABP("02017201", "8D7FFEF938589D95AAD928C2E2E7E48F", "AE17E567AECC8787F749A62F5541D522");
  
  //OTAA: initOTAA(String AppEUI, String AppKey);
  join_result = myLora.initOTAA("1122334455667788", "11111111111111111111111111111111");

  while(!join_result)
  {
    SerialUSB.println("Unable to join. Are your keys correct, and do you have LoRaWAN coverage?");
    delay(60000); //delay a minute before retry
    join_result = myLora.init();
  }
  SerialUSB.println("Successfully joined LoRaWAN network");
  
}

// https://developer.mbed.org/questions/4552/Get-timestamp-via-GPS/
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

void setup() 
{
  delay(5000);
  Wire.begin();

  SerialUSB.begin(57600);
  Serial1.begin(57600);

  SerialUSB.println("Testing Accelerometer");
  
  initialize_radio();
  //myLora.setDR(0);
  
  setLsm303Active(true);
  
  // initialize GPS with enable on pin 27
  sodaq_gps.init(27);
}

void loop()
{

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


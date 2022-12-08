/* 
 *  ASAE 2022-2023 Controls Team "main plane" On-board DAS
 *  Team: Evan Eng (Captain), Andrew Brillante, Christopher Calandra, Nikki Pilla, and Omar Romero
 *  Github for 2022-2023: https://github.com/christophercalandra/UD-ASAE-Controls-2022
 *  Github fot 2021-2022: https://github.com/fdoyle6/UD-ASAE-Controls-2021
 *  Github for 2020-2021: https://github.com/FirkinTage/UDASAE_Controls
 *  The DAS code from previous years was referenced for early iterations of design
 */

// Libraries ------------------------------------------------------
# include "Adafruit_BMP3XX.h"
#include <Wire.h>             // Library for I2C/TWI devices
#include <SPI.h>              // Library for Serial Peripheral Interface to communicate with the microcontroller
#include <Adafruit_LSM9DS1.h> // Library for IMU
#include <Adafruit_Sensor.h>  // Library that defines basic information about sensors that can be used
#include <Adafruit_GPS.h>      // Library for GPS
#include <RH_RF95.h>
#include <Servo.h>
//#include <NMEAGPS.h>
//#include <GPSport.h>

// Declaration of Variables ----------------------------------------

// Altimeter - Model: Adafruit BMP 390
# define SEALEVELPRESSURE_HPA (1008.47)
Adafruit_BMP3XX bmp; // I2C   
float height = 0.0, initHeight = 0.0;        

// IMU - Model: Adafruit LSM9DS1
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();  // I2C

// GPS - Model: Adafruit Ultimate GPS
#define GPSSerial Serial1
Adafruit_GPS GPS(&GPSSerial);
// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO  false
//NMEAGPS  gps; // This parses the GPS characters
//gps_fix  fix; // This holds on to the latest values

// Radio - Model: RFM9Xx LoRa Radio
#define RFM95_CS 8 // Pin 8 on Adafruit Feather M0
#define RFM95_RST 4 // Pin 4 on Adafruit Feather M0
#define RFM95_INT 3 // Pin 3 on Adafruit Feather M0 (Interrupt Pin)
#define RF95_FREQ 915.0 // Must match the transmitter signal frequency (RX's freq)
RH_RF95 rf95(RFM95_CS, RFM95_INT); // Initializing Radio driver
#define Feather_LED 13 // Blink on receipt
// Data collection variables
float data_1, data_2, data_3, data_4, data_5, data_6, data_7, data_8, data_9, data_10, data_11;//, data_12, data_13; 
uint32_t timer = millis();
uint8_t recvDataPacket[RH_RF95_MAX_MESSAGE_LEN];  //packet that will be received from ground station
uint8_t recvLen = sizeof(recvDataPacket);

// Pada release button 
bool buttonpress = false; 

// Pada servo 
#define servoPin 13
Servo servoP; 

// Data Collection Function ------------------------------------------------
int runcount = 0;
String data_collection(){
  String package = ""; 

  //Altimeter - Checks if it is reading correclty
  if (! bmp.performReading()) {
    Serial.println("Altimeter failed to perform reading: ");
  }
  if (runcount < 5){
    initHeight = bmp.readAltitude(SEALEVELPRESSURE_HPA)*3.28084; // Meters to feet -> multiply by 3.28084
  }
  data_1 = ((bmp.readAltitude(SEALEVELPRESSURE_HPA)*3.28084)-initHeight);
  
  /* Get a new sensor event */ 
  sensors_event_t a, m, g, temp;
  lsm.getEvent(&a, &m, &g, &temp); 

  data_2 = a.acceleration.x;
  data_3 = a.acceleration.y;
  data_4 = a.acceleration.z;
  data_5 = m.magnetic.x; 
  data_6 = m.magnetic.y;
  data_7 = m.magnetic.z; 
  //data_8 = g.gyro.x;
  //data_9 = g.gyro.y;
  //data_10 = g.gyro.z;

  /************************
  char GPSRaw = GPS.read();

  if (GPSECHO)
    if (GPSRaw) Serial.print(GPSRaw);
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trying to print out data
    GPS.lastNMEA();
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      //return; // we can fail to parse a sentence in which case we should just wait for another
      Serial.println("Failed");
  }
  ************************/
  //if(timer > millis()) timer = millis();
  
  if(millis()-timer > 2000){
    timer = millis();
    if(GPS.fix){
      data_8 = GPS.latitudeDegrees * 1000000;    // Latitude Data
      data_9 = GPS.longitudeDegrees * 1000000;   // Longitude Data
      data_10 = GPS.speed * 1.15078;    // To convert knots to mph
    }
  }
  data_11 = 1; //Drop value is initially 1, gets set to 0 once drop signal is sent
    
    
  // Packaging data
  package += String(data_1);          // Altitude (m)
  package += "," + String(data_2);    // Acceleration x-dir
  package += "," + String(data_3);    // Acceleration y-dir
  package += "," + String(data_4);    // Acceleration z-dir
  package += "," + String(data_5);    // Magnetic x-dir
  package += "," + String(data_6);    // Magnetic y-dir
  package += "," + String(data_7);    // Magnetic z-dir
  //package += "," + String(data_8);    // Gyroscope x-dir 
  //package += "," + String(data_9);    // Gyroscope y-dir
  //package += "," + String(data_10);   // Gyroscope z-dir
  package += "," + String(data_8);   // Latitude (degrees)
  package += "," + String(data_9);   // Longitude (degrees)
  package += "," + String(data_10);   // Speed (mph)
  package += "," + String(data_11);   // Drop Value

  // Send Package over radio
  //int dataOutLen = int(package.length());
  //uint8_t dataOut[dataOutLen];
  uint8_t dataOut[package.length()];
  //dataOut[dataOutLen - 1] = 0;
  package.getBytes(dataOut, package.length() +1); 
  rf95.send(dataOut, sizeof(dataOut));
  rf95.waitPacketSent();
  
  return(package); 
}

// Setup ---------------------------------------------------------
void setup(){
  Serial.begin(115200); 
  servoP.attach(13);
  servoP.write(0);
  
  // Accelerometer
  while (!Serial) {
    delay(1); // will pause Zero, Leonardo, etc until serial console opens
  }
  // Try to initialise accelerometer and warn if we couldn't detect the chip
  if (!lsm.begin()){
    Serial.println("Oops ... unable to initialize the LSM9DS1. Check your wiring!");
    while (1);
  }
  else{
    //Serial.println("IMU ready");
  }

  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);    // Set the accelerometer range
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);     // Set the magnetometer sensitivity
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);  // Setup the gyroscope 


  delay(100);
  // Altimeter - wire setup
  if (!bmp.begin_I2C()) {   // hardware I2C mode, can pass in address & alt Wire
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1);
  }
  else{
    //Serial.println("Altimeter ready"); 
  }
  
  /****************
  Serial.begin(9600);
  while (!Serial)
    ;
  Serial.print( F("NMEAsimple.INO: started\n") );

  gpsPort.begin(9600);
  ****************/
  //Serial.println("Initializing GPS...");

  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); //1 Hz update rate (Choices: 1,2,5,10)
  GPS.sendCommand(PGCMD_ANTENNA);
  delay(1000);
  //Serial.println("GPS ready");
  GPSSerial.println(PMTK_Q_RELEASE);

  // Radio - Model: RFM9Xx LoRa Radio
  pinMode(Feather_LED, OUTPUT);     
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  delay(100);
 
  // Manual reset for radio
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
  while (!rf95.init()){
    Serial.println("LoRa radio init failed");
    while (1);
  }
  
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)){
    Serial.println("Set frequency failed");
    while (1);
  }
  else{
    //Serial.println("Radio ready");
  }

  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on
  
  
  
  rf95.setTxPower(23, false);
}

// Main Loop --------------------------------------------------------------
void loop() {
  runcount += 1;
  char GPSRaw = GPS.read();

  if (GPSECHO)
    if (GPSRaw) Serial.print(GPSRaw);
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trying to print out data
    GPS.lastNMEA();
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  }
  
  data_collection();
  Serial.println(data_collection()); 

  if (rf95.waitAvailableTimeout(100)){
    if (rf95.available()){
      if (rf95.recv(recvDataPacket, &recvLen)){
        recvDataPacket[recvLen-1]=0; //Terminating character
        if(strlen((char*)recvDataPacket) > 0){
          if ((strncmp((char*)recvDataPacket,"drop",4)==0)){
            Serial.println((char*)recvDataPacket);
            servoP.write(100); 
            delay(1000);
            servoP.write(0);
            delay(1000);
            //dropConfirm = true;
          }
        }
      }
    }
  }
}

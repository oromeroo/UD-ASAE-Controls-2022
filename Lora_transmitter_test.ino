/*This is example code for transmitting a message with the RH_RF95 class
This code is designed to work with the other LoRa9x_RX modules */

#include <SPI.h>
#include <RH_RF95.h>

#define RFM95_CS 10 // Pin 10 on Arduino Uno
#define RFM95_RST 9 // Pin 9 on Arduino Uno
#define RFM95_INT 2 // Pin 2 on Arduino Uno (Interrupt Pin)

#define RF95_FREQ 915.0 // Must match the reciever signal frequency (RX's freq)

RH_RF95 rf95(RFM95_CS, RFM95_INT); // Initializing Radio driver

void setup(){
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  while (!Serial);
  Serial.begin(9600);
  delay(100);

  Serial.println("Arduino LoRa TX Test!");

  // Manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("LoRa radio initialization failed");
    while (1);
  }
  Serial.println("LoRa radio initialization confirmed");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("Set frequency failed");
    while (1);
  }
  Serial.print("Set frequency to: "); Serial.println(RF95_FREQ);
  
  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

 /*The default transmitter power is 13dBm, using PA_BOOST. 
  * If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
  you can set transmitter powers from 5 to 23 dBm: */
  rf95.setTxPower(23, false);
}

int16_t packetnum = 0;  // Packet counter, we increment per xmission

void loop(){
//  Serial.println("Sending to rf95_server"); // Send message to rf95_server
  
  char radiopacket[16] = "Message #          ";
  itoa(packetnum++, radiopacket+9, 10);
//  Serial.print("Sending "); 
  Serial.println(radiopacket);
  radiopacket[19] = 0;
  
//  Serial.println("Sending..."); delay(10);
  rf95.send((uint8_t *)radiopacket, 20);

//  Serial.println("Waiting for packet to complete..."); delay(10);
  rf95.waitPacketSent();
  // Now wait for a reply
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

//  Serial.println("Waiting for reply..."); delay(10);
  if (rf95.waitAvailableTimeout(1000)){ 
    // Should be a reply message for us now   
    if (rf95.recv(buf, &len)){
      Serial.print("Received reply: ");
      Serial.println((char*)buf);
      Serial.print("RSSI: ");
      Serial.println(rf95.lastRssi(), DEC);    
    }  
    else{
      Serial.println("Receive failed");
    } 
  }
  else{
    Serial.println("No reply, is there a listener around?");
  }
  delay(1000);
}

#include <TinyGPS.h>

/*
  COAST BOARD
  The coast can make 1 master communicate with multiple divers.

  Sends the "$G..." message to the master buoy via LoRa every 10 sec,
  and receive the RNG calculated by the 3 buoys. It then sends them to
  a Thingsboard server.
*/

// Include libraries
#include <SPI.h>
#include <LoRa.h>
#include "BufferedOutput.h"
#include "SafeStringReader.h"
#include <ESP32Time.h>
#include "WiFi.h"
#include <WiFiClientSecure.h>

#define AUTOMATIC_MEASUREMENT true

WiFiClientSecure net = WiFiClientSecure();

ESP32Time rtc;

struct tm timeinfo;


const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 2*3600;
const int   daylightOffset_sec = 0;
unsigned long tiempo = 0;

const char WIFI_SSID[] = "JorgeWiFi";               //Cambiar 
const char WIFI_PASSWORD[] = "123456789";           //Cambiar 

const long frequency = 866E6;                         // LoRa Frequency
const int csPin = 5;                                  // LoRa radio chip select
const int resetPin = 4;                               // LoRa radio reset
const int irqPin = 27;                                // change for your board; must be a hardware interrupt pin

String outgoing[10];                                  // outgoing messages tab (coast can communicate with 10 divers max)

byte msgCount = 0;                                    // count of outgoing messages
byte localAddress = 0xFF;                             // address of this device
byte destination = 0xAA;                              // master buoy

int measuringInterval = 15000;                                 // interval between each ping request (ms)
unsigned long lastTime = 0;
int i = 0;

String RNGdata[10];                                   // save the RNG data received from the master (non sorted)

createBufferedOutput(input, 255, DROP_UNTIL_EMPTY);   // create an extra output buffer for the Serial2
createSafeStringReader(sfReader, 50, "\r\n");         // create SafeStringReader to hold messages written on Serial2
createSafeString(accoustSignal, 50);                  // SafeString to save the "$G..." message as long as it is the same

void RTCsetTime()
{
  configTime (gmtOffset_sec, daylightOffset_sec, ntpServer);
  Serial.println("[CU] Getting RTC...");
  if (getLocalTime(&timeinfo))
  {
    rtc.setTimeStruct(timeinfo);
  }
}

void connectWIFI() 
{
  uint8_t n = 0;

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.print("[CU] Attempting to connect...");
  
  while (WiFi.status() != WL_CONNECTED)                               // Espera hasta conectar con la red WiFi
  {    
    Serial.print(".");
    delay(500);
    if (++n > 15) {
      Serial.println("[SETUP] Impossible to connect... \n\n");
      return;
    }
  }
  Serial.print("[SETUP] Connected to WiFi network: ");
  Serial.println(WIFI_SSID);
  delay(250);
  RTCsetTime();
}

// Initialization of LoRa
void LoRaInit() 
{
  LoRa.setPins(csPin, resetPin, irqPin);

  if (!LoRa.begin(frequency)) {
    Serial.println("[CU] LoRa init failed. Check your connections.");
    Serial.println("[CU] Rebooting...");
    ESP.restart();                     // if failed, restart ESP
  }

  Serial.println("[CU] LoRa init succeeded!");
}


// Send a message to master buoy
void sendMessage(SafeString& outgoing) {
  LoRa.beginPacket();                   // start packet
  LoRa.write(destination);              // add destination address
  LoRa.write(localAddress);             // add sender address
  LoRa.write(msgCount);                 // add message ID
  LoRa.write(outgoing.length());        // add payload length
  LoRa.print(outgoing);                 // add payload
  LoRa.endPacket();                     // finish packet and send it
  msgCount++;                           // increment message ID
}

// Send a message to master buoy
void sendAtomaticMessage(String outgoing) {
  LoRa.beginPacket();                   // start packet
  LoRa.write(destination);              // add destination address
  LoRa.write(localAddress);             // add sender address
  LoRa.write(msgCount);                 // add message ID
  LoRa.write(outgoing.length());        // add payload length
  LoRa.print(outgoing);                 // add payload
  LoRa.endPacket();                     // finish packet and send it
  msgCount++;                           // increment message ID
}

// Receive a message from master buoy
void onReceive(int packetSize) 
{
  if (packetSize == 0) return;          // if there's no packet, return

  // read packet header bytes:
  int recipient = LoRa.read();          // recipient address
  byte sender = LoRa.read();            // sender address
  byte incomingMsgId = LoRa.read();     // incoming msg ID
  byte incomingLength = LoRa.read();    // incoming msg length

  // read the message
  String incoming = "";
  while (LoRa.available()) {
    incoming += (char)LoRa.read();
  }

  // check length for error
  if (incomingLength != incoming.length()) {
    Serial.print(rtc.getTime ("%d/%m/%Y %H:%M:%S "));
    Serial.println("[CU] error: message length does not match length");
    return;                             // skip rest of function
  }

  // if the recipient isn't this device or broadcast,
  if (recipient != localAddress || sender != destination) {
    return;                             // skip rest of function
  }

  // if message is for this device, or broadcast, print details:
  //Serial.println("Message ID: " + String(incomingMsgId));
  Serial.print(rtc.getTime ("%d/%m/%Y %H:%M:%S "));
  Serial.println(incoming);
  //Serial.println();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  // Initialize serial
  Serial.begin(9600);
  if (!Serial)
  {
    Serial.println("[CU] Serial failed. Rebooting...");
    ESP.restart();                     // if failed, restart ESP
  }

  Serial.println("[CU] COAST BOARD");
  
  connectWIFI();

  // Prepare Serial to be read
  SafeString::setOutput(Serial);      // enable error messages and SafeString.debug() output to be sent to Serial
  input.connect(Serial);              // where "input" will read from
  sfReader.connect(input);

  // Initialize LoRa
  LoRaInit();
} 


void loop() 
{
  unsigned long currentTime = millis();
  // Save text written on Serial
  while (Serial.available()) {
    if (sfReader.read()){
      Serial.print(rtc.getTime ("%d/%m/%Y %H:%M:%S "));
      Serial.print("[CU] Forwarding '");
      Serial.print(sfReader);
      Serial.println("' to Master Bouy...");
      
      sendMessage(sfReader);
    }
  delay(10);
  }


  if (AUTOMATIC_MEASUREMENT && currentTime - lastTime >= measuringInterval) {
    Serial.println("[CU] Sending automatically '$G,007,RNG' to Master Bouy...");
    sendAtomaticMessage("$G,007,RNG");
    lastTime = currentTime;
  }

  // Receive the RNG tab from master buoy
  onReceive(LoRa.parsePacket());

  // Release one byte from the buffer each 9600 baud
  input.nextByteOut();
  
}


/*
  MASTER BUOY
  When receiving the "$G..." message, sends a PING request to buoys 2 and 3,
  receive the calculated RNG by Wifi, pings the diver too and sends the 3 RNG
  via LoRa to the board on the coast.
*/

// Include libraries
#include <SPI.h>
#include <LoRa.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "SafeStringReader.h"
#include "BufferedOutput.h"
#include "sMQTTBroker.h"
#include <TinyGPS.h>
#include <SoftwareSerial.h>

#define MEASURETIMEOUT 15000         // Timeout for Range Measurement Sequency
#define PMTK_SET_NMEA_OUTPUT_ALLDATA "$PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28"

TinyGPS gps;
SoftwareSerial ss(14, 26);

float flat = 0.0, flon = 0.0;
unsigned long age = 0, prec = 0;

bool measuring = false;

// Initialize variables
const char* ssid = "ESP32-Access-Point";  // Wifi 
const char* password = "123456789";
WiFiServer server(80);
bool wifiConnected = false;

// MQTT Broker info
//String mqtt_broker;                        // IP address of the board the broker is running on (ie. master)
#define mqttServer "127.0.0.1"
const char *mqtt_username = "master";
const char *mqtt_password = "public";
const int mqtt_port = 1883;
sMQTTBroker broker;                       // broker instanciation
bool MQTTconnection = false;              // stock the MQTT connection state

WiFiClient espClient;                     // creation of a client for the broker
PubSubClient client(espClient);           // client instanciation

const long frequency = 866E6;             // LoRa Frequency
const int csPin = 5;                      // LoRa radio chip select
const int resetPin = 4;                   // LoRa radio reset
const int irqPin = 27;                    // change for your board; must be a hardware interrupt pin

byte localAddress = 0xAA;                 // address of this device
const byte destinationCoast = 0xFF;       // destination of board on the coast
byte msgCount = 0;                        // count of outgoing messages
byte sender;                              // sender address

String range = "";                        // stock the RNG calculated by each buoy

int nbPing = 0, lastPing = 0;             // current and last ping counters
int msgReceived1 = 0, msgReceived2 = 0, msgReceived3 = 0;    // variables to check if a RNG data from slaves 1/2 or master has been received

createBufferedOutput(output, 255, DROP_UNTIL_EMPTY);  // create an extra output buffer for the Serial2
createBufferedOutput(output2, 255, DROP_UNTIL_EMPTY); // create an extra output buffer for the ss (GPS)
createSafeStringReader(sfReader, 50, "\r\n");         // create SafeStringReader to hold messages written on Serial2
createSafeStringReader(sfGPS, 100, "\r");      // create SafeStringReader to hold messages written on ss (GPS)
createSafeString(accoustic, 50);                      // SafeStringReader holding the accoustic signal to cast
createSafeString(msgToCoast, 2000);                   // SafeStringReader holding the final message to sent to the coast board
//createSafeString(msgGPS, 100);                         // SafeStringReader holding the GPS msg

SemaphoreHandle_t myMutex;    //Semaphore

// Create Wifi connection from the ESP32
bool createWifi()
{
  try{
    Serial.print("Setting AP (Access Point)…");
    WiFi.softAP(ssid, password);
  
    IPAddress IP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(IP);
    
    server.begin();

    wifiConnected = true;
    Serial.println("Wifi creation succeed !\n");
    return true;
  }
  catch(bool wifiConnected)
  {
    Serial.println("AP Wifi creation failed !\n");
    return false;
  }
}

// Initialize LoRa
void LoRaInit(){
  LoRa.setPins(csPin, resetPin, irqPin);

  if (!LoRa.begin(frequency)) {           
    Serial.println("LoRa init failed. Check your connections.");
    ESP.restart();                     // if failed, restart ESP
  }

  Serial.println("LoRa init succeeded.\n");
}

// Connect to MQTT broker for Wifi communication
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    
    // Attempt to connect
    if (client.connect("ESP32Client", mqtt_username, mqtt_password)) {
      Serial.println("connected");
      client.subscribe("esp32/pinger/RNG");     // subscribe

      MQTTconnection = true;                    // MQTT connection established
      msgToCoast = "[MQTT] MASTER BUOY : MQTT connection established";
    }
    
    else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

// Send a message to the coast board
void sendMessage(SafeString& outgoing, byte destination) {
  LoRa.beginPacket();                   // start packet
  LoRa.write(destination);              // add destination address
  LoRa.write(localAddress);             // add sender address
  LoRa.write(msgCount);                 // add message ID
  LoRa.write(outgoing.length());        // add payload length
  LoRa.print(outgoing);                 // add payload
  LoRa.endPacket();                     // finish packet and send it
  msgCount++;                           // increment message ID
}

// Receive a message from the coast board
void onReceive(int packetSize) 
{
  bool rngCommand = false, mSeqDone = false;
  short mSeq = 0;
  long measurePrevTime, measureTime = 0;

  if (packetSize == 0) return;    // if there's no packet, return        
  
  // read packet header bytes:
  int recipient = LoRa.read();
  sender = LoRa.read();
  byte incomingMsgId = LoRa.read();
  byte incomingLength = LoRa.read();

  // read the message
  String incoming = "";
  while (LoRa.available()) {
    incoming += (char)LoRa.read();
  }
  Serial.print("MSG received from Coast Unit (LoRa): ");
  Serial.println(incoming);

  // check length for error
  if (incomingLength != incoming.length()) {
    Serial.println("error: message length does not match length");
    return;                             // skip rest of function
  }

  // If the recipient isn't this device or broadcast,
  if (recipient != localAddress || sender != destinationCoast) {
    Serial.println("Recipient isn't this device or broadcast");
    return;                             // skip rest of function
  }

  // If message received is from the coast, transmit it to buoys
  if (sender == destinationCoast )
  {
    Serial.println("Starting RNG measurement sequence...");
    measuring = true;
    accoustic = incoming.c_str();      // print the incoming message in the SafeString

    measurePrevTime = millis();
    while ((measureTime <= MEASURETIMEOUT) && !mSeqDone)
    {
      mSeqDone = false;
      measureTime = millis() - measurePrevTime;

      // Check the connection to the MQTT broker
      if (!client.connected()) {
        reconnect();
      }
      client.loop();

      switch(mSeq)
      {
        case 0:
          if (!rngCommand){
            Serial.println("\nSENDING PING REQUEST... \n");
            Serial2.println(accoustic);        // print incoming message on Serial2 so main buoy pings
            nbPing++;                          // increase the number of pings by 1
            rngCommand = true;
          }
          else{
            if (responseB1() == true){
              Serial.println("\nB1 RANGE RECEIVED \n");
              rngCommand = false;
              mSeq ++;
            }
          }
          break;

        case 1:
          if (!rngCommand){
            Serial.println("\nSENDING PING REQUEST TO S1... \n");
            publishMQTT(accoustic, 1);
            rngCommand = true;
          }
          else{
            if (msgReceived2){
              Serial.println("\nB2 RANGE RECEIVED \n");
              rngCommand = false;
              mSeq ++;
            }
          }
          break;

        case 2:
          if (!rngCommand){
            Serial.println("\nSENDING PING REQUEST TO S2... \n");
            publishMQTT(accoustic, 2);
            rngCommand = true;
          }
          else{
            if (msgReceived3){
              Serial.println("\nB3 RANGE RECEIVED \n");
              rngCommand = false;
              mSeq ++;
            }
          }
          break;

        default:
            Serial.println("\nRANGE MEASUREMENT SEQUENCE DONE \n");
            // Concatenate sequence time in range msg
            range += ("\n[RNG] T," + String(measureTime)).c_str();
            mSeqDone = true;
          break;
      }
      Serial.print("\nmeasureTime: ");
      Serial.println(measureTime);
    }
    if (measureTime > MEASURETIMEOUT)
    {
      Serial.println("\nRNG COMM TIMEOUT! \n");
      
      msgToCoast = ("[FAIL] RNG Measurement Timeout: " + String(msgReceived1 + msgReceived2 + msgReceived3)).c_str();
      sendMessage(msgToCoast, destinationCoast);
      measuring = false;
      msgToCoast.clear();
      emptyTab();
      msgReceived1 = 0, msgReceived2 = 0, msgReceived3 = 0;
    }
    //mSeqDone = false;
    accoustic.clear();                 // clear SafeString
  }
}

// Send a message to slave buoys via MQTT
void publishMQTT(SafeString& msg, int slave)
{
  // Convert sfReader which holds the ping request into a char*
  const char* msg_char = msg.c_str();

  // Publish the ping response to the desired slave buoy
  switch(slave){
    case 1:
      //delay(500);
      client.publish("esp32/pinger/request/S1", msg_char);
      break;

    case 2:
      //delay(1500);
      client.publish("esp32/pinger/request/S2", msg_char);
      break;

    default:
      break;
  }
}

// Receive a message from slave buoys via MQTT
void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.println(topic);

  // Read the received message
  String messageTemp;
  for (int i = 0; i < length; i++) 
    messageTemp += (char)message[i];

  // Check which type of message has been received from the slave buoys (MQTT connection or RNG data ?)
  char *slave1, *slave2, *MQTT;
  slave1 = strstr(messageTemp.c_str(), "slave1");      // check if the message received by master contains "slave1" 
  slave2 = strstr(messageTemp.c_str(), "slave2");      // check if the message received by master contains "slave2"
  MQTT = strstr(messageTemp.c_str(), "MQTT");          // check if the message received by master contains "MQTT" 

  // Treatment of the received message depending on the type of message
  if (slave1){
    msgReceived2 ++;               // 1 RNG data was received from slave 1
    
    // Concatenate the 2 messages in one
    const char* one = "\n[RNG] B2,";
    const char* two = &messageTemp[7]; // 7 because we don't want to read the 7 characters of "slave1 " in messageTemp
    
    char buf[100];
    strcpy(buf, one);
    strcat(buf, two);

    // Stock the message in the RNG tab
    range += buf;              
    Serial.println("Message from slave buoy #1");
    Serial.println(range);
  }
  else if (slave2){
    msgReceived3 ++;                // 1 RNG data was received from slave 2
    
    // Concatenate the 2 messages in one
    const char* one = "\n[RNG] B3,";
    const char* two = &messageTemp[7];  // 7 because we don't want to read the 7 characters of "slave1 " in messageTemp
  
    char buf[100];
    strcpy(buf, one);
    strcat(buf, two);

    // Stock the message in the RNG tab
    range +=  buf;               
    Serial.println("Message from slave buoy #2");
    Serial.println(range);
  }
  else if(MQTT){
    MQTTconnection = true;              // MQTT connection established
    msgToCoast = messageTemp.c_str();
  }
  
  Serial.println();
}

int parseAccoustic(String inputString) 
{
  int result = 0;
  int colonIndex = inputString.indexOf(",");  // Buscar la posición del carácter ":"
  char flatStr[20];
  char flonStr[20];

  String auxStr = inputString.substring(17, 20);
  Serial.println(auxStr);

  dtostrf(flat, 18, 15, flatStr);
  dtostrf(flon, 18, 15, flonStr);

  if (colonIndex >= 0) {
    if (auxStr == "ACK"){
      Serial.println("ACK Received");
      result = 1;
    }
    else if (auxStr == "LOG"){
      Serial.println("LOG Received");
      range += (inputString.substring(1, 10) + inputString.substring(17, 20) + ",Timeout").c_str();
      range += String(",") + flatStr + String(",") + flonStr + String(",") + String(prec);
      //range += (",45.225461999976616,14.612716000040493");
      result = 2;
    }
    else if (auxStr == "RNG"){
      Serial.println("RNG Received");
      range += (inputString.substring(1, 10) + inputString.substring(17, 20) + "," + inputString.substring(21, 25)).c_str();
      range += String(",") + flatStr + String(",") + flonStr + String(",") + String(prec);
      //range += (",45.225461999976616,14.612716000040493");
      result = 3;
    }
    else{
      Serial.println("Serial error");
      result = 0;
    }
  }
  else{
    Serial.println("Serial error");
    result = 0;
  }

  return result;  // Si no se encuentra el campo o hay algún error, devuelve una cadena vacía
}

// Stock the ping response received by master buoy in the RNG tab
bool responseB1()
{
  String B1range = "";
  // If something is written on Serial2
  if (sfReader.read()){
    msgReceived1 ++;                    // 1 RNG data was received from master
    
    B1range += "\n[RNG] B1,";
    B1range += sfReader.c_str();
    Serial.println(B1range);

    int res = parseAccoustic(B1range);
    Serial.print("Type received: ");
    Serial.println(res);
    switch(res)
    {
      case 0:           //En caso de que el mensaje recibido sea error o ACK devuelve un "false" (repite la lectura)
      case 1:
      default:
        return false;
        break;
      case 2:           //En caso de que el mensaje recibido sea LOG o RNG devuelve un "true" (lectura completada con fallo)
      case 3: 
        //range += B1range;
        return true;
        break;
    }
  }
  return false;
}

// Check if master received all the RNG data it is supposed to or not
bool isTabFilled() {
  // if we are waiting for the responses of ping #N and we receive it from every buoys
  // (each buoy has to receive 2 messages for the ping to be correctly done : one for the ACK and the other for the RNG)
  if(nbPing == lastPing+1 && (msgReceived1 == 2 && msgReceived2 == 2 && msgReceived3 == 2))
    return true;  // true, we send data

  // if we pass to a new ping #N+1 but we didn't receive the response from all the buoys
  else if (nbPing != lastPing+1 && (msgReceived1 != 0 || msgReceived2 != 0 || msgReceived3 != 0))  
    return true;   // false, but we still send data before emptying the tab

  // if we are waiting for the responses of ping #N and we don't receive it from every buoy
  return false;  // false, but we don't do nothing (either we wait until we receive every responses or the ping #N+1)
}

// Empty the RNG tab 
void emptyTab() {
  range = "";

  //Serial.println("TAB EMPTY\n");
}


/************************************************ MULTITHREADING FUNCTIONS ******************************************/

void brokerLoop(void * parameter)
{
  TaskHandle_t holderTask = xSemaphoreGetMutexHolder(myMutex);
  // Broker initialization
  const unsigned short mqttPort = 1883;

  if (holderTask == NULL) {
    broker.init(mqttPort);
    Serial.println("Broker initialized");
  
    // Update message received
    while(1) {
      broker.update();
      delay(100);        // to avoid watchdog error and reboot of the board
    }
  }    
}

void masterBuoyLoop(void * parameter)
{ 
  // Connect to MQTT broker
  client.setServer(mqttServer, mqtt_port);
  client.setCallback(callback);       // listen for incoming messages

  // Prepare Serial2 to be read
  SafeString::setOutput(Serial);      // enable error messages and SafeString.debug() output to be sent to Serial
  output.connect(Serial2);            // where "output" will read from
  sfReader.connect(output);

  while(1)
  {
    TaskHandle_t holderTask = xSemaphoreGetMutexHolder(myMutex);
    
    if (holderTask == NULL) {
      // Check the connection to the MQTT broker
      if (!client.connected()) {
        reconnect();
      }
      client.loop();

      // If the MQTT connection is established with 1 of the 3 buoys
      if(MQTTconnection){
        // Send a connexion confirmation message to the coast board
        sendMessage(msgToCoast, destinationCoast);

        // Change the connection state so †he message is sent just once
        MQTTconnection = false;
      }
    
      // Receive data from the coast board
      onReceive(LoRa.parsePacket());
    
      //delay(1000);
      // Stock the ping response received by master buoy in the RNG tab
      //responseB1();  
    
      // Send the RNG tab to the board on the coast over LoRa
      if(isTabFilled()){                            // if at least 1 of the 3 RNG is received
      msgToCoast = range.c_str();
      sendMessage(msgToCoast, destinationCoast);
      Serial.println("MESSAGE SENT.\n");

      // Reinitialization of the variable for the receive check
      msgReceived1 = 0, msgReceived2 = 0, msgReceived3 = 0;
      measuring = false;
      // Empty the RNG tab
      emptyTab();
      }

      // Save the number of the previous ping
      lastPing = nbPing - 1;
    
      // Reset the value of sender
      sender = 0;
    
      // Clear SafeStrings
      output.nextByteOut();
      msgToCoast.clear();
      msgReceived1 = 0, msgReceived2 = 0, msgReceived3 = 0;
      emptyTab();
    }
  }
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void GPS_Manager()
{
  bool newData = false;
  char charArray[50];


  while (ss.available())
  {
    xSemaphoreTake(myMutex, (TickType_t)0);
    char c = ss.read();
    //Serial.write(c); // uncomment this line if you want to see the GPS data flowing
    if (gps.encode(c)) // Did a new valid sentence come in?
      newData = true;
    delayMicroseconds(500);
  }
  /*
  if (sfGPS.read())
  {
    xSemaphoreTake(myMutex, (TickType_t)0);
    const char* gpsString = sfGPS.c_str();
    Serial.write(sfGPS.c_str());
    for (int i = 0; gpsString[i] != '\0'; ++i) {
      if (gps.encode(gpsString[i]))
        newData = true;
    }
  }*/

  xSemaphoreGive(myMutex);

  if (newData)
  {
    gps.f_get_position(&flat, &flon, &age);
    Serial.print("LAT=");
    Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 15);
    Serial.print(" LON=");
    Serial.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 15);
    Serial.print(" SAT=");
    Serial.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
    Serial.print(" PREC=");
    if (gps.hdop() == TinyGPS::GPS_INVALID_HDOP) {
      prec = 0;
    } else {
      prec = gps.hdop();
    }
    Serial.println(prec);
    
  }
}

TaskHandle_t Task1, Task2; // Task handles serve the purpose of referencing a particular task in other parts of the code

void setup() {
  // Initialize serial
  Serial.begin(9600);
  Serial2.begin(9600);  // initialize serial2 for nano modem pins 16 (receiver) and 17 (transmitter)
  ss.begin(9600);
  /*
  output2.connect(ss, 9600);
  Serial.println("MAIN BUOY");
  sfGPS.connect(output2);*/
  //while (!Serial);

  Serial.println("MAIN BUOY");

  // Create a Wi-Fi AP network with SSID and password
  while(!wifiConnected)
    createWifi();
  
  // Initialize LoRa
  LoRaInit();

  //GPS Initialized
  ss.println(PMTK_SET_NMEA_OUTPUT_ALLDATA);
  
  //Semaphore
  myMutex = xSemaphoreCreateMutex();

  // Multi threading 
  xTaskCreatePinnedToCore(brokerLoop,    "Broker Task",    10000,    NULL,    1,    &Task2,    0);
  xTaskCreatePinnedToCore(masterBuoyLoop,    "Master Buoy Task",    10000,      NULL,    1,    &Task1,    1);
}

void loop() 
{
  if (measuring == false)
  {
    GPS_Manager();
  }
}

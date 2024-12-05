/*
  SLAVE BUOY
  Receive the ping request from the master buoy, ping the desired
  diver and send the response to the master over MQTT.
*/

// Include libraries
#include <WiFi.h>
#include <PubSubClient.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>

#define BUOY_NUM 2

#define SSID "ESP32-Access-Point"    // SSID & password for WiFi
#define PASSWORD "123456789"

#define MQTT_BROKER "192.168.4.1"    // MQTT Broker info: has to be changed depending on the board the broker is running on
#define MQTT_USERNAME ("slave" + String(BUOY_NUM)).c_str()
#define MQTT_PASSWORD "public"
#define MQTT_PORT 1883

#define PMTK_SET_NMEA_OUTPUT_ALLDATA "$PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28"

TinyGPS gps;
SoftwareSerial ss(14, 26);

float flat = 0.0, flon = 0.0;
unsigned long age = 0, prec = 0;

bool MQTTconnection = false;                // stock the MQTT connection state
bool wificonn_notpossible = false;

bool measuring = false;

String range = "";                        // stock the RNG calculated by each buoy

WiFiClient espClient;                       // creation of a client for the broker
PubSubClient client(espClient);

void connectWIFI()
{
  uint8_t n = 0;

  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID, PASSWORD);

  Serial.print("    - Attempting WiFi connection...");

  while (WiFi.status() != WL_CONNECTED)                               // Espera hasta conectar con la red WiFi
  {
    Serial.print(".");
    delay(500);
    if (++n > 15)
    {
      Serial.println("[SETUP] Impossible to connect... \n\n");
      wificonn_notpossible = true;
      return;
    }
  }
  Serial.print("[SETUP] Connected to WiFi Network: ");
  Serial.println(SSID);
  delay(250);
}

// Check the connection to MQTT broker
void reconnect()
{
  // Loop until we're reconnected
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");

    // Attempt to connect
    String client_id = "esp32-client-";
    client_id += String(WiFi.macAddress());

    if (client.connect(client_id.c_str(), MQTT_USERNAME, MQTT_PASSWORD))
    {
      Serial.println("connected");
      client.subscribe(("esp32/pinger/request/S" + String(BUOY_NUM)).c_str());    // subscribe to receive ping request

      MQTTconnection = true;                          // MQTT connection established
    }

    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");

      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

// Receive a message
void callback(char* topic, byte* message, unsigned int length)
{
  String messageTemp = "";                         // message received by Serial2

  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");

  // Read the received message
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

  measuring = true;

  // Print the message on Serial2
  Serial.println("Message forwarding via Serial2 port (ESP32 -> nordic uC board -> nano modem)");
  Serial2.println(messageTemp);
  //messageTemp = "";             // clear variable
}

// Send a RNG message
void publishMQTT(const char* msg, bool RNG)
{
  char buf[100];

  // If the message to be sent is a RNG data
  if (RNG)
  {
    // Convert message which holds the ping response into a char*
    //const char* one = MQTT_USERNAME;
    const char* two = msg;

    // Concatenate the 2 char
    strcpy(buf, MQTT_USERNAME);
    strcat(buf, " ");
    strcat(buf, two);
  }

  // If the message to be published is NOT a RNG data
  else
    strcpy(buf, msg);

  // Publish the ping response
  client.publish("esp32/pinger/RNG", buf);
  Serial.print("DATA PUBLISHED : ");
  Serial.println(buf);
}

int parseAccoustic(String inputString) 
{
  int result = 0;
  int colonIndex = inputString.indexOf("$");  // Buscar la posición del carácter "$"

  String auxStr = inputString.substring(7, 10);
  //Serial.println(auxStr);

  if (colonIndex >= 0) {
    if (auxStr == "ACK"){
      Serial.println("ACK Received");
      result = 1;
    }
    else if (auxStr == "LOG"){
      Serial.println("LOG Received");
      range += (inputString.substring(7, 10) + ",Timeout").c_str();
      char flatStr[20];
      char flonStr[20];
      dtostrf(flat, 18, 15, flatStr);
      dtostrf(flon, 18, 15, flonStr);
      range += String(",") + flatStr + String(",") + flonStr + String(",") + String(prec);
      //range += (",45.225461999976616,14.612715999959510"); //S1
      //range += (",45.225462000046750,14.612716000000002"); //S2
      result = 2;
    }
    else if (auxStr == "RNG"){
      Serial.println("RNG Received");
      char flatStr[20];
      char flonStr[20];
      dtostrf(flat, 18, 15, flatStr);
      dtostrf(flon, 18, 15, flonStr);
      range += (inputString.substring(7, 10) + "," + inputString.substring(11, 15)).c_str();
      range += String(",") + flatStr + String(",") + flonStr + String(",") + String(prec);
      //range += (",45.225461999976616,14.612715999959510"); //S1
      //range += (",45.225462000046750,14.612716000000002"); //S2
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

// Stock the ping response received by buoy
bool responseB()
{
  bool msgRec = false;
  String Brange = "";

  // If something is written on Serial2
  while (Serial2.available())
  {
    char c = Serial2.read();
    if (c != '\r' && c != '\n')
    {
      Brange += c;
    }
    delay(5);
  }
  if (Brange != ""){
    msgRec = true;
  }
  else{
    msgRec = false;
  }
  if (msgRec)
  {
    Serial.println(Brange);

    int res = parseAccoustic(Brange);
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
        //range += Brange;
        return true;
        break;
    }
  }
  return false;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup()
{
  // Initialize serial
  Serial.begin(9600);
  Serial2.begin(9600);  // initialize serial2 for nano modem pins 16 (receiver) and 17 (transmitter)
  ss.begin(9600);
  //while (!Serial);

  Serial.print("SLAVE BUOY #");
  Serial.println(BUOY_NUM);

  // Connect to WiFi
  connectWIFI();

  // Connect to MQTT broker
  client.setServer(MQTT_BROKER, MQTT_PORT);
  client.setCallback(callback);       // listen for incoming messages
  
  ss.println(PMTK_SET_NMEA_OUTPUT_ALLDATA);
}


void loop()
{
  String messageTemp = "";                         // message received by Serial2
  // Check the connection to the broker
  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("Attempting to connect to WiFi Network...");

    connectWIFI();
  }
  if (WiFi.status() == WL_CONNECTED)
  {
    if (!client.connected())
    {
      reconnect();
    }
    client.loop();
  }

  // If the MQTT connection is established with the slave
  if (MQTTconnection)
  {
    // Send a connection confirmation message to the coast board
    const char* MQTTstate = ("[MQTT] SLAVE BUOY #" + String(BUOY_NUM) + " : MQTT connection established").c_str();
    publishMQTT(MQTTstate, false);

    // Change the connection state so the message is sent just once
    MQTTconnection = false;
  }
  bool res = responseB();

  if (measuring == false)
  {
    GPS_Manager();
  }

  if (res == true)
  {
    Serial.println("Publicando...");
    publishMQTT(range.c_str(), true);
    range = "";
    measuring = false;
  }
}

void GPS_Manager()
{
  bool newData = false;

  // For one second we parse GPS data and report some key values
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (ss.available())
    {
      char c = ss.read();
       //Serial.write(c); // uncomment this line if you want to see the GPS data flowing
      if (gps.encode(c)) // Did a new valid sentence come in?
        newData = true;
    }
  }

  if (newData)
  {
    gps.f_get_position(&flat, &flon, &age);
    Serial.print("LAT=");
    Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 10);
    Serial.print(" LON=");
    Serial.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 10);
    Serial.print(" SAT=");
    Serial.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
    Serial.print(" PREC=");
    if (gps.hdop() == TinyGPS::GPS_INVALID_HDOP) {
      prec = 0;
    } else {
      prec = gps.hdop();
    }
    Serial.print(prec);
  }
}
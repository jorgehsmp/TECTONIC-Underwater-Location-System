void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); 
  Serial.println("[CU] LoRa init succeed!");
  delay(3000);
  Serial.println("[MQTT] MASTER BOUY : MQTT connection established");
  delay(1000);
  Serial.println("[MQTT] SLAVE BOUY #1: MQTT connection established");
  delay(1000);
  Serial.println("[MQTT] SLAVE BOUY #2: MQTT connection established");
  delay(1000);
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(3000);
  Serial.println("[CU] Forwarding ''$G,007,RNG'' to Master Bouy...");
  delay(10000);
  Serial.println("[RNG] B1, RNG, 0.14,45.226703,14.613075");
  Serial.println("[RNG] B2, RNG, 0.23,45.226703,14.613075");
  Serial.println("[RNG] B3, RNG, 0.28,45.226703,14.613075");
  Serial.println("[RNG] T,4245");
  delay(3000);
  Serial.println("[CU] Forwarding ''$G,007,RNG'' to Master Bouy...");
  delay(10000);
  Serial.println("[RNG] B1, RNG, 0.09,45.226703,14.613075");
  Serial.println("[RNG] B2, RNG, 2.48,45.226703,14.613075");
  Serial.println("[RNG] B3, LOG, Timeout,45.226703,14.613075");
  Serial.println("[RNG] T,8472");
  delay(3000);
  Serial.println("[CU] Forwarding ''$G,007,RNG'' to Master Bouy...");
  delay(10000);
  Serial.println("[FAIL] RNG Measurement Timeout: 3");
}

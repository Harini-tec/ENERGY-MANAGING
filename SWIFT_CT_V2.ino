#include <WiFi.h>
#include <PubSubClient.h>
#include <EmonLib.h>             // Include Emon Library
#include <Arduino.h>
#include <ZMPT101B.h>

#define SENSITIVITY 500.0f

// WiFi credentials
const char* ssid = "paramesh";
const char* password = "94441param73392";

// MQTT broker settings
const char* mqtt_server = "swift1.paramchatmobile.com";
const int mqtt_port = 1883;
const char* mqtt_topic = "swiftpower";
String device_id = "prototype2";

WiFiClient espClient;
PubSubClient client(espClient);

ZMPT101B voltageSensor(33, 50.0);

EnergyMonitor emon1, emon2, emon3, emon4, emon5;      // Create instances for two CT sensors

const int analogPin1 = 32;       // Pin where the first CT sensor is connected
const int analogPin2 = 35;       // Pin where the second CT sensor is connected
const int analogPin3 = 34; 
const int analogPin4 = 39; 
const int analogPin5 = 36; 

float currentValue1 = 0.0;       // Measured current value for first sensor
float currentValue2 = 0.0;       // Measured current value for second sensor
float currentValue3 = 0.0; 
float currentValue4 = 0.0; 
float currentValue5 = 0.0; 



const float calibrationConstant1 = 43.48;  // Updated calibration constant for first CT (20A primary, 20mA secondary, 60Ω burden)
const float calibrationConstant2 = 16.67;  // Updated calibration constant for second CT (50A primary, 50mA secondary, 23Ω burden)
const float calibrationConstant3 = 16.67;
const float calibrationConstant4 = 16.67;
const float calibrationConstant5 = 16.67;

float voltage = 0;

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
  // Handle messages received from the broker if needed
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP32Client")) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish(mqtt_topic, "ESP32 connected");
      // ... and resubscribe
      client.subscribe(mqtt_topic);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);

  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  // Initialize the emon library with the analog pins and calibration constants
  emon1.current(analogPin1, calibrationConstant1);
  emon2.current(analogPin2, calibrationConstant2);
  emon3.current(analogPin3, calibrationConstant3);
  emon4.current(analogPin4, calibrationConstant4);
  emon5.current(analogPin5, calibrationConstant5);
  voltageSensor.setSensitivity(SENSITIVITY);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // Read the current values from both CT sensors
  currentValue1 = emon1.calcIrms(1780);  // Calculate Irms for first sensor over 1780 samples (about 1 cycle of 50Hz AC)
  currentValue2 = emon2.calcIrms(1780);  // Calculate Irms for second sensor over 1780 samples (about 1 cycle of 50Hz AC)
   currentValue3 = emon3.calcIrms(1780);
    currentValue4 = emon4.calcIrms(1780);
     currentValue5 = emon5.calcIrms(1780);
  voltage = voltageSensor.getRmsVoltage();

  // Calculate the actual current values and power
  float actualCurrent1 = currentValue1 / 3.536;
  float actualCurrent2 = currentValue2 / 3.536;
    float actualCurrent3 = currentValue3 / 3.536;
      float actualCurrent4 = currentValue4 / 3.536;
        float actualCurrent5 = currentValue5 / 3.536;
  float power1 = voltage * actualCurrent1;
  float power2 = voltage * actualCurrent2;
  float power3 = voltage * actualCurrent3;
  float power4 = voltage * actualCurrent4;
  float power5 = voltage * actualCurrent5;
  float frequency = 50.0;  // Assuming a fixed frequency of 50Hz
  float energy = 0.0;      // Placeholder for energy calculation if needed

  // Print the values to the Serial Monitor
  Serial.print("Current CT1 (A): ");
  Serial.println(actualCurrent1);
  Serial.print("Current CT2 (A): ");
  Serial.println(actualCurrent2);
    Serial.print("Current CT3 (A): ");
  Serial.println(actualCurrent3);
    Serial.print("Current CT4 (A): ");
  Serial.println(actualCurrent4);
    Serial.print("Current CT5 (A): ");
  Serial.println(actualCurrent5);

  
  Serial.print("Voltage (V): ");
  Serial.println(voltage);
  Serial.print("Power CT1 (W): ");
  Serial.println(power1);
  Serial.print("Power CT2 (W): ");
  Serial.println(power2);
  Serial.print("Power CT3 (W): ");
  Serial.println(power3);
  Serial.print("Power CT4 (W): ");
  Serial.println(power4);
  Serial.print("Power CT5 (W): ");
  Serial.println(power5);
  
  // Create the data strings for each channel
  String payload1 = device_id + ",channel1," + String(voltage) + "," + String(actualCurrent1) + "," + String(power1) + "," + String(energy) + "," + String(frequency);
  String payload2 = device_id + ",channel2," + String(voltage) + "," + String(actualCurrent2) + "," + String(power2) + "," + String(energy) + "," + String(frequency);
   String payload3 = device_id + ",channel3," + String(voltage) + "," + String(actualCurrent3) + "," + String(power3) + "," + String(energy) + "," + String(frequency);
    String payload4 = device_id + ",channel4," + String(voltage) + "," + String(actualCurrent4) + "," + String(power4) + "," + String(energy) + "," + String(frequency);
     String payload5 = device_id + ",channel5," + String(voltage) + "," + String(actualCurrent5) + "," + String(power5) + "," + String(energy) + "," + String(frequency);
  // Publish the data to the MQTT broker
  client.publish(mqtt_topic, payload1.c_str());
  delay(2000);
  client.publish(mqtt_topic, payload2.c_str());
  delay(2000);
  client.publish(mqtt_topic, payload3.c_str());
  delay(2000);
  client.publish(mqtt_topic, payload4.c_str());
  delay(2000);
  client.publish(mqtt_topic, payload5.c_str());
  delay(2000);
}

#include <ArduinoMqttClient.h>
#if defined(ARDUINO_SAMD_MKRWIFI1010) || defined(ARDUINO_SAMD_NANO_33_IOT) || defined(ARDUINO_AVR_UNO_WIFI_REV2)
#include <WiFiNINA.h>
#elif defined(ARDUINO_SAMD_MKR1000)
#include <WiFi101.h>
#elif defined(ARDUINO_ARCH_ESP8266)
#include <ESP8266WiFi.h>
#elif defined(ARDUINO_ARCH_ESP32)
#include <WiFi.h>
#endif

char wifiSSID[] = "Sachin";    // Replace with your network SSID (name)
char wifiPassword[] = "sachinarora";    // Replace with your network password (use for WPA, or as a key for WEP)

const int triggerPin = 2;    // Pin for triggering distance measurement
const int echoPin = 3;    // Pin for receiving echo signal

float duration, measuredDistance;

WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

const char mqttBroker[] = "broker.mqttdashboard.com";  // MQTT broker address
int mqttPort = 1883;                                  // MQTT broker port
const char mqttTopic[] = "SIT_210/detection";              // MQTT topic for publishing data

const long measurementInterval = 1000;                 // Interval between distance measurements
unsigned long previousMeasurementMillis = 0;           // Store the previous time

int measurementCount = 0;

void setup() {
  // Initialize serial communication and wait for the port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // Wait for the serial port to connect (needed for native USB port only)
  }

  // Attempt to connect to the Wi-Fi network:
  Serial.print("Attempting to connect to WPA SSID: ");
  Serial.println(wifiSSID);
  while (WiFi.begin(wifiSSID, wifiPassword) != WL_CONNECTED) {
    // Connection failed, retrying
    Serial.print(".");
    delay(5000);
  }

  Serial.println("Connected to the network");
  Serial.println();

  // Connect to the MQTT broker:
  Serial.print("Attempting to connect to the MQTT broker: ");
  Serial.println(mqttBroker);

  if (!mqttClient.connect(mqttBroker, mqttPort)) {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());

    while (1);
  }

  Serial.println("Connected to the MQTT broker!");
  Serial.println();
}

void loop() {
  mqttClient.poll();

  // Measure distance at regular intervals to avoid delays in the loop:
  unsigned long currentMillis = millis();

  if (currentMillis - previousMeasurementMillis >= measurementInterval)
  {
    previousMeasurementMillis = currentMillis;

    // Ultrasonic sensor measurement:
    digitalWrite(triggerPin, LOW);
    delayMicroseconds(2);
    digitalWrite(triggerPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(triggerPin, LOW);

    duration = pulseIn(echoPin, HIGH);
    measuredDistance = (duration * 0.0343) / 2;
    Serial.print("Measured Distance: ");
    Serial.println(measuredDistance);

    // Publish MQTT message if an object is detected:
    if (measuredDistance < 12)
    {
      mqttClient.beginMessage(mqttTopic);
      mqttClient.print("Object detected nearby.");
      delay(1000);
    }

    Serial.println();

    measurementCount++;
  }
}

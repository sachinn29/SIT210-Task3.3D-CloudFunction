#include <ArduinoMqttClient>

// Include the appropriate Wi-Fi library based on the board being used
#if defined(ARDUINO_SAMD_MKRWIFI1010) || defined(ARDUINO_SAMD_NANO_33_IOT) || defined(ARDUINO_AVR_UNO_WIFI_REV2)
#include <WiFiNINA.h>
#elif defined(ARDUINO_SAMD_MKR1000)
#include <WiFi101.h>
#elif defined(ARDUINO_ARCH_ESP8266)
#include <ESP8266WiFi.h>
#elif defined(ARDUINO_ARCH_ESP32)
#include <WiFi.h>
#endif

char wifiNetworkName[] = "Sachin";    // Replace with your network SSID (name)
char wifiPassword[] = "sachinarora";    // Replace with your network password (use for WPA, or as a key for WEP)

int ledPin = 2;  // Pin for controlling an LED

WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

const char mqttBroker[] = "mqtt-dashboard.com";
int mqttPort = 1883;
const char mqttTopic[] = "sachin-object";

void setup() {
  // Initialize serial communication and wait for the port to open:
  Serial.begin(9600);
  pinMode(ledPin, OUTPUT);
  while (!Serial) {
    ; // Wait for the serial port to connect. Needed for the native USB port only
  }

  // Attempt to connect to the Wi-Fi network:
  Serial.print("Attempting to connect to WiFi network: ");
  Serial.println(wifiNetworkName);
  while (WiFi.begin(wifiNetworkName, wifiPassword) != WL_CONNECTED) {
    // Connection failed, retry
    Serial.print(".");
    delay(5000);
  }

  Serial.println("Connected to the network");
  Serial.println();

  Serial.print("Attempting to connect to the MQTT broker: ");
  Serial.println(mqttBroker);

  if (!mqttClient.connect(mqttBroker, mqttPort)) {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());

    while (1);
  }

  Serial.println("Connected to the MQTT broker!");
  Serial.println();

  Serial.print("Subscribing to topic: ");
  Serial.println(mqttTopic);
  Serial.println();

  // Subscribe to a topic
  mqttClient.subscribe(mqttTopic);

  Serial.print("Waiting for messages on topic: ");
  Serial.println(mqttTopic);
  Serial.println();
}

void loop() {
  int messageSize = mqttClient.parseMessage();
  if (messageSize) {
    // We received a message, print out the topic and contents
    Serial.print("Received a message with topic '");
    Serial.print(mqttClient.messageTopic());
    Serial.print("', length ");
    Serial.print(messageSize);
    Serial.println(" bytes:");

    // Use the Stream interface to print the contents
    while (mqttClient.available()) {
      Serial.print((char)mqttClient.read());
    }
    Serial.println();

    // Blink the LED to indicate message reception
    for (int i = 0; i < 3; i++) {
      digitalWrite(ledPin, HIGH);
      delay(200);
      digitalWrite(ledPin, LOW);
      delay(200);
    }

    Serial.println();
  }
}

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include "secrets.h"

#define LED_PIN 8
#define WIFI_CONN_TIMEOUT 10000 // 10 seconds
#define WIFI_CONN_CHECK_FREQ 200 

// F0:9E:9E:B5:65:5C - Master MAC Address
uint8_t masterAddress[] = {0xF0, 0x9E, 0x9E, 0xB5, 0x65, 0x5C};
unsigned int WiFiConnectionTimeStarted = 0;
const long HEARTBEAT_INTERVAL_MS = 100; 
unsigned long lastSendTime = 0;
uint8_t dataToSend = 0xDE; // Data doesn't really matter, just the act of sending

void initWiFi();
void initESPNow();
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);

void setup() {
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(115200);

  initWiFi();  
  initESPNow();

  Serial.println("Detector board ready. Sending heartbeats...");
}

void loop() {
  unsigned long currentTime = millis();

  if (currentTime - lastSendTime >= HEARTBEAT_INTERVAL_MS) {
    esp_err_t result = esp_now_send(masterAddress, &dataToSend, sizeof(dataToSend));
       
    if (result == ESP_OK) {
      //Serial.println("Heartbeat sent successfully.");
    } else {
      Serial.printf("Error sending heartbeat (code %d).", result);
    }
        
    lastSendTime = currentTime;
  }
    
  delay(100);
}

void initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  WiFiConnectionTimeStarted = millis();
  
  while (!WiFi.isConnected()) {
    digitalWrite(LED_PIN, HIGH);
    delay(WIFI_CONN_CHECK_FREQ / 2);
    digitalWrite(LED_PIN, LOW);
    delay(WIFI_CONN_CHECK_FREQ / 2);

    if (millis() - WiFiConnectionTimeStarted > WIFI_CONN_TIMEOUT) {
      Serial.printf("WiFI connection failed (code %d)! Rebooting in 5 seconds..", WiFi.waitForConnectResult());
      delay(5000);
      ESP.restart();
      return;
    }
  }

  Serial.printf("Connected to WiFi, IP: %s, Chan: %d\n\r", WiFi.localIP().toString().c_str(), WiFi.channel());
  Serial.printf("RSSI: %d\n\r", WiFi.RSSI());
  Serial.printf("Board MAC: %s\n\r", WiFi.macAddress().c_str());
}

void initESPNow() {
  esp_err_t result = esp_now_init();
  if (result != ESP_OK) {
      Serial.printf("Error initializing ESP-NOW (code %d)! Rebooting in 5 seconds..", result);
      delay(5000);
      ESP.restart();
      return;
  }

  esp_now_register_send_cb(OnDataSent);

  // register peer
  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, masterAddress, 6);
  peerInfo.channel = WiFi.channel(); // Use default channel (0)
  peerInfo.encrypt = false;
  result = esp_now_add_peer(&peerInfo);
  if (result != ESP_OK){
    Serial.printf("Failed to add peer (code %d)! Rebooting in 5 seconds..", result);
    delay(5000);
    ESP.restart();
    return;
  }
}

// Callback function when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    Serial.print("Last Packet Send Status: ");
    if (status == ESP_NOW_SEND_SUCCESS) {
      Serial.println("Delivery Success");
      digitalWrite(LED_PIN, LOW);
    } else {
      digitalWrite(LED_PIN, HIGH);
      // If the Master is OFF (power out), the delivery will fail. 
      // We log it, but we don't act on it here.
      Serial.printf("Delivery Fail (%d)\n\r", status);
    }
}

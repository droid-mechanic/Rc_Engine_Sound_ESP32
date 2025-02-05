#include <Arduino.h>
#include <Bluepad32.h>
#include <esp_now.h>
#include <WiFi.h>

ControllerPtr controller;

uint8_t broadcastAddress1[] = {0xE4, 0x65, 0xB8, 0xD8, 0xD3, 0x1C};

esp_now_peer_info_t peerInfo;

typedef struct struct_message
{ // This is the data packet
  uint8_t axisX;
  uint8_t axisY;
  uint8_t axisLX;
  uint8_t axisLY;
  bool l1;
  bool l2;
  bool r1;
  bool r2;
  uint8_t dpad;
  bool button1;
  bool button2;
  bool button3;
  bool button4;
} struct_message;

// Create a struct_message called trailerData
struct_message remoteData;

void onConnectedController(ControllerPtr ctl) {
  if (controller == nullptr) {
    Serial.printf("CALLBACK: Controller is connected");
    ControllerProperties properties = ctl->getProperties();
    Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id, properties.product_id);
    controller = ctl;
    ctl->setColorLED(255, 0, 0);
    ctl->playDualRumble(0 /* delayedStartMs */, 250 /* durationMs */, 0x80 /* weakMagnitude */, 0x40 /* strongMagnitude */);
  } else {
    Serial.println("CALLBACK: Controller connected, but could not found empty slot");
  }
}

void onDisconnectedController(ControllerPtr ctl) {
  if (controller == ctl) {
    Serial.printf("CALLBACK: Controller disconnected");
    controller = nullptr;
  } else {
    Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
  }
}

void processGamepad(ControllerPtr ctl) {
  int LXValue = ctl->axisX();
  int LYValue = ctl->axisY();

  remoteData.axisX = map(LXValue, -512, 511, 0, 256);
  remoteData.axisY = map(LYValue, -512, 511, 0, 256);
  remoteData.axisLX = map(ctl->axisLX, -512, 511, 0, 256);
  remoteData.axisLY = map(ctl->axisLY, -512, 511, 0, 256);
  remoteData.dpad = ctl->dpad();
  remoteData.button1 = ctl->a();
  remoteData.button2 = ctl->b();
  remoteData.button3 = ctl->x();
  remoteData.button4 = ctl->y();

  esp_err_t result = esp_now_send(0, (uint8_t *) &remoteData, sizeof(struct_message));
}

void processControllers() {
  if (controller && controller->isConnected() && controller->hasData()) {
    if (controller->isGamepad()) {
      processGamepad(controller);
    } else {
      Serial.println("Unsupported controller");
    }
  }
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  Serial.print("Packet to: ");
  // Copies the sender mac address to a string
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print(macStr);
  Serial.print(" send status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup() {
  Serial.begin(115200);
  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t* addr = BP32.localBdAddress();
  Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.forgetBluetoothKeys();
  BP32.enableVirtualDevice(false);
  // Set device as a Wi-Fi Station for ESP-NOW
  WiFi.mode(WIFI_STA); // WIFI_STA = Station (router required) WIFI_AP = ESP32 is an access point for stations
  WiFi.setTxPower (WIFI_POWER_MINUS_1dBm); // Set power to lowest possible value WIFI_POWER_MINUS_1dBm  WIFI_POWER_19_5dBm
  // shut down wifi
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(OnDataSent);

  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  // register first peer  
  memcpy(peerInfo.peer_addr, broadcastAddress1, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    //Serial.println("Failed to add peer");
    return;
  }

  Serial.println("Ready.");
}

void loop() {
  unsigned long currentTime = millis();
  bool dataUpdated = BP32.update();
  if (dataUpdated) {
    processControllers();
  }
  vTaskDelay(1);
}
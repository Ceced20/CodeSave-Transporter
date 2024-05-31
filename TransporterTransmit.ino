#include <esp_now.h>
#include <WiFi.h>

// Define joystick pins
#define joystick1_x 32
#define joystick1_y 33
#define joystick2_x 35
#define joystick2_y 34

// State variables for joystick positions
int last_x_value1 = 512;
int last_y_value1 = 512;
int last_x_value2 = 512;
int last_y_value2 = 512;

// Receiver MAC Address
uint8_t broadcastAddress[] = {0xCC, 0x7B, 0x5C, 0x26, 0xCC, 0xB4};

// Structure to send data
typedef struct struct_message {
  int joystick1_x_value;
  int joystick1_y_value;
  int joystick2_x_value;
  int joystick2_y_value;
} struct_message;

// Create a struct_message called joystickData
struct_message joystickData;

esp_now_peer_info_t peerInfo;

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup() {
  // Init Serial Monitor
  Serial.begin(115200);

  // Set up joystick pins
  pinMode(joystick1_x, INPUT);
  pinMode(joystick1_y, INPUT);
  pinMode(joystick2_x, INPUT);
  pinMode(joystick2_y, INPUT);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register send callback
  esp_now_register_send_cb(OnDataSent);

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
}

void loop() {
  // Read joystick values
  int xValue1 = analogRead(joystick1_x);
  int yValue1 = analogRead(joystick1_y);
  int xValue2 = analogRead(joystick2_x);
  int yValue2 = analogRead(joystick2_y);

  // Check if joystick 1 has moved significantly
  if (abs(xValue1 - last_x_value1) > 10 || abs(yValue1 - last_y_value1) > 10) {
    Serial.print("Joystick 1 moved: x=");
    Serial.print(xValue1);
    Serial.print(", y=");
    Serial.println(yValue1);

    joystickData.joystick1_x_value = xValue1;
    joystickData.joystick1_y_value = yValue1;
    last_x_value1 = xValue1;
    last_y_value1 = yValue1;
  }

  // Check if joystick 2 has moved significantly
  if (abs(xValue2 - last_x_value2) > 10 || abs(yValue2 - last_y_value2) > 10) {
    Serial.print("Joystick 2 moved: x=");
    Serial.print(xValue2);
    Serial.print(", y=");
    Serial.println(yValue2);

    joystickData.joystick2_x_value = xValue2;
    joystickData.joystick2_y_value = yValue2;
    last_x_value2 = xValue2;
    last_y_value2 = yValue2;
  }

  // Send joystick data via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &joystickData, sizeof(joystickData));

  if (result == ESP_OK) {
    Serial.println("Sent with success");
  } else {
    Serial.println("Error sending the data");
  }

  delay(50); // Send data at 20Hz
}

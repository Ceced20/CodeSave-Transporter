#include <esp_now.h>
#include <WiFi.h>

// Structure to receive data
typedef struct struct_message {
  int joystick1_x_value;
  int joystick1_y_value;
  int joystick2_x_value;
  int joystick2_y_value;
} struct_message;

// Create a struct_message to hold incoming data
struct_message joystickData;

// Callback function executed when data is received
void OnDataRecv(const esp_now_recv_info *recvInfo, const uint8_t *incomingData, int len) {
  // Cast the incomingData pointer to the correct struct_message type
  const struct_message *data = reinterpret_cast<const struct_message*>(incomingData);
  
  // Extract data and print
  Serial.print("Joystick 1: x=");
  Serial.print(data->joystick1_x_value);
  Serial.print(", y=");
  Serial.println(data->joystick1_y_value);
  Serial.print("Joystick 2: x=");
  Serial.print(data->joystick2_x_value);
  Serial.print(", y=");
  Serial.println(data->joystick2_y_value);
}

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init()!= ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register receive callback
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  // Nothing to do here
}

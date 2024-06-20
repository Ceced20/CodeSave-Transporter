#include <BluetoothSerial.h>

// Define joystick pins
#define joystick2_x 32
#define joystick2_y 33
#define joystick1_x 34
#define joystick1_y 35
#define button1 14
#define button2 13
#define button3 27
//tombol-tombol active low

// State variables for joystick positions
int last_x_value1 = 512;
int last_y_value1 = 512;
int last_x_value2 = 512;
int last_y_value2 = 512;
int b3state = 0;
// Receiver MAC Address
uint8_t recvAddress[] = {0xCC, 0x7B, 0x5C, 0x26, 0xCC, 0xB6};
const char pin[] = "1234";
String transmitter_btname = "BT-Transmitter";
bool connected = false;

BluetoothSerial btserial;
// Structure to send data
typedef struct struct_message {
  uint16_t headerbytes;
  int M1;
  int M2;
  int servoA;
  int servoC;
  int servoS;
} struct_message;

// Create a struct_message called joystickData
struct_message joystickData;

// Math power forward
int mapToPower(int value, int maxpower) {
  int midpoint = 2048;
  float powerRange = maxpower;
  float scaledValue = ((float)value - midpoint) / midpoint;
  return (int)(scaledValue * maxpower);

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


  // Init ESP-NOW
  Serial.println("init bluetooth...");
  btserial.begin(transmitter_btname, true);
  btserial.setPin(pin);
}

void loop() {
  //tunggu sampai connect
  if (!connected) {
    Serial.println("connecting...");
    connected = btserial.connect(recvAddress);
  }
  if (connected) {
    Serial.println("Connected successfully!");
  }
  while (btserial.connected()) {

    // Read joystick values
    int xValue1 = analogRead(joystick1_x);
    int yValue1 = 4095 - analogRead(joystick1_y);
    int xValue2 = analogRead(joystick2_x);
    int yValue2 = 4095 - analogRead(joystick2_y);

    // Map joystick values to power
    int scaledValueX = mapToPower(xValue1, 100);
    int scaledValueY = mapToPower(yValue1, 100);
    joystickData.M1 = (scaledValueX / 2)  + scaledValueY;
    joystickData.M2 = (-scaledValueX / 2) + scaledValueY;
    joystickData.M1 = constrain(joystickData.M1, -100, 100);
    joystickData.M2 = constrain(joystickData.M2, -100, 100);
    if (digitalRead(button1) == LOW ) {
      joystickData.servoA = 0;
    }

    if (digitalRead(button2) == LOW) {
      joystickData.servoA = 180;
    }

    if (digitalRead(button3) == LOW) {
      if (b3state == 0) { //sebelumnya tidak ditekan
        b3state = 1; //catat keadaan sekarang supaya tidak mengulang
        if (joystickData.servoC == 140) {
          joystickData.servoC = 20;
        }
        else {
          joystickData.servoC = 140;
        }
      }
    }
    else { //kalau button tidak ditekan
      b3state = 0;
    }
    //joystick2 mapping value

    scaledValueX = mapToPower(xValue2, 180);
    scaledValueY = mapToPower(yValue2, 180);
    joystickData.servoS = atan2(scaledValueY, -scaledValueX) * 57.29578 + 90;
    Serial.print(xValue1); Serial.print(",");
    Serial.print(yValue1); Serial.print(",");
    Serial.print(xValue2); Serial.print(",");
    Serial.print(yValue2); Serial.print(",   ");
    Serial.print(joystickData.M1); Serial.print(",");
    Serial.print(joystickData.M2); Serial.print(",");
    Serial.print(joystickData.servoA); Serial.print(",");
    Serial.print(joystickData.servoC); Serial.print(",");
    Serial.print(joystickData.servoS); Serial.println();

    /*
      // Check if joystick 1 has moved significantly
      if (abs(xValue1 - last_x_value1) > 10 || abs(yValue1 - last_y_value1) > 10) {
        Serial.print("Joystick 1 moved: x=");
        Serial.print(joystickData.M1);
        Serial.print(", y=");
        Serial.println(joystickData.M2);
        last_x_value1 = xValue1;
        last_y_value1 = yValue1;
      }

      // Check if joystick 2 has moved significantly
      if (abs(xValue2 - last_x_value2) > 10 || abs(yValue2 - last_y_value2) > 10) {
        Serial.print("Joystick 2 moved: s=");
        Serial.println(joystickData.servoS);
        last_x_value2 = xValue2;
        last_y_value2 = yValue2;
      }
    */
    // Send joystick data via bluetooth
    joystickData.headerbytes = 0x2655;
    //btserial.println(joystickData.M1);
    btserial.write((uint8_t*) &joystickData, sizeof(joystickData));
    /*
      if (result == ESP_OK) {
        Serial.println("Sent with success");
      } else {
        //Serial.println("Error sending the data");
      }
    */
    delay(50); // Send data at 20Hz
  }
}

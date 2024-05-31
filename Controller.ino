#include <BluetoothSerial.h>
BluetoothSerial SerialBT;

int joystick1_x = 32;
int joystick1_y = 33;
int joystick2_x = 35;
int joystick2_y = 32;
int last_x_value1 = 512;
int last_y_value1 = 512;
int last_x_value2 = 512;
int last_y_value2 = 512;

void setup() {
  Serial.begin(9600);
  SerialBT.begin("ESP32test");
}

void loop() {
  int xValue1 = analogRead(joystick1_x);
  int yValue1 = analogRead(joystick1_y);
  int xValue2 = analogRead(joystick2_x);
  int yValue2 = analogRead(joystick2_y);

  if (abs(xValue1 - last_x_value1) > 10 || abs(yValue1 - last_y_value1) > 10) {
    Serial.print("Joystick 1 moved: x=");
    Serial.print(xValue1);
    Serial.print(", y=");
    Serial.println(yValue1);
    SerialBT.print("Joystick 1 moved: x=");
    SerialBT.print(xValue1);
    SerialBT.print(", y=");
    SerialBT.println(yValue1);
    last_x_value1 = xValue1;
    last_y_value1 = yValue1;

    if (yValue1 > 512) { // Forward
      int speed1 = map(yValue1, 512, 1023, 10, 100); // Map to 10-100% speed
      char command1[2] = {'W', (char)speed1};
      SerialBT.write((const uint8_t*)command1, 2);
    } else if (yValue1 < 512) { // Backward
      int speed1 = map(yValue1, 0, 511, 10, 100); // Map to 10-100% speed
      char command1[2] = {'S', (char)speed1};
      SerialBT.write((const uint8_t*)command1, 2);
    }
  } else {
    char command1[2] = {'S', 0};
    SerialBT.write((const uint8_t*)command1, 2);
  }

  if (abs(xValue2 - last_x_value2) > 10 || abs(yValue2 - last_y_value2) > 10) {
    Serial.print("Joystick 2 moved: x=");
    Serial.print(xValue2);
    Serial.print(", y=");
    Serial.println(yValue2);
    SerialBT.print("Joystick 2 moved: x=");
    SerialBT.print(xValue2);
    SerialBT.print(", y=");
    SerialBT.println(yValue2);
    last_x_value2 = xValue2;
    last_y_value2 = yValue2;

    if (xValue2 > 512) { // Right
      int speed2 = map(xValue2, 512, 1023, 10, 100); // Map to 10-100% speed
      char command2[2] = {'D', (char)speed2};
      SerialBT.write((const uint8_t*)command2, 2);
    } else if (xValue2 < 512) { // Left
      int speed2 = map(xValue2, 0, 511, 10, 100); // Map to 10-100% speed
      char command2[2] = {'A', (char)speed2};
      SerialBT.write((const uint8_t*)command2, 2);
    }
  } else {
    char command2[2] = {'A', 0};
    SerialBT.write((const uint8_t*)command2, 2);
  }

  delay(50); // Send commands at 20Hz
}

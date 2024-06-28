
#include <esp_now.h>
#include <WiFi.h>

// Define joystick pins
#define joystick2_x 32
#define joystick2_y 33
#define joystick1_x 34
#define joystick1_y 35
#define button1 14
#define button2 13
#define button3 27
#define LEDPIN 2
//tombol-tombol active low
#define SGRIP_ANGLE_HOLD 140
#define SGRIP_ANGLE_OPEN 0
#define SANGKAT_UP 0
#define SANGKAT_DOWN 180
int CenterX1;
int CenterY1;
int CenterX2;
int CenterY2;
int deadband = 80;
// State variables for joystick positions
int lastValueY = 0;
int b3state = 0;
long t = 0, tlastsend = 0;
int interval = 50;
int yaccelfwd = 20;
int ydecelfwd = 100;
int yaccelrev = 10;
int ydecelrev = 100;
int xValue1 = 0;
int yValue1 = 0;
int xValue2 = 0;
int yValue2 = 0;
// Receiver MAC Address
uint8_t broadcastAddress[] = { 0xCC, 0x7B, 0x5C, 0x26, 0xCC, 0xB4 };
//uint8_t broadcastAddress[] = { 0x24, 0xdc, 0xc3, 0x99, 0x10, 0x00 };
esp_now_peer_info_t peerInfo;

// Structure to send data
typedef struct struct_message {
  int M1;
  int M2;
  int servoA;
  int servoC;
  int servoS;
} struct_message;

// Create a struct_message called joystickData
struct_message joystickData;

// Math power forward
float mapToPower(float value, float maxpower) {
  float midpoint = 2048;
  if (fabs(value - midpoint) < deadband) return 0;
  float scaledValue = (value - midpoint) * maxpower / midpoint;
  return scaledValue;
}

void center() {
  //hitung titik tengah dari joystick
  CenterX1 = 0;
  CenterY1 = 0;
  CenterX2 = 0;
  CenterY2 = 0;
  delay(100);
  for (int i = 0; i < 128; i++) {
    CenterX1 += analogRead(joystick1_x);
    delayMicroseconds(50);
    CenterY1 += analogRead(joystick1_y);
    delayMicroseconds(50);
    CenterX2 += analogRead(joystick2_x);
    delayMicroseconds(50);
    CenterY2 += analogRead(joystick2_y);
    delayMicroseconds(50);
    //delay(1);
  }
  CenterX1 /= 128;
  CenterY1 /= 128;
  CenterX2 /= 128;
  CenterY2 /= 128;
}


// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  ledon();
}

void ledon() {
  digitalWrite(LEDPIN, HIGH);
}

void ledoff() {
  digitalWrite(LEDPIN, LOW);
}

void setup() {
  // Init Serial Monitor
  Serial.begin(115200);

  // Set up joystick pins
  pinMode(joystick1_x, INPUT);
  pinMode(joystick1_y, INPUT);
  pinMode(joystick2_x, INPUT);
  pinMode(joystick2_y, INPUT);
  //LED pin
  pinMode(LEDPIN, OUTPUT);
  ledoff();

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    ledon();
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
    ledon();
    return;
  }
  ledon();
  delay(200);
  ledoff();
  delay(200);
  ledon();
  delay(200);
  ledoff();
  delay(500);
  center();
}

void loop() {
  // Read joystick values
  t = millis();
  int newxValue1 = analogRead(joystick1_x) - (CenterX1 - 2048);
  delayMicroseconds(50);
  int newyValue1 = 4095 - (analogRead(joystick1_y) - (CenterY1 - 2048));
  delayMicroseconds(50);
  int newxValue2 = analogRead(joystick2_x) - (CenterX2 - 2048);
  delayMicroseconds(50);
  int newyValue2 = 4095 - (analogRead(joystick2_y) - (CenterY2 - 2048));
  delayMicroseconds(50);
  //rata-ratakan nilai baru dengan nilai lama untuk memperkecil fluktuasi
  xValue1 = (7 * xValue1 + newxValue1) / 8;
  yValue1 = (7 * yValue1 + newyValue1) / 8;
  xValue2 = (7 * xValue2 + newxValue2) / 8;
  yValue2 = (7 * yValue2 + newyValue2) / 8;
  // proses data untuk dikirim kalau sudah waktunya
  if (t - tlastsend >= interval) {
    tlastsend = t;
    // Map joystick values to power
    int scaledValueX = mapToPower(xValue1, 125);
    int scaledValueY = mapToPower(yValue1, 125);
    //discretization
    //dari 0 sampai 40 menghasilkan kecepatan 20
    //dari 40-99 menghasilkan kecepatan 40
    //untuk 100 baru kecepatannya 100
    if (scaledValueY > 0) {
      if (scaledValueY <= 40) scaledValueY = 20;
      else if (scaledValueY <= 99) scaledValueY = 40;
      else scaledValueY = 100;
    } else if (scaledValueY < 0) {
      if (scaledValueY >= -40) scaledValueY = -20;
      else if (scaledValueY >= -99) scaledValueY = -40;
      else scaledValueY = -100;
    }
    //batasi laju perubahan kecepatan dengan nilai yaccel
    if (scaledValueY >= 0) {                          //gerak maju
      if (scaledValueY > lastValueY) {                //perubahan power positif
        if (scaledValueY - lastValueY > yaccelfwd) {  //perubahannya besar
          scaledValueY = lastValueY + yaccelfwd;
        }
      } else {  //perubahan power negatif, melambat
        if (lastValueY - scaledValueY > ydecelfwd) {
          scaledValueY = lastValueY - ydecelfwd;
        }
      }
    } else {                            //gerak mundur
      if (scaledValueY < lastValueY) {  //makin negatif = makin cepat tapi mundur
        if (lastValueY - scaledValueY > yaccelrev) {
          scaledValueY = lastValueY - yaccelrev;
        }
      } else {  //power negatif tapi bertambah menuju 0 = mundur melambat
        if (scaledValueY - lastValueY > ydecelrev) {
          scaledValueY = lastValueY + ydecelrev;
        }
      }
    }
    lastValueY = scaledValueY;
    joystickData.M1 = (scaledValueX / 2) + scaledValueY;
    joystickData.M2 = (-scaledValueX / 2) + scaledValueY;
    joystickData.M1 = constrain(joystickData.M1, -100, 100);
    joystickData.M2 = constrain(joystickData.M2, -100, 100);
    if (digitalRead(button1) == LOW) {
      joystickData.servoA = SANGKAT_UP;
    }

    if (digitalRead(button2) == LOW) {
      joystickData.servoA = SANGKAT_DOWN;
    }

    if (digitalRead(button3) == LOW) {
      if (b3state == 0) {  //sebelumnya tidak ditekan
        b3state = 1;       //catat keadaan sekarang supaya tidak mengulang
        if (joystickData.servoC == SGRIP_ANGLE_HOLD) {
          joystickData.servoC = SGRIP_ANGLE_OPEN;
        } else {
          joystickData.servoC = SGRIP_ANGLE_HOLD;
        }
      }
    } else {  //kalau button tidak ditekan
      b3state = 0;
    }
    //joystick2 mapping value

    scaledValueX = mapToPower(xValue2, 100);
    scaledValueY = mapToPower(yValue2, 100);
    if (abs(scaledValueX) < 20 && abs(scaledValueY) < 20) {
      //joystick center, servo ke 90 derajat
      joystickData.servoS = 90;
    } else if (scaledValueY >= 0) {
      joystickData.servoS = atan2(scaledValueY, -scaledValueX) * 57.29578;
    } else {
      joystickData.servoS = atan2(-scaledValueY, -scaledValueX) * 57.29578;
    }

    /*
    Serial.print(xValue1);
    Serial.print(",");
    Serial.print(yValue1);
    Serial.print(",");
    Serial.print(xValue2);
    Serial.print(",");
    Serial.print(yValue2);
    Serial.print(",   ");
    Serial.print(joystickData.M1);
    Serial.print(",");
    Serial.print(joystickData.M2);
    Serial.print(",");
    Serial.print(joystickData.servoA);
    Serial.print(",");
    Serial.print(joystickData.servoC);
    Serial.print(", xy=");
    Serial.print(scaledValueX);
    Serial.print(",");
    Serial.print(scaledValueY);
    Serial.print(",s=");
  
    Serial.print(joystickData.servoS);
    Serial.println();
  */


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
    // Send joystick data via ESP-NOW
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&joystickData, sizeof(joystickData));
/*
    if (result == ESP_OK) {
      ledon();
      //Serial.println("Sent with success");
    } else {
      ledoff();
      //Serial.println("Error sending the data");
    }
*/
    //delay(25);  // Send data at 20Hz
    //ledoff();
    //delay(25);
  }
  if (t - tlastsend > 10) ledoff();
}

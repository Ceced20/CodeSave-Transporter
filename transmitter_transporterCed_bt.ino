#include <BluetoothSerial.h>

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
// State variables for joystick positions
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
int yaccel = 50;
int ydecel = 50;
int xValue1 = 0;
int yValue1 = 0;
int xValue2 = 0;
int yValue2 = 0;
// Receiver MAC Address
uint8_t recvAddress[] = {0xCC, 0x7B, 0x5C, 0x26, 0xCC, 0xB6};
const char pin[] = "1234";
String transmitter_btname = "BT-Transmitter";
bool connected = false;

BluetoothSerial btserial;

void ledon() {
  digitalWrite(LEDPIN, HIGH);
}

void ledoff() {
  digitalWrite(LEDPIN, LOW);
}
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
void setup() {
  // Init Serial Monitor
  Serial.begin(115200);

  // Set up joystick pins
  pinMode(joystick1_x, INPUT);
  pinMode(joystick1_y, INPUT);
  pinMode(joystick2_x, INPUT);
  pinMode(joystick2_y, INPUT);
  pinMode(LEDPIN, OUTPUT);
  ledoff();
  Serial.println("init bluetooth...");
  btserial.begin(transmitter_btname, true);
  btserial.setPin(pin);
  center();
  xValue1 = 2048;
  yValue1 = 2048;
  xValue2 = 2048;
  yValue2 = 2048;
  joystickData.servoA = SANGKAT_DOWN;
  joystickData.servoS = 90;
  joystickData.servoC = SGRIP_ANGLE_OPEN;
}

void loop() {


  //tunggu sampai connect
  if (!connected) {
    Serial.println("connecting...");
    connected = btserial.connect(recvAddress);
  }
  if (connected) {
    Serial.println("Connected successfully!");
    center();
  }
  while (btserial.connected()) {
    t = millis();
    //baca posisi joystick
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
    // Map joystick values to power
    if (t - tlastsend >= interval) {
      tlastsend = t;
      // Map joystick values to power
      int scaledValueX = mapToPower(xValue1, 125);
      int scaledValueY = mapToPower(yValue1, 125);
      //batasi laju perubahan kecepatan dengan nilai yaccel
      if (scaledValueY >= 0) { //gerak maju
        if (scaledValueY > lastValueY) { //perubahan power positif
          if (scaledValueY - lastValueY > yaccel) { //perubahannya besar
            scaledValueY = lastValueY + yaccel;
          }
        }
        else { //perubahan power negatif, melambat
          if (lastValueY - scaledValueY > ydecel) {
            scaledValueY = lastValueY - ydecel;
          }
        }
      }
      else { //gerak mundur
        if (scaledValueY < lastValueY) { //makin negatif = makin cepat tapi mundur
          if (lastValueY - scaledValueY > yaccel) {
            scaledValueY = lastValueY - yaccel;
          }
        }
        else { //power negatif tapi betambah menuju 0 = mundur melambat
          if (scaledValueY - lastValueY > ydecel) {
            scaledValueY = lastValueY + ydecel;
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
      Serial.print(xValue1); Serial.print(",");
      Serial.print(yValue1); Serial.print(",");
      Serial.print(xValue2); Serial.print(",");
      Serial.print(yValue2); Serial.print(",   ");
      Serial.print(joystickData.M1); Serial.print(",");
      Serial.print(joystickData.M2); Serial.print(",");
      Serial.print(joystickData.servoA); Serial.print(",");
      Serial.print(joystickData.servoC); Serial.print(",");
      Serial.print(joystickData.servoS); Serial.println();

      // Send joystick data via bluetooth
      joystickData.headerbytes = 0x2655; //yang dikirim byte kecil dulu (litle endian) jadi 0x55 lalu 0x26
      //btserial.println(joystickData.M1);
      btserial.write((uint8_t*) &joystickData, sizeof(joystickData));

    }
    if (t - tlastsend > 10) ledoff();
  }
  //keluar dari sini berarti putus koneksi
  connected = false;
}

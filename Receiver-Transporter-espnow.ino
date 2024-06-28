
#include <WiFi.h>
#include <esp_now.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <driver/mcpwm.h>  // Include the MCPWM driver
#include <driver/gpio.h>
#include <ESP32Servo.h>

//board: ESP32BOT3

//output motor: M1 kiri, M2 kanan
//kiri=13-12, kanan=17-16
#define M2A 16
#define M2B 17
#define M1A 12
#define M1B 13
#define PIN_SJEPIT 25
#define PIN_SANGKAT 23
#define PIN_SSETIR 26
#define PIN_LED 2
//#define TIMEOUT 500
#define SGRIP_ANGLE_HOLD 140
#define SGRIP_ANGLE_OPEN 0
#define SANGKAT_UP 0
#define SANGKAT_DOWN 180
#define SSETIROFFSET 0
//LED tambahan
#define T1 14
#define T2 15
#define T3 18
#define T4 19
//channel ledc:
#define PWMC1 15
#define PWMC2 14
#define PWMC3 13
#define PWMC4 12
#define TIMEOUT 500

typedef struct struct_message {
  int M1;
  int M2;
  int servoA;
  int servoC;
  int servoS;
} struct_message;

// Create a struct_message to hold incoming data
struct_message joystickData;

//deklarasi servo
Servo sjepit, sangkat, ssetir;

//variabel-variabel
long t = 0, tlastrecv = 0, tlastcalc = 0;
//urutan: xgyro, ygyro, zgyro, xaccel, yaccel, zaccel
int gyrooffsets[6] = { 159, 20, 1, -2695, -1034, 1278 };  //robot trirover-swerve
MPU6050 mpu;
Quaternion q;
VectorFloat grav;
uint8_t packetsize, fifocount;
float ypr[3];  //yaw pitch roll
uint8_t fifobuffer[64];
float arahhadap = 0, deltaarah = 0, targetarah = 0;
int adagyro = 0;
int use_PID = 1;
int m1power = 0, m2power = 0, deltamotor = 0;
int motorenable = 0;  //akan enable begitu ada data yang diterima, akan disable ketika lama tidak ada data (timeout)
int timeout = 120;
//=========fungsi-fungsi untuk motor==========
void mcpwm_initialize() {
  // Initialize GPIO for MCPWM
  mcpwm_config_t conf;
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, M1A);  //hubungkan unit PWM 0, fungsi yang mana, ke pin mana
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, M1B);  //kiri
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, M2A);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1B, M2B);  //kanan
  conf.frequency = 1000;
  conf.cmpr_a = 0;
  conf.cmpr_b = 0;
  conf.duty_mode = MCPWM_DUTY_MODE_0;  //active high
  conf.counter_mode = MCPWM_UP_COUNTER;
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &conf);
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &conf);
  mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_0);
  mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_1);
}
void updatemotor(int m1, int m2) {
  //kiri
  if (m1 >= 0) {
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, m1);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0);
  } else {
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, -m1);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 0);
  }
  //kanan
  if (m2 >= 0) {
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, m2);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, 0);
  } else {
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, -m2);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, 0);
  }
}
//======PID===============
class PIDController {
  float KP = 0, KI = 0, KD = 0, Omax = 0, Imax = 0;
  float Iacc = 0, selisihsebelum = 0;
  float t_, tireset;
  float emin;
  int resetimode;
public:
  PIDController(float kp, float ki, float kd, float outmax, float imax) {
    KP = kp;
    KI = ki;
    KD = kd;
    Imax = imax;
    Omax = outmax;
    t_ = 0;
    tireset = 0;
    resetimode = 0;
    emin = 0;
  }
  float calc(float nilai, float target, float dt) {
    float selisih = nilai - target;
    //float Ppart = KP * selisih;
    //if (Ppart > Imax)Ppart = Imax; else if (Ppart < -Imax)Ppart = -Imax;
    //float newIacc = Iacc + selisih * KI * dt;
    t_ += dt;
    if (resetimode) {
      if (selisih * selisihsebelum < 0) {
        Iacc = 0;
      }
      if (tireset > 0) {
        if (t_ > tireset) {
          t_ -= tireset;
          Iacc = 0;
        }
      }
    } else {
      Iacc += selisih * KI * dt;
    }
    if (emin > 0) {
      if (fabs(selisih) > emin) {
        Iacc = 0;
      }
    }
    //float Iaccmax = Imax - fabs(Ppart);
    //if (Iaccmax < 0)Iaccmax = 0;
    if (Iacc > Imax) Iacc = Imax;
    else if (Iacc < -Imax) Iacc = -Imax;

    float diff = (selisih - selisihsebelum) / dt;
    selisihsebelum = selisih;
    //float diff = (nilai - nilaisebelum) / dt;
    //nilaisebelum = nilai;
    //Serial.println(diff*10000);
    float output = KP * selisih + Iacc + KD * diff;
    if (output > Omax) output = Omax;
    else if (output < -Omax) output = -Omax;
    return output;
  }
  void setKP(float x) {
    KP = x;
  }
  void setKI(float x) {
    KI = x;
  }
  void setKD(float x) {
    KD = x;
  }
  void setIreset(int mode) {
    resetimode = mode;
  }
  void setTIreset(float t) {
    t_ = t;
  }
  void setEminInt(float e) {
    emin = e;
  }
  void setOutmax(float x) {
    Omax = x;
  }
  void setImax(float x) {
    Imax = x;
  }
  float getKP() {
    return KP;
  }
  float getKI() {
    return KI;
  }
  float getKD() {
    return KD;
  }
  void reset() {
    Iacc = 0;
    selisihsebelum = 0;
  }
};
//PIDController depan(65, 0.5, 16000, 80, 50); //KP, KI, KD, outmax,Imax
PIDController depan(50, 0, 10000, 80, 50);  //KP, KI, KD, outmax,Imax

class RateLimiter {
private:
  int target;
  int posisisekarang;
  float r;
  long *timervar;
  long lastt, updateinterval;
public:
  RateLimiter(float maxrate, long *Timervar) {
    r = maxrate;  //satuannya dalam point per milisekon
    timervar = Timervar;
    posisisekarang = 0;
    updateinterval = 2;
  }
  void jumpto(int posisi) {
    posisisekarang = posisi;
    target = posisi;
  }
  int update(int newtarget) {
    target = newtarget;
    //long tt = millis();
    if ((*timervar - lastt) > updateinterval) {
      float selisih = (*timervar - lastt) * r;
      lastt = *timervar;
      if (target > posisisekarang) {
        if (target - posisisekarang > selisih) {
          posisisekarang += selisih;
        } else {
          posisisekarang = target;
        }
      } else {
        if (posisisekarang - target > selisih) {
          posisisekarang -= selisih;
        } else {
          posisisekarang = target;
        }
      }
      //Serial.println(selisih);
    }
    return posisisekarang;
  }
};
//untuk mencegah perubahan power mendadak me-reset ESP32:
RateLimiter m1ratelimit(1, &t);  //1 point per milisekon = 100 poin dalam 100ms
RateLimiter m2ratelimit(1, &t);

// Callback function executed when data is received
//untuk versi 5.1.4 ke atas:
//void OnDataRecv(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
//untuk versi 5.0.6 ke bawah:
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  ledon();
  // Cast the incomingData pointer to the correct struct_message type
  //const struct_message *data = reinterpret_cast<const struct_message *>(incomingData);
  memcpy(&joystickData, incomingData, sizeof(joystickData));
  //motor 1
  //gunakan PID untuk mengatur arah hadap, jadi jangan langsung output M1-M2 ke motor
  if (use_PID) {
    m1power = (joystickData.M1 + joystickData.M2) / 2;
    m2power = m1power;
    //selisih M1-M2  menentukan perubahan arah hadap
    //M1 kiri, M2 kanan
    //maks 180 derajat (= pi radian) dalam 1000 ms
    //interval pengiriman data = 50 ms, jadi perubahan sudut maks = pi/20 rad per interval data
    //perintah ini mengubah target arah hadap, tidak mengubah power motor. PID yang akan menghitung power motor
    putardelta((joystickData.M1 - joystickData.M2) * 1.5708e-3);  //0.15708/100;
  } else {
    m1power = joystickData.M1;
    m2power = joystickData.M2;
    deltamotor = 0;
    0;
  }
  ledcWrite(PWMC1, 255 - 2 * max(abs(m1power), abs(m2power)));
  sjepit.write(joystickData.servoC);
  sangkat.write(joystickData.servoA);
  int ssetirwrite = joystickData.servoS + SSETIROFFSET;
  ssetirwrite = constrain(ssetirwrite, 0, 180);
  ssetir.write(ssetirwrite);
  motorenable = 1;
  //catat waktu terakhir receive
  tlastrecv = t;
}

void ledon() {
  digitalWrite(PIN_LED, HIGH);
}
void ledoff() {
  digitalWrite(PIN_LED, LOW);
}
void noswerve() {
  ssetir.write(90 + SSETIROFFSET);
}
void putardelta(float delta) {
  targetarah = targetarah + delta;
  if (targetarah >= M_PI) {
    targetarah -= (2 * M_PI);
  } else if (targetarah <= -M_PI) {
    targetarah += (2 * M_PI);
  }
}
int sign(int a) {
  if (a >= 0) return 1;
  else return -1;
}

void stopmovement() {
  m1power = 0;
  m2power = 0;
  noswerve();
  motorenable = 0;
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_LED, OUTPUT);
  ledoff();
  Wire.begin(21, 22, 400000);

  // Initialize MCPWM
  mcpwm_initialize();

  //attach servo pins
  sjepit.attach(PIN_SJEPIT);
  sangkat.attach(PIN_SANGKAT);
  ssetir.attach(PIN_SSETIR);
  sjepit.write(140);  //nilai posisi awal
  sangkat.write(180);
  ssetir.write(90 + SSETIROFFSET);
  //=======LED indikator========
  ledcSetup(PWMC1, 1000, 8);
  ledcSetup(PWMC2, 1000, 8);
  ledcSetup(PWMC3, 1000, 8);
  ledcSetup(PWMC4, 1000, 8);
  ledcAttachPin(14, PWMC1);
  ledcAttachPin(15, PWMC2);
  ledcAttachPin(18, PWMC3);
  ledcAttachPin(19, PWMC4);
  for (int i = 0; i < 255; i++) {
    int t1 = i * 2;
    int t2 = t1 - 40;
    int t3 = t1 - 80;
    int t4 = t1 - 120;
    t1 = constrain(t1, 0, 255);
    t2 = constrain(t2, 0, 255);
    t3 = constrain(t3, 0, 255);
    t4 = constrain(t4, 0, 255);
    ledcWrite(PWMC1, 256 - t1);
    ledcWrite(PWMC2, 256 - t2);
    ledcWrite(PWMC3, 256 - t3);
    ledcWrite(PWMC4, 256 - t4);
    delay(2);
  }
  ledcWrite(PWMC1, 256);
  ledcWrite(PWMC2, 256);
  ledcWrite(PWMC3, 256);
  ledcWrite(PWMC4, 256);
  //=========gyro======================
  if (mpu.testConnection()) {
    Serial.println("MPU6050 koneksi OK");
  } else {
    Serial.println("MPU connection failed");
    ledcWrite(PWMC1, 255);
    ledcWrite(PWMC2, 255);
    ledcWrite(PWMC3, 255);
    ledcWrite(PWMC4, 0);
  }
  if (mpu.dmpInitialize() == 0) {
    Serial.println("DMP OK");
    mpu.setXGyroOffset(gyrooffsets[0]);
    mpu.setYGyroOffset(gyrooffsets[1]);
    mpu.setZGyroOffset(gyrooffsets[2]);
    mpu.setXAccelOffset(gyrooffsets[3]);
    mpu.setYAccelOffset(gyrooffsets[4]);
    mpu.setZAccelOffset(gyrooffsets[5]);
    mpu.setDMPEnabled(true);
    packetsize = mpu.dmpGetFIFOPacketSize();
    adagyro = 1;
  }
  //setelan PID
  depan.setEminInt(10 * M_PI / 180);
  Serial.println("Init ESP_NOW...");
  // Initialize WiFi in Station mode
  WiFi.mode(WIFI_STA);
  Serial.println("ESPNow Receiver");
  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    ledon();
    return;
  }

  // Register receive callback
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  // Main loop logic
  t = millis();
  long deltat = t - tlastcalc;
  if (deltat > 10) {
    tlastcalc = t;
    if (adagyro) {
      if (mpu.dmpGetCurrentFIFOPacket(fifobuffer)) {
        mpu.dmpGetQuaternion(&q, fifobuffer);
        mpu.dmpGetGravity(&grav, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &grav);
        arahhadap = ypr[0];  //dalam radian
        //Serial.println(arahhadap);
        //kalau dekat batas +- pi:
        if (targetarah > M_PI_2) {
          if (arahhadap < 0) arahhadap += 2 * M_PI;
        } else if (targetarah < -M_PI_2) {
          if (arahhadap > 0) arahhadap -= 2 * M_PI;
        }
        //float deltat = t - tlastcalc;
        float powerputar = -depan.calc(arahhadap, targetarah, deltat);  //positif = putar kanan
        //kalau arah hadap < target (kurang ke kanan), selisih = negatif, hasil PID calc = negatif, output harus positif
        float faktorputar = (90 - fabs(joystickData.servoS - 90)) / 90.;
        deltamotor = powerputar * faktorputar;

        ledcWrite(PWMC3, 255 - fabs(powerputar * 2.55));
      }
    } else {  //diminta pakai gyro, tapi malfungsi
      deltamotor = (joystickData.M2 - joystickData.M1) / 2;
      ledcWrite(PWMC3, 255 - abs(deltamotor));
    }
    //kalau komunikasi putus atau tidak ada data yang diterima setelah sekian lama, matikan motor
    if (t - tlastrecv > timeout) {
      stopmovement();
    } else if (t - tlastrecv > 2) {  //matikan LED sesudah 2 ms sejak terakhir recv
      ledoff();
    }
    //update motor
    //putar positif ke kanan, power kiri tambahkan, power kanan kurangi
    if (motorenable) {
      int powerkiri = m1power + deltamotor;
      int powerkanan = m2power - deltamotor;
      if (powerkiri > 100) powerkiri = 100;
      else if (powerkiri < -100) powerkiri = -100;
      if (powerkanan > 100) powerkanan = 100;
      else if (powerkanan < -100) powerkanan = -100;
      powerkiri = m1ratelimit.update(powerkiri);
      powerkanan = m2ratelimit.update(powerkanan);
      updatemotor(powerkiri, powerkanan);
    } else {
      updatemotor(0, 0);
      targetarah = arahhadap;
    }
  }
}

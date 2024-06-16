#include <WiFi.h>
#include <esp_now.h>
#include <driver/mcpwm.h>  // Include the MCPWM driver
#include <driver/gpio.h>
#include <ESP32Servo.h>

// Define motor pins
#define m1kanan 32
#define m2kanan 33
#define m1kiri 18
#define m2kiri 19
#define PIN_SJEPIT 16
#define PIN_SANGKAT 17
#define PIN_SSETIR 5
#define PIN_LED 2
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
long t=0, tlastrecv=0;

// Callback function executed when data is received
//untuk versi 5.1.4 ke atas:
//void OnDataRecv(const esp_now_peer_info *info, const uint8_t *incomingData, int len) {
//untuk versi 5.0.6 ke bawah:
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  ledon();
  // Cast the incomingData pointer to the correct struct_message type
  const struct_message *data = reinterpret_cast<const struct_message *>(incomingData);
  //motor 1
  int p = data->M1;
  if (p >= 0) {  //maju
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, p);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 0);
  } else {  //mundur
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, p);
  }
  p = data->M2;
  if (p >= 0) {  //maju
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, p);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, 0);
  } else {  //mundur
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, 0);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, p);
  }
  sjepit.write(data->servoC);
  sangkat.write(data->servoA);
  ssetir.write(data->servoS);
  //catat waktu terakhir receive
  tlastrecv = t;
}

void ledon(){
  digitalWrite(PIN_LED,HIGH);
}
void ledoff(){
  digitalWrite(PIN_LED,LOW);
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_LED,OUTPUT);
  ledoff();
  // Initialize WiFi in Station mode
  WiFi.mode(WIFI_STA);
  Serial.println("ESPNow Receiver");

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register receive callback
  esp_now_register_recv_cb(OnDataRecv);

  // Initialize MCPWM
  mcpwm_initialize();

  //attach servo pins
sjepit.attach(PIN_SJEPIT);
sangkat.attach(PIN_SANGKAT);
ssetir.attach(PIN_SSETIR);
}
  //servowrite


void mcpwm_initialize() {
  // Initialize GPIO for MCPWM
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, m1kanan);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, m2kanan);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, m1kiri);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1B, m2kiri);


  // Configure MCPWM
  mcpwm_config_t conf;
  conf.frequency = 1000;  // Frequency = 1kHz
  conf.cmpr_a = 0;        // Duty cycle of PWMxA = 0
  conf.cmpr_b = 0;        // Duty cycle of PWMxB = 0
  conf.counter_mode = MCPWM_UP_COUNTER;
  conf.duty_mode = MCPWM_DUTY_MODE_0;

  // Initialize MCPWM0A and MCPWM1A with above settings
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &conf);
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &conf);

  // Start the MCPWM timers
  mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_0);
  mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_1);
}



void loop() {
  // Main loop logic
  t = millis();
  //kalau komunikasi putus atau tidak ada data yang diterima setelah sekian lama, matikan motor
  if(t-tlastrecv > TIMEOUT){
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 0);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, 0);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, 0);
  }
  else if(t-tlastrecv > 2){ //matikan LED sesudah 2 ms sejak terakhir recv
    ledoff();
  }
}

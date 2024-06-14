#include <WiFi.h>
#include <esp_now.h>
#include <driver/mcpwm.h>  // Include the MCPWM driver
#include <driver/gpio.h>
#include <ESP32Servo.h>

// Define motor pins
#define MOTOR_RIGHT_PWM 18
#define MOTOR_RIGHT_DIR 5
#define MOTOR_LEFT_PWM 18
#define MOTOR_LEFT_DIR 5

void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len);

void setup() {
  Serial.begin(115200);
  
  // Initialize WiFi in Station mode
  WiFi.mode(WIFI_STA);
  Serial.println("ESPNow Receiver");
  
  // Initialize ESP-NOW
  if (esp_now_init()!= ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register receive callback
  esp_now_register_recv_cb(OnDataRecv);

  // Initialize MCPWM
  mcpwm_initialize();
}

void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  // Handle received data
  Serial.print("Bytes received: ");
  Serial.println(len);
  
  // Add code here to process received data
  // For example, let's assume the received data is a motor speed command
  int motor;
  float speed;
  memcpy(&motor, data, sizeof(int));
  memcpy(&speed, data + sizeof(int), sizeof(float));
  set_motor_speed(motor, speed);
}

void mcpwm_initialize() {
  // Initialize GPIO for MCPWM
  gpio_config_t io_conf;
  io_conf.intr_type = GPIO_INTR_DISABLE;
  io_conf.mode = GPIO_MODE_OUTPUT;
  io_conf.pin_bit_mask = (1ULL << MOTOR_RIGHT_PWM) | (1ULL << MOTOR_LEFT_PWM);
  gpio_config(&io_conf);

  // Configure MCPWM
  mcpwm_config_t conf;
  conf.frequency = 1000;    // Frequency = 1kHz
  conf.cmpr_a = 0;          // Duty cycle of PWMxA = 0
  conf.cmpr_b = 0;          // Duty cycle of PWMxB = 0
  conf.counter_mode = MCPWM_UP_COUNTER;
  conf.duty_mode = MCPWM_DUTY_MODE_0; 

  // Initialize MCPWM0A and MCPWM1A with above settings
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &conf);
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &conf);

  // Start the MCPWM timers
  mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_0);
  mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_1);
}

void set_motor_speed(int motor, float speed) {
  if (motor == 0) {
    // Set right motor direction and speed
    gpio_set_level(MOTOR_RIGHT_DIR, speed >= 0? 0 : 1);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, abs(speed) * 100);
  } else {
    // Set left motor direction and speed
    gpio_set_level(MOTOR_LEFT_DIR, speed >= 0? 0 : 1);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, abs(speed) * 100);
  }
}

void loop() {
  // Main loop logic
}

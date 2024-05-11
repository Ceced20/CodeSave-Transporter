#include <driver/mcpwm.h>

#define  M1Forward 32
#define  M1Backward 33
#define  M2Forward 18
#define  M2Backward 19

// Define Variables
int speed1 = 0;
int speed2 = 0;
int ch0, ch1, ch2, ch3, ch6;

void setup() {
  Serial.begin(9600);
  pinMode(M1Forward, OUTPUT);
  pinMode(M1Backward, OUTPUT);
  pinMode(M2Forward, OUTPUT);
  pinMode(M2Backward, OUTPUT);
  // Initialize MCPWM

  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, M1Forward);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, M1Backward);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, M2Forward);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1B, M2Backward);

  mcpwm_config_t pwm_config;
  pwm_config.frequency = 1000; // Set the frequency of the PWM signal
  pwm_config.cmpr_a = 0; // Set the duty cycle of the PWM signal
  pwm_config.cmpr_b = 0; // Set the duty cycle of the PWM signal
  pwm_config.counter_mode = MCPWM_UP_COUNTER;
  pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);
  mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_0);
  mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_1);

}

void loop() {
  if (Serial.available() > 0) {
    ch0 = Serial.read();
    if (ch0 == 'W' || ch0 == 'w') {
      // Move Forward
      speed1 = 100;
      speed2 = 100;
      mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, speed1);
      mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 0);
      mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, speed2);
      mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, 0);
    } else if (ch0 == 'S' || ch0 == 's') {
      // Move Backward
      speed1 = 100;
      speed2 = 100;
      mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0);
      mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, speed1);
      mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, 0);
      mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, speed2);
    } else if (ch0 == 'A' || ch0 == 'a') {
      // Turn Left
      speed1 = 100;
      speed2 = 0;
      mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, speed1);
      mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 0);
      mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, 0);
      mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, speed2);
    } else if (ch0 == 'D' || ch0 == 'd') {
      // Turn Right
      speed1 = 0;
      speed2 = 100;
      mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, speed1);
      mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 0);
      mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, speed2);
      mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, 0);
    } else if (ch0 == 'X' || ch0 == 'x') {
      // Stop
      speed1 = 0;
      speed2 = 0;
      mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, speed1);
      mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 0);
      mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, speed2);
      mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, 0);
    }
  }
}

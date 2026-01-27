/***
 * Group: 8
 *
 * Members: Alice Ahlberg
 *          Daniel Fjelkner
 *          David Georgian Iosifescu
 *
 * Course code: MF2103
 *
 * Task description: Lab 1 - Peripherals Driver
 *                   Peripheral driver functions for motor control and encoder
 * reading.
 *
 * Compiler: ARM GCC
 *
 * Other information: Handles GPIO, PWM generation, and Encoder reading with
 * basic filtering.
 *
 * References: Course material MF2103
 *
 ***/

#include "peripherals.h"

#define RESOLUTION 2048

int16_t encoder; // Global variable, can be used for debugging purposes
static int32_t rpm_filt = 0;
static uint8_t vel_initialized = 0;

/* Enable both half-bridges to drive the motor */
void Peripheral_GPIO_EnableMotor(void) {
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
  return;
}

/* Disable both half-bridges to stop the motor */
void Peripheral_GPIO_DisableMotor(void) {
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
  return;
}

/* Drive the motor in both directions */
void Peripheral_PWM_ActuateMotor(int32_t vel) {
  uint32_t arr = TIM3->ARR;

  // Saturate vel to [-2^30, 2^30]
  const int32_t VEL_MAX = (1L << 30);
  if (vel > VEL_MAX)
    vel = VEL_MAX;
  if (vel < -VEL_MAX)
    vel = -VEL_MAX;

  if (vel == 0) {
    TIM3->CCR1 = 0;
    TIM3->CCR2 = 0;
    return;
  }

  int32_t duty_cycle;
  uint32_t duty_abs;

  if (vel > 0) {
    // Convert velocity (scaled 2^30) to duty cycle (timer counts)
    // Formula: duty = (vel / 2^30) * ARR
    duty_cycle = (int32_t)(((int64_t)vel * (int64_t)arr) >> 30);

    if (duty_cycle > (int32_t)arr)
      duty_cycle = (int32_t)arr;
    if (duty_cycle < 0)
      duty_cycle = 0;

    TIM3->CCR1 = (uint16_t)duty_cycle;
    TIM3->CCR2 = 0;
  } else {
    duty_abs = (uint32_t)(-vel); // safe now, since vel >= -2^30

    duty_cycle = (int32_t)(((int64_t)duty_abs * (int64_t)arr) >> 30);

    if (duty_cycle > (int32_t)arr)
      duty_cycle = (int32_t)arr;
    if (duty_cycle < 0)
      duty_cycle = 0;

    TIM3->CCR1 = 0;
    TIM3->CCR2 = (uint16_t)duty_cycle;
  }
}

/* Read the encoder value and calculate the current velocity in RPM */
int32_t Peripheral_Encoder_CalculateVelocity(uint32_t ms) {
  static uint32_t last_ms = 0;

  // First call: initialize timestamp and filter, return 0
  if (!vel_initialized) {
    last_ms = ms;
    rpm_filt = 0;
    vel_initialized = 1;

    // Reset counter for clean start
    TIM1->EGR |= TIM_EGR_UG;
    return 0;
  }

  // Time span since last call
  uint32_t dt_ms = ms - last_ms;
  last_ms = ms;

  // If no time passed, keep previous filtered RPM
  if (dt_ms == 0)
    return rpm_filt;

  // Read raw timer counts and cast to signed 16-bit to handle
  // overflow/underflow correctly Negate because encoder direction is opposite
  // to motor drive
  encoder = -(int16_t)(TIM1->CNT & 0xFFFF);

  // Reset counter for next interval
  TIM1->EGR |= TIM_EGR_UG;

  // -------------------------------------------------------------------------
  // Instantaneous RPM:
  //   RPM = counts * 60000 / (RESOLUTION * dt_ms)
  //   60000 = 60 s/min * 1000 ms/s
  // -------------------------------------------------------------------------
  int64_t num = (int64_t)encoder * 60000;             // counts * 60000
  int64_t den = (int64_t)RESOLUTION * (int64_t)dt_ms; // CPR * dt_ms

  if (den == 0)
    return rpm_filt;

  int32_t rpm = (int32_t)(num / den);

  // Apply IIR low-pass filter to smooth RPM
  // Formula: Y[n] = alpha*X[n] + (1-alpha)*Y[n-1]
  // Implemented as: (alpha_num * rpm + (alpha_den - alpha_num) * rpm_filt) /
  // alpha_den
  {
    const int32_t alpha_num = 1; // alpha = 1/4
    const int32_t alpha_den = 10;

    rpm_filt =
        (alpha_num * rpm + (alpha_den - alpha_num) * rpm_filt) / alpha_den;
  }

  return rpm_filt;
}

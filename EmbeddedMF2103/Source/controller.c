#include "controller.h"
#include <stdint.h>

// Controller gains
// Kp: [control units / RPM]
// Ki: [control units / (RPM * second)]
#define KP 300000
#define KI 400000

#define CONTROL_MAX 1073741823L
#define CONTROL_MIN (-1073741824L)

// Internal state
static int64_t integrator = 0;
static uint32_t time_prev = 0;
static uint8_t first_call_after_reset = 1;

int32_t Controller_PIController(const int32_t *ref, const int32_t *meas,
                                const uint32_t *ms) {
  if (!ref || !meas || !ms)
    return 0;

  // First call: initialize timing
  if (first_call_after_reset) {
    time_prev = *ms;
    integrator = 0;
    first_call_after_reset = 0;
    return 0;
  }

  // Time step (ms)
  uint32_t now_ms = *ms;
  uint32_t dt_ms = now_ms - time_prev;
  time_prev = now_ms;

  if (dt_ms == 0)
    return 0;

  // Error in RPM
  int32_t error = *ref - *meas;

  // Proportional term (calc in 64-bit to avoid overflow)
  int64_t p_term = (int64_t)KP * (int64_t)error;

  // Integral term
  // I += Ki * error * dt
  // dt = dt_ms / 1000
  int64_t i_increment = (int64_t)KI * (int64_t)error * (int64_t)dt_ms / 1000;

  integrator += i_increment;

  // PI output
  int64_t control_64 = p_term + integrator;

  // Saturate output and update integrator (anti-windup)
  if (control_64 > CONTROL_MAX) {
    control_64 = CONTROL_MAX;
    integrator = CONTROL_MAX - p_term;
  } else if (control_64 < CONTROL_MIN) {
    control_64 = CONTROL_MIN;
    integrator = CONTROL_MIN - p_term;
  }

  return (int32_t)control_64;
}

void Controller_Reset(void) {
  integrator = 0;
  time_prev = 0;
  first_call_after_reset = 1;
}

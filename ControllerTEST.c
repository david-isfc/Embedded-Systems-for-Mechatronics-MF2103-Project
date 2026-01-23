#include "controller.h"

// Controller gains (to be tuned)
// These values are starting points and should be tuned based on system response
// Kp: Proportional gain in units of (control units) / (error units)
// Ki: Integral gain in units of (control units) / (error units * ms)
// Gains are scaled to allow fine-tuning with integer arithmetic
#define KP_NUMERATOR 1000		// Proportional gain numerator (scaled by 1000)
#define KP_DENOMINATOR 1		// Proportional gain denominator
#define KI_NUMERATOR 50			// Integral gain numerator (scaled by 1000, units: control/(error*ms))
#define KI_DENOMINATOR 1		// Integral gain denominator

// Anti-windup limits
// Maximum control signal range: -1,073,741,824 to +1,073,741,823
#define CONTROL_MAX 1073741823L
#define CONTROL_MIN (-1073741824L)

// Internal state variables
static int64_t integrator = 0;		// Integrator state (64-bit to prevent overflow)
static uint32_t time_prev = 0;		// Previous time reading
static uint8_t first_call_after_reset = 1;	// Flag for first call after reset

// Mock controller variable (for testing without controller)
int32_t Controller_MockControl = 268435456;	// Default: 25% duty cycle (can be changed in debugger)

int32_t Controller_PIController(const int32_t* reference, const int32_t* measured, const uint32_t* millisec)
{
	// Return 0 on first call after reset (as per requirements)
	if (first_call_after_reset)
	{
		time_prev = *millisec;
		first_call_after_reset = 0;
		return 0;
	}
	
	// Calculate time difference in milliseconds
	uint32_t dt_ms = *millisec - time_prev;
	
	// Avoid division by zero or invalid time
	if (dt_ms == 0)
	{
		// If no time has passed, return previous output (but we don't store it)
		// For safety, return 0
		return 0;
	}
	
	// Calculate error
	int32_t error = *reference - *measured;
	
	// Proportional term: P = Kp * error
	// Kp = KP_NUMERATOR / KP_DENOMINATOR
	int64_t p_term = ((int64_t)KP_NUMERATOR * (int64_t)error) / KP_DENOMINATOR;
	
	// Integral term: I = Ki * error * dt_ms
	// Ki = KI_NUMERATOR / KI_DENOMINATOR (units: control/(error*ms))
	// dt_ms is in milliseconds
	// So: I_increment = (Ki * error * dt_ms) / KI_DENOMINATOR
	// Since Ki is scaled, we divide by the scaling factor (1000) to get proper units
	int64_t i_increment = ((int64_t)KI_NUMERATOR * (int64_t)error * (int64_t)dt_ms) / (KI_DENOMINATOR * 1000);
	
	// Update integrator
	integrator += i_increment;
	
	// Calculate control signal: control = P + I
	int64_t control_64 = p_term + integrator;
	
	// Apply anti-windup: clamp the integrator if control signal saturates
	// This prevents integrator from growing unbounded
	if (control_64 > CONTROL_MAX)
	{
		// Control would saturate positive, clamp integrator
		integrator = CONTROL_MAX - p_term;
		control_64 = CONTROL_MAX;
	}
	else if (control_64 < CONTROL_MIN)
	{
		// Control would saturate negative, clamp integrator
		integrator = CONTROL_MIN - p_term;
		control_64 = CONTROL_MIN;
	}
	
	// Update previous time
	time_prev = *millisec;
	
	// Return 32-bit control signal
	return (int32_t)control_64;
}

void Controller_Reset(void)
{
	// Reset internal state variables
	integrator = 0;
	time_prev = 0;
	first_call_after_reset = 1;
}

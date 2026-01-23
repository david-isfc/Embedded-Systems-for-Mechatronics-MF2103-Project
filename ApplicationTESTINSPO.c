#include "peripherals.h"

// Encoder resolution: number of encoder counts per revolution
// NOTE: This must be determined from your encoder datasheet!
// Typical quadrature encoders: if encoder has N pulses per revolution,
// encoder mode counts 4*N counts per revolution (due to quadrature decoding)
// Example: 11 pulses/rev -> 44 counts/rev
// TODO: Update RESOLUTION based on your actual encoder specifications
#define RESOLUTION 44		// Number of encoder counts per revolution (pulses * 4 for quadrature)

// Global variables for encoder velocity calculation
int16_t encoder;			// Global variable, can be used for debugging purposes
static int16_t encoder_prev = 0;	// Previous encoder reading
static uint32_t time_prev = 0;		// Previous time reading
static uint8_t first_call = 1;		// Flag for first call

/* Enable both half-bridges to drive the motor */
void Peripheral_GPIO_EnableMotor(void)
{
	// Set GPIO Port A pins PA0 and PA1 high to enable both half-bridges
	// NOTE: Verify pin assignments based on your hardware configuration
	// These pins should correspond to the BTN8982 shield enable pins
	// Check STM32 pinout diagram to confirm PA0/PA1 mapping
	// Assuming PA0 and PA1 are configured as outputs (done in Task 0)
	GPIOA->BSRR = GPIO_BSRR_BS0 | GPIO_BSRR_BS1;
}

/* Disable both half-bridges to stop the motor */
void Peripheral_GPIO_DisableMotor(void)
{
	// Set GPIO Port A pins PA0 and PA1 low to disable both half-bridges
	// NOTE: Verify pin assignments match EnableMotor function
	GPIOA->BSRR = GPIO_BSRR_BR0 | GPIO_BSRR_BR1;
}

/* Drive the motor in both directions */
void Peripheral_PWM_ActuateMotor(int32_t control)
{
	// Input range: -1,073,741,824 to +1,073,741,823 (31-bit signed)
	// This represents -100% to +100% duty cycle
	// Need to scale to Timer 3 CCR register values (16-bit, 0 to ARR)
	
	// Timer 3 ARR (Auto-reload register) value determines max PWM period
	// Read ARR value directly from the register (configured in Task 0)
	// NOTE: Timer 3 CH1 and CH2 are assumed to be configured for PWM
	// Verify channel assignments match your hardware configuration
	uint32_t arr = TIM3->ARR;
	
	if (control == 0)
	{
		// Motor stationary: set both PWM channels to 0
		TIM3->CCR1 = 0;
		TIM3->CCR2 = 0;
		return;
	}
	
	// Scale control signal to duty cycle using efficient bit-shifting
	// Input range: -1,073,741,824 to +1,073,741,823 (which is -2^30 to 2^30-1)
	// We want to map this to [0, ARR] for PWM duty cycle
	// 
	// Efficient bit-shifting approach:
	// Instead of dividing by 1,000,000,000 (approximation), we divide by 2^30 exactly
	// Dividing by 2^30 is equivalent to right-shifting by 30 bits: value >> 30
	// This is much faster than division and gives exact scaling
	// 
	// Formula: duty = (control * ARR) / 2^30
	// Using bit-shift: duty = (control * ARR) >> 30
	// 
	// Example: control = 1,073,741,823 (100% = 2^30 - 1)
	//   duty = (1,073,741,823 * ARR) >> 30 ≈ ARR (slightly less, which is correct)
	
	int32_t duty_cycle;
	uint32_t duty_abs;
	
	if (control > 0)
	{
		// Clockwise: CH1 active, CH2 = 0
		// Scale: duty = (control * ARR) >> 30
		// Using 64-bit intermediate to avoid overflow during multiplication
		duty_cycle = (int32_t)(((int64_t)control * (int64_t)arr) >> 30);
		// Clamp to valid range
		if (duty_cycle > (int32_t)arr)
			duty_cycle = (int32_t)arr;
		if (duty_cycle < 0)
			duty_cycle = 0;
		// Set PWM: CH1 for forward, CH2 off
		TIM3->CCR1 = (uint16_t)duty_cycle;
		TIM3->CCR2 = 0;
	}
	else if (control < 0)
	{
		// Counter-clockwise: CH2 active, CH1 = 0
		// Scale: duty = (|control| * ARR) >> 30
		// Make control positive first, then apply same bit-shift scaling
		duty_abs = (uint32_t)(-control);  // Make positive (control is negative)
		duty_cycle = (int32_t)(((int64_t)duty_abs * (int64_t)arr) >> 30);
		// Clamp to valid range
		if (duty_cycle > (int32_t)arr)
			duty_cycle = (int32_t)arr;
		if (duty_cycle < 0)
			duty_cycle = 0;
		// Set PWM: CH2 for reverse, CH1 off
		TIM3->CCR1 = 0;
		TIM3->CCR2 = (uint16_t)duty_cycle;
	}
}

/* Read the encoder value and calculate the current velocity in RPM */
int32_t Peripheral_Encoder_CalculateVelocity(uint32_t millisec)
{
	// Read Timer 1 counter register (16-bit signed value)
	// Timer 1 is configured in encoder mode, CNT register contains the count
	int16_t encoder_current = (int16_t)(TIM1->CNT);
	
	// Store current reading for debugging
	encoder = encoder_current;
	
	// First call: return 0 and store initial values
	if (first_call)
	{
		encoder_prev = encoder_current;
		time_prev = millisec;
		first_call = 0;
		return 0;
	}
	
	// Calculate time difference in milliseconds
	uint32_t dt_ms = millisec - time_prev;
	
	// Avoid division by zero
	if (dt_ms == 0)
	{
		return 0;
	}
	
	// Calculate encoder difference, handling 16-bit overflow/underflow
	// The counter wraps around, so we need to handle signed arithmetic properly
	int32_t encoder_diff = (int32_t)encoder_current - (int32_t)encoder_prev;
	
	// Handle overflow: detect wraparound in 16-bit signed counter
	// Counter range: -32768 to 32767 (65536 total values)
	// If the difference is larger than half the counter range (32767), assume wraparound
	// This handles the case where the counter wrapped around rather than moved the long way
	if (encoder_diff > 32767)
	{
		// Wrapped forward: e.g., 32767 -> -32768 (should be interpreted as -1)
		encoder_diff -= 65536;
	}
	else if (encoder_diff < -32767)
	{
		// Wrapped backward: e.g., -32768 -> 32767 (should be interpreted as +1)
		encoder_diff += 65536;
	}
	
	// Calculate velocity in RPM
	// Formula: v = (encoder_diff / RESOLUTION) * (60000 / dt_ms)
	// encoder_diff is in counts
	// RESOLUTION is counts per revolution
	// dt_ms is time difference in milliseconds
	// 60000 converts milliseconds to minutes (60 * 1000)
	
	// Use 64-bit intermediate to avoid overflow
	// v = (encoder_diff * 60000) / (RESOLUTION * dt_ms)
	int64_t numerator = (int64_t)encoder_diff * 60000LL;
	int64_t denominator = (int64_t)RESOLUTION * (int64_t)dt_ms;
	
	// Perform division with proper rounding
	int32_t velocity_rpm;
	if (denominator != 0)
	{
		// Round to nearest: add half of denominator before division
		if (numerator >= 0)
		{
			velocity_rpm = (int32_t)((numerator + denominator / 2) / denominator);
		}
		else
		{
			velocity_rpm = (int32_t)((numerator - denominator / 2) / denominator);
		}
	}
	else
	{
		velocity_rpm = 0;
	}
	
	// Update previous values for next call
	encoder_prev = encoder_current;
	time_prev = millisec;
	
	return velocity_rpm;
}

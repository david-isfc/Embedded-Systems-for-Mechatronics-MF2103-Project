#include "peripherals.h"

#define RESOLUTION 0		// TODO: Determine the number of pulses per revolution

int16_t encoder;			// Global variable, can be used for debugging purposes

/* Enable both half-bridges to drive the motor */
void Peripheral_GPIO_EnableMotor(void)
{
	return;
}

/* Disable both half-bridges to stop the motor */
void Peripheral_GPIO_DisableMotor(void)
{
	return;
}

/* Drive the motor in both directions */
void Peripheral_PWM_ActuateMotor(int32_t vel)
{
	return;
}

/* Read the encoder value and calculate the current velocity in RPM */
int32_t Peripheral_Encoder_CalculateVelocity(uint32_t ms)
{
	return 0;
}

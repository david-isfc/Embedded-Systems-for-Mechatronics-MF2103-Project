#include "application.h"
#include "cmsis_os2.h"
#include "controller.h"
#include "main.h"
#include "peripherals.h"
/* Global variables ----------------------------------------------------------*/
int32_t reference, velocity, control;
uint32_t millisec;
/* Thread IDs */
osThreadId_t tid_app_main;
osThreadId_t tid_app_ctrl;
osThreadId_t tid_app_ref;
/* Timer IDs */
osTimerId_t timer_ctrl;
osTimerId_t timer_ref;
/* Thread Definitions */
void app_main(void *argument);
void app_ctrl(void *argument);
void app_ref(void *argument);
/* Timer Callbacks */
static void Timer_Callback(void *argument);
/* Constants */
#define FLAG_periodic 0x01
/* Functions -----------------------------------------------------------------*/
/* Run setup needed for all periodic tasks */
void Application_Setup() {
  // Reset global variables
  reference = 2000;
  velocity = 0;
  control = 0;
  millisec = 0;
  // Initialise hardware
  Peripheral_GPIO_EnableMotor();
  // Initialize controller
  Controller_Reset();
  // Initialize CMSIS-RTOS
  osKernelInitialize();
  // Create the main thread
  tid_app_main = osThreadNew(app_main, NULL, NULL);
  // Start the kernel
  osKernelStart();
}
/* Define what to do in the infinite loop (called by app_main) */
void Application_Loop() {
  // Do nothing
  osThreadFlagsWait(FLAG_periodic, osFlagsWaitAll, osWaitForever);
}
/* app_main Thread */
void app_main(void *argument) {
  /* Create child threads */
  // app_ctrl: High priority (runs often: 10ms)
  const osThreadAttr_t ctrl_attr = {.priority = osPriorityAboveNormal};
  tid_app_ctrl = osThreadNew(app_ctrl, NULL, &ctrl_attr);
  // app_ref: Normal/Low priority (runs rarely: 4000ms)
  const osThreadAttr_t ref_attr = {.priority = osPriorityNormal};
  tid_app_ref = osThreadNew(app_ref, NULL, &ref_attr);
  /* Create and Start Timers */
  // Timer for Control Loop (10ms)
  timer_ctrl =
      osTimerNew(Timer_Callback, osTimerPeriodic, (void *)tid_app_ctrl, NULL);
  osTimerStart(timer_ctrl, PERIOD_CTRL);
  // Timer for Reference Loop (4000ms)
  timer_ref =
      osTimerNew(Timer_Callback, osTimerPeriodic, (void *)tid_app_ref, NULL);
  osTimerStart(timer_ref, PERIOD_REF);
  for (;;) {
    Application_Loop();
  }
}
/* app_ctrl Thread */
void app_ctrl(void *argument) {
  for (;;) {
    // Wait for signal from timer
    osThreadFlagsWait(FLAG_periodic, osFlagsWaitAll, osWaitForever);
    // Get time (from OS)
    millisec = Main_GetTickMillisec();
    // Calculate motor velocity
    velocity = Peripheral_Encoder_CalculateVelocity(millisec);
    // Calculate control signal
    control = Controller_PIController(&reference, &velocity, &millisec);
    // Apply control signal to motor
    Peripheral_PWM_ActuateMotor(control);
  }
}
/* app_ref Thread */
void app_ref(void *argument) {
  for (;;) {
    // Wait for signal from timer
    osThreadFlagsWait(FLAG_periodic, osFlagsWaitAll, osWaitForever);
    // Flip the direction of the reference
    reference = -reference;
  }
}
/* Timer Callback */
static void Timer_Callback(void *argument) {
  osThreadId_t tid = (osThreadId_t)argument;
  osThreadFlagsSet(tid, FLAG_periodic);
}

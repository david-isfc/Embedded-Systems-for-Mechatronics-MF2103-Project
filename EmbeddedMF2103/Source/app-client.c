#include "main.h"
#include "application.h"
#include "controller.h"
#include "peripherals.h"
#include "network_protocol.h"
#include "cmsis_os2.h"

#ifdef _ETHERNET_ENABLED
#include "socket.h"
#include "wizchip_conf.h"
#endif

/* Global variables ----------------------------------------------------------*/
int32_t reference, velocity, control;
uint32_t millisec;

/* Thread IDs */
osThreadId_t tid_app_main;
osThreadId_t tid_app_ctrl;
osThreadId_t tid_app_comm;

/* Timer IDs */
osTimerId_t timer_ctrl;

/* Connection state */
#ifdef _ETHERNET_ENABLED
static uint8_t client_socket = 0;
static volatile uint8_t connected = 0;
#endif

/* Thread Definitions */
void app_main(void *argument);
void app_ctrl(void *argument);
void app_comm(void *argument);

/* Timer Callbacks */
static void Timer_Callback(void *argument);

/* Constants */
#define FLAG_periodic 0x01
#define FLAG_connected 0x02
#define FLAG_control_ready 0x04

/* Functions -----------------------------------------------------------------*/

void Application_Setup() {
  reference = 2000;
  velocity = 0;
  control = 0;
  millisec = 0;

  Peripheral_GPIO_EnableMotor();
  Controller_Reset();

#ifdef _ETHERNET_ENABLED
  client_socket = 0;
  connected = 0;
#endif

  osKernelInitialize();
  const osThreadAttr_t main_attr = {.priority = osPriorityBelowNormal};
  tid_app_main = osThreadNew(app_main, NULL, &main_attr);
  osKernelStart();
}

void Application_Loop() {
#ifdef _ETHERNET_ENABLED
  for (;;) {
    if (!connected) {
      // Try to connect
      uint8_t server_ip[4] = {192, 168, 0, 10};
      
      // Create socket (port 0 for client, will be assigned automatically)
      int8_t result = socket(0, Sn_MR_TCP, 0, 0);
      if (result >= 0) {
        client_socket = (uint8_t)result;
        // Connect to server
        if (connect(client_socket, server_ip, SERVER_PORT) == SOCK_OK) {
          connected = 1;
          Controller_Reset();
          Peripheral_GPIO_EnableMotor();
          if (timer_ctrl != NULL) {
            osTimerStart(timer_ctrl, PERIOD_CTRL);
          }
          osThreadFlagsSet(tid_app_comm, FLAG_connected);
        } else {
          close(client_socket);
          client_socket = 0;
        }
      }
      osDelay(500);
    } else {
      osDelay(100);
    }
  }
#else
  osThreadFlagsWait(FLAG_periodic, osFlagsWaitAll, osWaitForever);
#endif
}

void app_main(void *argument) {
  const osThreadAttr_t ctrl_attr = {.priority = osPriorityAboveNormal};
  tid_app_ctrl = osThreadNew(app_ctrl, NULL, &ctrl_attr);
  
  const osThreadAttr_t comm_attr = {.priority = osPriorityNormal};
  tid_app_comm = osThreadNew(app_comm, NULL, &comm_attr);
  
  timer_ctrl = osTimerNew(Timer_Callback, osTimerPeriodic, (void *)tid_app_ctrl, NULL);
  
  for (;;) {
    Application_Loop();
  }
}

void app_ctrl(void *argument) {
  for (;;) {
    osThreadFlagsWait(FLAG_periodic, osFlagsWaitAll, osWaitForever);
    
#ifdef _ETHERNET_ENABLED
    if (!connected) {
      continue;
    }
    
    millisec = Main_GetTickMillisec();
    velocity = Peripheral_Encoder_CalculateVelocity(millisec);
    
    // Wait for control signal from server (with timeout for safety)
    uint32_t flags = osThreadFlagsWait(FLAG_control_ready, osFlagsWaitAny, PERIOD_CTRL * 2);
    
    if (!(flags & FLAG_control_ready) || !connected) {
      Peripheral_PWM_ActuateMotor(0);
      Peripheral_GPIO_DisableMotor();
      if (!connected) continue;
      connected = 0;
      continue;
    }
    
    Peripheral_PWM_ActuateMotor(control);
#else
    millisec = Main_GetTickMillisec();
    velocity = Peripheral_Encoder_CalculateVelocity(millisec);
    control = Controller_PIController(&reference, &velocity, &millisec);
    Peripheral_PWM_ActuateMotor(control);
#endif
  }
}

void app_comm(void *argument) {
#ifdef _ETHERNET_ENABLED
  ClientData_t tx_data;
  ServerData_t rx_data;
  int32_t bytes_sent, bytes_received;
  
  for (;;) {
    osThreadFlagsWait(FLAG_connected, osFlagsWaitAny, osWaitForever);
    
    while (connected) {
      // Check socket status
      uint8_t socket_status;
      if (getsockopt(client_socket, SO_STATUS, &socket_status) != SOCK_OK || socket_status != SOCK_ESTABLISHED) {
        connected = 0;
        break;
      }
      
      // Send velocity data
      tx_data.velocity = velocity;
      tx_data.timestamp = millisec;
      
      bytes_sent = send(client_socket, (uint8_t*)&tx_data, (uint16_t)sizeof(ClientData_t));
      if (bytes_sent != (int32_t)sizeof(ClientData_t)) {
        connected = 0;
        break;
      }
      
      // Receive control signal
      bytes_received = recv(client_socket, (uint8_t*)&rx_data, (uint16_t)sizeof(ServerData_t));
      if (bytes_received != (int32_t)sizeof(ServerData_t)) {
        connected = 0;
        break;
      }
      
      control = rx_data.control;
      osThreadFlagsSet(tid_app_ctrl, FLAG_control_ready);
    }
    
    // Cleanup on disconnect
    if (client_socket != 0) {
      close(client_socket);
      client_socket = 0;
    }
    Peripheral_PWM_ActuateMotor(0);
    Peripheral_GPIO_DisableMotor();
    if (timer_ctrl != NULL) {
      osTimerStop(timer_ctrl);
    }
    Controller_Reset();
  }
#else
  for (;;) {
    osDelay(1000);
  }
#endif
}

static void Timer_Callback(void *argument) {
  osThreadId_t tid = (osThreadId_t)argument;
  osThreadFlagsSet(tid, FLAG_periodic);
}

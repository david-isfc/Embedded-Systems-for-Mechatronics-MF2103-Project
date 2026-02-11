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
osThreadId_t tid_app_ref;
osThreadId_t tid_app_comm;

/* Timer IDs */
osTimerId_t timer_ctrl;
osTimerId_t timer_ref;

/* Connection state */
#ifdef _ETHERNET_ENABLED
static uint8_t server_socket = 0;
static volatile uint8_t connected = 0;
#endif

/* Thread Definitions */
void app_main(void *argument);
void app_ctrl(void *argument);
void app_ref(void *argument);
void app_comm(void *argument);

/* Timer Callbacks */
static void Timer_Callback(void *argument);

/* Constants */
#define FLAG_periodic 0x01
#define FLAG_connected 0x02

/* Functions -----------------------------------------------------------------*/

void Application_Setup() {
  reference = 2000;
  velocity = 0;
  control = 0;
  millisec = 0;

  Controller_Reset();

#ifdef _ETHERNET_ENABLED
  server_socket = 0;
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
      // Try to accept connection
      // Create socket with server port (socket() binds the port automatically)
      int8_t result = socket(0, Sn_MR_TCP, SERVER_PORT, 0);
      if (result >= 0) {
        server_socket = (uint8_t)result;
        // Listen for connections (also accepts automatically in WIZnet)
        if (listen(server_socket) == SOCK_OK) {
          // Wait for connection (check status)
          uint8_t socket_status;
          uint32_t timeout = 0;
          while (timeout < 5000) { // 5 second timeout
            if (getsockopt(server_socket, SO_STATUS, &socket_status) == SOCK_OK) {
              if (socket_status == SOCK_ESTABLISHED) {
                connected = 1;
                Controller_Reset();
                if (timer_ctrl != NULL) {
                  osTimerStart(timer_ctrl, PERIOD_CTRL);
                }
                if (timer_ref != NULL) {
                  osTimerStart(timer_ref, PERIOD_REF);
                }
                osThreadFlagsSet(tid_app_comm, FLAG_connected);
                break;
              }
            }
            osDelay(10);
            timeout += 10;
          }
          if (!connected) {
            close(server_socket);
            server_socket = 0;
          }
        } else {
          close(server_socket);
          server_socket = 0;
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
  
  const osThreadAttr_t ref_attr = {.priority = osPriorityNormal};
  tid_app_ref = osThreadNew(app_ref, NULL, &ref_attr);
  
  const osThreadAttr_t comm_attr = {.priority = osPriorityNormal};
  tid_app_comm = osThreadNew(app_comm, NULL, &comm_attr);
  
  timer_ctrl = osTimerNew(Timer_Callback, osTimerPeriodic, (void *)tid_app_ctrl, NULL);
  timer_ref = osTimerNew(Timer_Callback, osTimerPeriodic, (void *)tid_app_ref, NULL);
  
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
    
    // Calculate control signal (velocity updated by comm thread)
    control = Controller_PIController(&reference, &velocity, &millisec);
    osThreadFlagsSet(tid_app_comm, FLAG_periodic); // Signal control ready
#else
    millisec = Main_GetTickMillisec();
    velocity = Peripheral_Encoder_CalculateVelocity(millisec);
    control = Controller_PIController(&reference, &velocity, &millisec);
    Peripheral_PWM_ActuateMotor(control);
#endif
  }
}

void app_ref(void *argument) {
  for (;;) {
    osThreadFlagsWait(FLAG_periodic, osFlagsWaitAll, osWaitForever);
    
#ifdef _ETHERNET_ENABLED
    if (!connected) {
      continue;
    }
#endif
    
    reference = -reference;
  }
}

void app_comm(void *argument) {
#ifdef _ETHERNET_ENABLED
  ClientData_t rx_data;
  ServerData_t tx_data;
  int32_t bytes_received, bytes_sent;
  
  for (;;) {
    osThreadFlagsWait(FLAG_connected, osFlagsWaitAny, osWaitForever);
    
    while (connected) {
      // Check socket status
      uint8_t socket_status;
      if (getsockopt(server_socket, SO_STATUS, &socket_status) != SOCK_OK || socket_status != SOCK_ESTABLISHED) {
        connected = 0;
        break;
      }
      
      // Receive velocity data
      bytes_received = recv(server_socket, (uint8_t*)&rx_data, (uint16_t)sizeof(ClientData_t));
      if (bytes_received != (int32_t)sizeof(ClientData_t)) {
        connected = 0;
        break;
      }
      
      velocity = rx_data.velocity;
      millisec = rx_data.timestamp;
      
      // Wait for control calculation
      osThreadFlagsWait(FLAG_periodic, osFlagsWaitAny, PERIOD_CTRL);
      
      if (!connected) {
        break;
      }
      
      // Send control signal
      tx_data.control = control;
      bytes_sent = send(server_socket, (uint8_t*)&tx_data, (uint16_t)sizeof(ServerData_t));
      if (bytes_sent != (int32_t)sizeof(ServerData_t)) {
        connected = 0;
        break;
      }
    }
    
    // Cleanup on disconnect
    if (server_socket != 0) {
      close(server_socket);
      server_socket = 0;
    }
    if (timer_ctrl != NULL) {
      osTimerStop(timer_ctrl);
    }
    if (timer_ref != NULL) {
      osTimerStop(timer_ref);
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

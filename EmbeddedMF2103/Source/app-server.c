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
static int8_t server_socket = -1;
static int8_t client_socket = -1;
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
  server_socket = -1;
  client_socket = -1;
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
      server_socket = socket(AF_INET, SOCK_STREAM, 0);
      if (server_socket >= 0) {
        sockaddr_in server_addr;
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(SERVER_PORT);
        server_addr.sin_addr.s_addr = 0;
        
        if (bind(server_socket, (sockaddr*)&server_addr, sizeof(server_addr)) == 0) {
          if (listen(server_socket, 1) == 0) {
            sockaddr_in client_addr;
            uint8_t client_addr_len = sizeof(client_addr);
            client_socket = accept(server_socket, (sockaddr*)&client_addr, &client_addr_len);
            
            if (client_socket >= 0) {
              connected = 1;
              Controller_Reset();
              if (timer_ctrl != NULL) {
                osTimerStart(timer_ctrl, PERIOD_CTRL);
              }
              if (timer_ref != NULL) {
                osTimerStart(timer_ref, PERIOD_REF);
              }
              osThreadFlagsSet(tid_app_comm, FLAG_connected);
            } else {
              close(server_socket);
              server_socket = -1;
              osDelay(500);
            }
          } else {
            close(server_socket);
            server_socket = -1;
            osDelay(500);
          }
        } else {
          close(server_socket);
          server_socket = -1;
          osDelay(500);
        }
      } else {
        osDelay(500);
      }
    } else {
      osDelay(100); // Check connection status periodically
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
  
  for (;;) {
    osThreadFlagsWait(FLAG_connected, osFlagsWaitAny, osWaitForever);
    
    while (connected) {
      // Receive velocity data
      if (recv(client_socket, (uint8_t*)&rx_data, sizeof(ClientData_t), 0) != sizeof(ClientData_t)) {
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
      if (send(client_socket, (uint8_t*)&tx_data, sizeof(ServerData_t), 0) != sizeof(ServerData_t)) {
        connected = 0;
        break;
      }
    }
    
    // Cleanup on disconnect
    if (client_socket >= 0) {
      close(client_socket);
      client_socket = -1;
    }
    if (server_socket >= 0) {
      close(server_socket);
      server_socket = -1;
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

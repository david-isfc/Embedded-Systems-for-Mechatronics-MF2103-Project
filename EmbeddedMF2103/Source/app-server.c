/**
 * @file app-server.c
 * @brief Server application for distributed control system
 * 
 * This file implements the server-side of the distributed control system.
 * The server generates reference signals, receives velocity data from client,
 * calculates control signals using PI controller, and sends control to client.
 * 
 * Note: Socket API functions (socket, bind, listen, accept, send, recv, etc.)
 * are based on Berkeley socket API. You may need to adjust function signatures
 * or includes based on the actual WIZnet ioLibrary implementation.
 */

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
static volatile uint8_t connection_established = 0;
static volatile uint8_t connection_lost = 0;
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
#define FLAG_disconnected 0x04
#define FLAG_new_data 0x08
#define FLAG_control_ready 0x10

/* Functions -----------------------------------------------------------------*/

/* Run setup needed for all periodic tasks */
void Application_Setup() {
  // Reset global variables
  reference = 2000;
  velocity = 0;
  control = 0;
  millisec = 0;

  // Initialize controller
  Controller_Reset();

#ifdef _ETHERNET_ENABLED
  // Reset connection state
  server_socket = -1;
  client_socket = -1;
  connection_established = 0;
  connection_lost = 0;
#endif

  // Initialize CMSIS-RTOS
  osKernelInitialize();

  // Create the main thread
  const osThreadAttr_t main_attr = {.priority = osPriorityBelowNormal};
  tid_app_main = osThreadNew(app_main, NULL, &main_attr);

  // Start the kernel
  osKernelStart();
}

/* Define what to do in the infinite loop (called by app_main) */
void Application_Loop() {
#ifdef _ETHERNET_ENABLED
  // Connection management loop
  for (;;) {
    // Check if connection is lost
    if (connection_lost) {
      // Stop timers
      if (timer_ctrl != NULL) {
        osTimerStop(timer_ctrl);
      }
      if (timer_ref != NULL) {
        osTimerStop(timer_ref);
      }
      
      // Close sockets if open
      if (client_socket >= 0) {
        close(client_socket);
        client_socket = -1;
      }
      if (server_socket >= 0) {
        close(server_socket);
        server_socket = -1;
      }
      
      connection_established = 0;
      connection_lost = 0;
      
      // Reset controller
      Controller_Reset();
      
      // Wait before attempting reconnection
      osDelay(1000);
    }
    
    // Try to establish connection
    if (!connection_established) {
      // Create server socket
      server_socket = socket(AF_INET, SOCK_STREAM, 0);
      
      if (server_socket >= 0) {
        // Bind to port
        sockaddr_in server_addr;
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(SERVER_PORT);
        server_addr.sin_addr.s_addr = 0; // INADDR_ANY
        
        if (bind(server_socket, (sockaddr*)&server_addr, sizeof(server_addr)) == 0) {
          // Listen for connections
          if (listen(server_socket, 1) == 0) {
            // Accept connection (blocking)
            sockaddr_in client_addr;
            uint8_t client_addr_len = sizeof(client_addr);
            client_socket = accept(server_socket, (sockaddr*)&client_addr, &client_addr_len);
            
            if (client_socket >= 0) {
              // Connection accepted
              connection_established = 1;
              connection_lost = 0;
              
              // Reset controller for new session (t=0)
              Controller_Reset();
              
              // Start timers
              if (timer_ctrl != NULL) {
                osTimerStart(timer_ctrl, PERIOD_CTRL);
              }
              if (timer_ref != NULL) {
                osTimerStart(timer_ref, PERIOD_REF);
              }
              
              // Signal communication thread
              osThreadFlagsSet(tid_app_comm, FLAG_connected);
            } else {
              // Accept failed
              close(server_socket);
              server_socket = -1;
              osDelay(500);
            }
          } else {
            // Listen failed
            close(server_socket);
            server_socket = -1;
            osDelay(500);
          }
        } else {
          // Bind failed
          close(server_socket);
          server_socket = -1;
          osDelay(500);
        }
      } else {
        osDelay(500); // Wait before retry
      }
    } else {
      // Connection established, wait for disconnect signal
      osThreadFlagsWait(FLAG_disconnected, osFlagsWaitAny, osWaitForever);
    }
  }
#else
  // Non-ethernet version (should not be reached if configured correctly)
  osThreadFlagsWait(FLAG_periodic, osFlagsWaitAll, osWaitForever);
#endif
}

/* app_main Thread */
void app_main(void *argument) {
  /* Create child threads */
  // app_ctrl: High priority (runs often: 50ms)
  const osThreadAttr_t ctrl_attr = {.priority = osPriorityAboveNormal};
  tid_app_ctrl = osThreadNew(app_ctrl, NULL, &ctrl_attr);
  
  // app_ref: Normal priority (runs rarely: 4000ms)
  const osThreadAttr_t ref_attr = {.priority = osPriorityNormal};
  tid_app_ref = osThreadNew(app_ref, NULL, &ref_attr);
  
  // app_comm: Normal priority (handles communication)
  const osThreadAttr_t comm_attr = {.priority = osPriorityNormal};
  tid_app_comm = osThreadNew(app_comm, NULL, &comm_attr);
  
  /* Create and Start Timers */
  // Timer for Control Loop (50ms)
  timer_ctrl = osTimerNew(Timer_Callback, osTimerPeriodic, (void *)tid_app_ctrl, NULL);
  
  // Timer for Reference Loop (4000ms)
  timer_ref = osTimerNew(Timer_Callback, osTimerPeriodic, (void *)tid_app_ref, NULL);
  
  for (;;) {
    Application_Loop();
  }
}

/* app_ctrl Thread */
void app_ctrl(void *argument) {
  for (;;) {
#ifdef _ETHERNET_ENABLED
    // Wait for new velocity data from communication thread
    // The communication thread will set FLAG_new_data when data is received
    uint32_t flags = osThreadFlagsWait(FLAG_new_data, osFlagsWaitAny, osWaitForever);
    
    if (!(flags & FLAG_new_data)) {
      continue;
    }
    
    // Only execute if connected
    if (!connection_established || connection_lost) {
      continue;
    }
    
    // Calculate control signal (velocity and timestamp already updated by communication thread)
    control = Controller_PIController(&reference, &velocity, &millisec);
    
    // Signal communication thread that control signal is ready
    osThreadFlagsSet(tid_app_comm, FLAG_control_ready);
#else
    // Non-ethernet version - use timer
    osThreadFlagsWait(FLAG_periodic, osFlagsWaitAll, osWaitForever);
    millisec = Main_GetTickMillisec();
    velocity = Peripheral_Encoder_CalculateVelocity(millisec);
    control = Controller_PIController(&reference, &velocity, &millisec);
    Peripheral_PWM_ActuateMotor(control);
#endif
  }
}

/* app_ref Thread */
void app_ref(void *argument) {
  for (;;) {
    // Wait for signal from timer
    osThreadFlagsWait(FLAG_periodic, osFlagsWaitAll, osWaitForever);
    
#ifdef _ETHERNET_ENABLED
    // Only execute if connected
    if (!connection_established || connection_lost) {
      continue;
    }
#endif
    
    // Flip the direction of the reference
    reference = -reference;
  }
}

/* app_comm Thread */
void app_comm(void *argument) {
#ifdef _ETHERNET_ENABLED
  ClientData_t rx_data;
  ServerData_t tx_data;
  int32_t bytes_received, bytes_sent;
  
  for (;;) {
    // Wait for connection to be established
    osThreadFlagsWait(FLAG_connected, osFlagsWaitAny, osWaitForever);
    
    // Communication loop
    while (connection_established && !connection_lost) {
      // Check socket status periodically
      int8_t socket_status = getsockopt(client_socket, SO_STATUS, NULL);
      if (socket_status != SOCK_ESTABLISHED) {
        connection_lost = 1;
        osThreadFlagsSet(tid_app_main, FLAG_disconnected);
        break;
      }
      
      // Receive velocity and timestamp from client (blocking)
      bytes_received = recv(client_socket, (uint8_t*)&rx_data, sizeof(ClientData_t), 0);
      
      if (bytes_received != sizeof(ClientData_t)) {
        // Receive failed, connection lost
        connection_lost = 1;
        osThreadFlagsSet(tid_app_main, FLAG_disconnected);
        break;
      }
      
      // Update global variables (read by control thread)
      velocity = rx_data.velocity;
      millisec = rx_data.timestamp;
      
      // Signal control thread that new data is available
      osThreadFlagsSet(tid_app_ctrl, FLAG_new_data);
      
      // Wait for control thread to calculate control signal
      uint32_t flags = osThreadFlagsWait(FLAG_control_ready, osFlagsWaitAny, PERIOD_CTRL);
      
      if (!(flags & FLAG_control_ready)) {
        // Timeout - control calculation took too long, connection may be lost
        connection_lost = 1;
        osThreadFlagsSet(tid_app_main, FLAG_disconnected);
        break;
      }
      
      // Send control signal to client
      tx_data.control = control;
      bytes_sent = send(client_socket, (uint8_t*)&tx_data, sizeof(ServerData_t), 0);
      
      if (bytes_sent != sizeof(ServerData_t)) {
        // Send failed, connection lost
        connection_lost = 1;
        osThreadFlagsSet(tid_app_main, FLAG_disconnected);
        break;
      }
    }
  }
#else
  for (;;) {
    osDelay(1000);
  }
#endif
}

/* Timer Callback */
static void Timer_Callback(void *argument) {
  osThreadId_t tid = (osThreadId_t)argument;
  osThreadFlagsSet(tid, FLAG_periodic);
}

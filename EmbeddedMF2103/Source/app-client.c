/**
 * @file app-client.c
 * @brief Client application for distributed control system
 * 
 * This file implements the client-side of the distributed control system.
 * The client reads the encoder, calculates velocity, sends data to server,
 * receives control signals, and actuates the motor.
 * 
 * Note: Socket API functions (socket, connect, send, recv, etc.) are based on
 * Berkeley socket API. You may need to adjust function signatures or includes
 * based on the actual WIZnet ioLibrary implementation.
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
osThreadId_t tid_app_comm;

/* Timer IDs */
osTimerId_t timer_ctrl;

/* Connection state */
#ifdef _ETHERNET_ENABLED
static int8_t client_socket = -1;
static volatile uint8_t connection_established = 0;
static volatile uint8_t connection_lost = 0;
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
#define FLAG_disconnected 0x04
#define FLAG_data_ready 0x08      // Velocity data ready to send
#define FLAG_control_received 0x10 // Control signal received

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

#ifdef _ETHERNET_ENABLED
  // Reset connection state
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
      // Stop motor immediately
      Peripheral_PWM_ActuateMotor(0);
      Peripheral_GPIO_DisableMotor();
      
      // Stop timers
      if (timer_ctrl != NULL) {
        osTimerStop(timer_ctrl);
      }
      
      // Close socket if open
      if (client_socket >= 0) {
        close(client_socket);
        client_socket = -1;
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
      // Create socket
      client_socket = socket(AF_INET, SOCK_STREAM, 0);
      
      if (client_socket >= 0) {
        // Server address: 192.168.0.10
        uint8_t server_ip[4] = {192, 168, 0, 10};
        
        // Connect to server
        sockaddr_in server_addr;
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(SERVER_PORT);
        server_addr.sin_addr.s_addr = (server_ip[0] << 24) | (server_ip[1] << 16) | 
                                       (server_ip[2] << 8) | server_ip[3];
        
        if (connect(client_socket, (sockaddr*)&server_addr, sizeof(server_addr)) == 0) {
          // Connection successful
          connection_established = 1;
          connection_lost = 0;
          
          // Reset controller for new session
          Controller_Reset();
          
          // Enable motor
          Peripheral_GPIO_EnableMotor();
          
          // Start control timer
          if (timer_ctrl != NULL) {
            osTimerStart(timer_ctrl, PERIOD_CTRL);
          }
          
          // Signal communication thread
          osThreadFlagsSet(tid_app_comm, FLAG_connected);
        } else {
          // Connection failed, close socket
          close(client_socket);
          client_socket = -1;
          osDelay(500); // Wait before retry
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
  
  // app_comm: Normal priority (handles communication)
  const osThreadAttr_t comm_attr = {.priority = osPriorityNormal};
  tid_app_comm = osThreadNew(app_comm, NULL, &comm_attr);
  
  /* Create Timer */
  // Timer for Control Loop (50ms)
  timer_ctrl = osTimerNew(Timer_Callback, osTimerPeriodic, (void *)tid_app_ctrl, NULL);
  
  for (;;) {
    Application_Loop();
  }
}

/* app_ctrl Thread */
void app_ctrl(void *argument) {
  for (;;) {
    // Wait for signal from timer
    osThreadFlagsWait(FLAG_periodic, osFlagsWaitAll, osWaitForever);
    
#ifdef _ETHERNET_ENABLED
    // Only execute if connected
    if (!connection_established || connection_lost) {
      continue;
    }
    
    // Get time (from OS)
    millisec = Main_GetTickMillisec();
    
    // Calculate motor velocity
    velocity = Peripheral_Encoder_CalculateVelocity(millisec);
    
    // Signal communication thread that new data is ready
    osThreadFlagsSet(tid_app_comm, FLAG_data_ready);
    
    // Wait for control signal from server (via communication thread)
    // Use timeout to detect connection loss
    uint32_t flags = osThreadFlagsWait(FLAG_control_received, osFlagsWaitAny, PERIOD_CTRL * 2);
    
    if (!(flags & FLAG_control_received) || connection_lost) {
      // Timeout or connection lost - stop motor immediately
      Peripheral_PWM_ActuateMotor(0);
      connection_lost = 1;
      osThreadFlagsSet(tid_app_main, FLAG_disconnected);
      continue;
    }
    
    // Apply control signal to motor
    Peripheral_PWM_ActuateMotor(control);
#else
    // Non-ethernet version
    millisec = Main_GetTickMillisec();
    velocity = Peripheral_Encoder_CalculateVelocity(millisec);
    control = Controller_PIController(&reference, &velocity, &millisec);
    Peripheral_PWM_ActuateMotor(control);
#endif
  }
}

/* app_comm Thread */
void app_comm(void *argument) {
#ifdef _ETHERNET_ENABLED
  ClientData_t tx_data;
  ServerData_t rx_data;
  int32_t bytes_sent, bytes_received;
  
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
      
      // Wait for control thread to signal that new velocity data is ready
      uint32_t flags = osThreadFlagsWait(FLAG_data_ready, osFlagsWaitAny, PERIOD_CTRL * 2);
      
      if (!(flags & FLAG_data_ready)) {
        // Timeout - no data ready, connection may be lost
        connection_lost = 1;
        osThreadFlagsSet(tid_app_main, FLAG_disconnected);
        break;
      }
      
      // Prepare data to send
      tx_data.velocity = velocity;
      tx_data.timestamp = millisec;
      
      // Send velocity and timestamp to server
      bytes_sent = send(client_socket, (uint8_t*)&tx_data, sizeof(ClientData_t), 0);
      
      if (bytes_sent != sizeof(ClientData_t)) {
        // Send failed, connection lost
        connection_lost = 1;
        osThreadFlagsSet(tid_app_main, FLAG_disconnected);
        break;
      }
      
      // Receive control signal from server (blocking)
      bytes_received = recv(client_socket, (uint8_t*)&rx_data, sizeof(ServerData_t), 0);
      
      if (bytes_received != sizeof(ServerData_t)) {
        // Receive failed, connection lost
        connection_lost = 1;
        osThreadFlagsSet(tid_app_main, FLAG_disconnected);
        break;
      }
      
      // Update control signal
      control = rx_data.control;
      
      // Signal control thread that control signal is received
      osThreadFlagsSet(tid_app_ctrl, FLAG_control_received);
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

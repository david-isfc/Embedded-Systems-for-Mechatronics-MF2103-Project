#include "main.h"
#include "application.h"
#include "controller.h"
#include "network_protocol.h"
#include "cmsis_os2.h"

#ifdef _ETHERNET_ENABLED
#include "socket.h"
#include "wizchip_conf.h"
#endif

/* Thread and Timer Flags */
#define FLAG_TICK        0x01
#define FLAG_CONN_UP     0x02

/* Thread IDs */
osThreadId_t tid_app_main;
osThreadId_t tid_app_ref;
osThreadId_t tid_app_comm;

/* Timer IDs */
osTimerId_t timer_ref;

/* Global State */
static volatile uint8_t connected = 0;
int32_t reference = 2000; // Starting reference value

/* --- Function Prototypes --- */
void app_main(void *argument);
void app_ref(void *argument);
void app_comm(void *argument);
static void Timer_Callback(void *argument);

/**
 * @brief Setup RTOS kernel and create the Manager thread.
 */
void Application_Setup() {
    osKernelInitialize();
    
    const osThreadAttr_t main_attr = { .priority = osPriorityBelowNormal, .name = "Manager" };
    tid_app_main = osThreadNew(app_main, NULL, &main_attr);
    
    osKernelStart();
}

/**
 * @brief Main Thread: Handles TCP Listening and Thread synchronization.
 */
void app_main(void *argument) {
    // 1. Create sub-threads first
    tid_app_ref = osThreadNew(app_ref, NULL, NULL);
    tid_app_comm = osThreadNew(app_comm, NULL, NULL);

    // 2. Allow kernel to register Thread IDs before creating timer
    osDelay(100); 
    
    timer_ref = osTimerNew(Timer_Callback, osTimerPeriodic, NULL, NULL);

    uint8_t sn = 0; // WIZnet Socket 0

    for (;;) {
        if (!connected) {
            // Open socket in TCP Server mode
            if (socket(sn, Sn_MR_TCP, SERVER_PORT, 0) == sn) {
                if (listen(sn) == SOCK_OK) {
                    uint8_t status;
                    while (connected == 0) {
                        getsockopt(sn, SO_STATUS, &status);
                        
                        if (status == SOCK_ESTABLISHED) {
                            connected = 1;
                            Controller_Reset();
                            
                            // Start reference toggle timer (e.g. 2000ms)
                            osTimerStart(timer_ref, PERIOD_REF); 
                            
                            // Signal the Comm thread to begin processing
                            osThreadFlagsSet(tid_app_comm, FLAG_CONN_UP);
                        } else if (status == SOCK_CLOSED) {
                            break; 
                        }
                        osDelay(100);
                    }
                }
            }
        }
        osDelay(1000); 
    }
}

/**
 * @brief Communication & Control Thread.
 * Processes incoming sensor data and returns PI control signals.
 */
void app_comm(void *argument) {
    ClientData_t rx_pkt;
    ServerData_t tx_pkt;
    uint8_t sn = 0;

    for (;;) {
        // Block until a client connects
        osThreadFlagsWait(FLAG_CONN_UP, osFlagsWaitAny, osWaitForever);
        
        while (connected) {
            // Blocking receive: wait for packet from Client
            int32_t ret = recv(sn, (uint8_t*)&rx_pkt, sizeof(rx_pkt));
            
            if (ret <= 0) {
                connected = 0;
                break;
            }

            // Calculate PI signal based on the current 'reference' global
            tx_pkt.control = Controller_PIController(&reference, &rx_pkt.velocity, &rx_pkt.timestamp);
            
            // Send control value back to client
            if (send(sn, (uint8_t*)&tx_pkt, sizeof(tx_pkt)) != sizeof(tx_pkt)) {
                connected = 0;
                break;
            }

            /* CRITICAL: Yield the CPU to allow app_ref to run! 
               Without this, app_comm may starve other threads of the same priority. */
            osThreadYield();
        }
        
        // Clean up on disconnect
        osTimerStop(timer_ref);
        close(sn);
        osThreadFlagsClear(FLAG_CONN_UP);
    }
}

/**
 * @brief Reference Thread: Toggles the reference value for a square wave.
 */
void app_ref(void *argument) {
    for (;;) {
        // Wait for the periodic signal from the Timer
        osThreadFlagsWait(FLAG_TICK, osFlagsWaitAny, osWaitForever);
        
        if (connected) {
            reference = -reference; // Square wave flip
            
            // HEARTBEAT: Toggle Green LED (PA5) to confirm thread is waking up
            HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5); 
        }
    }
}

/**
 * @brief Timer Callback: Signals the app_ref thread.
 */
static void Timer_Callback(void *argument) {
    if (tid_app_ref != NULL) {
        osThreadFlagsSet(tid_app_ref, FLAG_TICK);
    }
}

/**
 * @brief Yields the main loop to RTOS threads.
 */
void Application_Loop() {
    osThreadYield();
}

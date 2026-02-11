#include "main.h"
#include "application.h"
#include "controller.h"
#include "network_protocol.h"
#include "cmsis_os2.h"

#ifdef _ETHERNET_ENABLED
#include "socket.h"
#include "wizchip_conf.h"
#endif

// Removed #define PERIOD_REF 2000 to avoid macro redefinition warning

/* Thread and Timer Flags */
#define FLAG_TICK       0x01
#define FLAG_CONN_UP    0x02

/* Thread IDs */
osThreadId_t tid_app_main;
osThreadId_t tid_app_ref;
osThreadId_t tid_app_comm;

/* Timer IDs */
osTimerId_t timer_ref;

/* Global State */
static volatile uint8_t connected = 0;
int32_t reference = 2000;

/* --- Function Prototypes (Fixes 'undeclared identifier' errors) --- */
void app_main(void *argument);
void app_ref(void *argument);
void app_comm(void *argument);
static void Timer_Callback(void *argument);

/* --- Application Setup --- */
void Application_Setup() {
    osKernelInitialize();
    
    // Create the manager thread
    const osThreadAttr_t main_attr = { .priority = osPriorityBelowNormal, .name = "Manager" };
    tid_app_main = osThreadNew(app_main, NULL, &main_attr);
    
    osKernelStart();
}

/* --- Main Thread: TCP Listener & Handshaking --- */
void app_main(void *argument) {
    // Create sub-threads
    tid_app_ref = osThreadNew(app_ref, NULL, NULL);
    tid_app_comm = osThreadNew(app_comm, NULL, NULL);
    
    // Create timer for the reference signal
    // PERIOD_REF is used from application.h
    timer_ref = osTimerNew(Timer_Callback, osTimerPeriodic, (void*)tid_app_ref, NULL);

    uint8_t sn = 0; // Use WIZnet Socket 0

    for (;;) {
        if (!connected) {
            // Open socket with SERVER_PORT from network_protocol.h
            if (socket(sn, Sn_MR_TCP, SERVER_PORT, 0) == sn) {
                if (listen(sn) == SOCK_OK) {
                    uint8_t status;
                    while (connected == 0) {
                        getsockopt(sn, SO_STATUS, &status);
                        if (status == SOCK_ESTABLISHED) {
                            connected = 1;
                            Controller_Reset();
                            osTimerStart(timer_ref, PERIOD_REF); // Uses definition from application.h
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

/* --- Communication & Control Thread --- */
void app_comm(void *argument) {
    ClientData_t rx_pkt;
    ServerData_t tx_pkt;
    uint8_t sn = 0;

    for (;;) {
        osThreadFlagsWait(FLAG_CONN_UP, osFlagsWaitAny, osWaitForever);
        
        while (connected) {
            // Wait for sensor data from Client
            int32_t ret = recv(sn, (uint8_t*)&rx_pkt, sizeof(rx_pkt));
            
            if (ret <= 0) {
                connected = 0;
                break;
            }

            // Calculate PI signal immediately upon packet arrival
            tx_pkt.control = Controller_PIController(&reference, &rx_pkt.velocity, &rx_pkt.timestamp);
            
            // Send back to client
            if (send(sn, (uint8_t*)&tx_pkt, sizeof(tx_pkt)) != sizeof(tx_pkt)) {
                connected = 0;
                break;
            }
        }
        osTimerStop(timer_ref);
        close(sn);
    }
}

/* --- Reference Thread --- */
void app_ref(void *argument) {
    for (;;) {
        osThreadFlagsWait(FLAG_TICK, osFlagsWaitAny, osWaitForever);
        if (connected) {
            reference = -reference; // Square wave toggle
        }
    }
}

/* --- Helper Callbacks --- */
static void Timer_Callback(void *argument) {
    osThreadId_t tid = (osThreadId_t)argument;
    osThreadFlagsSet(tid, FLAG_TICK);
}

void Application_Loop() {
    osThreadYield();
}

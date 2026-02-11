#include "main.h"
#include "application.h"
#include "network_protocol.h"
#include "peripherals.h" 
#include "cmsis_os2.h"

#ifdef _ETHERNET_ENABLED
#include "socket.h"
#include "wizchip_conf.h"
#endif

/* Flags */
#define FLAG_TICK      0x01
#define FLAG_CONN_UP   0x02
#define FLAG_DATA_RX   0x04

osThreadId_t tid_app_main, tid_app_ctrl, tid_app_comm;
osTimerId_t timer_ctrl;

static volatile uint8_t connected = 0;
static int32_t global_velocity = 0;
static int32_t global_control = 0;
static uint32_t global_timestamp = 0;

/* Prototypes */
void app_main(void *argument);
void app_ctrl(void *argument);
void app_comm(void *argument);
static void Timer_Callback(void *argument);

void Application_Setup() {
    osKernelInitialize();
    const osThreadAttr_t main_attr = {.priority = osPriorityNormal, .name = "Manager"};
    tid_app_main = osThreadNew(app_main, NULL, &main_attr);
    osKernelStart();
}

void app_main(void *argument) {
    tid_app_ctrl = osThreadNew(app_ctrl, NULL, NULL);
    tid_app_comm = osThreadNew(app_comm, NULL, NULL);
    timer_ctrl = osTimerNew(Timer_Callback, osTimerPeriodic, NULL, NULL);

    // START TIMER IMMEDIATELY for testing
    osTimerStart(timer_ctrl, 10); 

    uint8_t server_ip[4] = {192, 168, 0, 10};
    uint8_t sn = 0;

    for (;;) {
        if (!connected) {
            if (socket(sn, Sn_MR_TCP, 0, 0) == sn) {
                if (connect(sn, server_ip, SERVER_PORT) == SOCK_OK) {
                    connected = 1;
                    osThreadFlagsSet(tid_app_comm, FLAG_CONN_UP);
                } else {
                    close(sn); //
                }
            }
        }
        osDelay(1000); 
    }
}

void app_ctrl(void *argument) {
    for (;;) {
        // If stuck here, Timer_Callback is not firing
        osThreadFlagsWait(FLAG_TICK, osFlagsWaitAny, osWaitForever);
        
        global_timestamp = Main_GetTickMillisec();
        global_velocity = Peripheral_Encoder_CalculateVelocity(global_timestamp);
        
        if (connected) {
            osThreadFlagsSet(tid_app_comm, FLAG_TICK); 
            
            // Wait for Comm thread to finish network trip
            uint32_t flags = osThreadFlagsWait(FLAG_DATA_RX, osFlagsWaitAny, 50);
            
            if (flags & FLAG_DATA_RX) {
                Peripheral_PWM_ActuateMotor(global_control);
            } else {
                Peripheral_PWM_ActuateMotor(0); // Safety timeout
            }
        } else {
            // Run motor at 0 if not connected, but keep the thread cycling
            Peripheral_PWM_ActuateMotor(0);
        }
    }
}

void app_comm(void *argument) {
    ClientData_t tx_pkt;
    ServerData_t rx_pkt;
    uint8_t sn = 0;

    for (;;) {
        // If stuck here, connect() has not succeeded
        osThreadFlagsWait(FLAG_CONN_UP, osFlagsWaitAny, osWaitForever);
        
        while (connected) {
            // Wait for Ctrl thread to provide new sample
            osThreadFlagsWait(FLAG_TICK, osFlagsWaitAny, osWaitForever);
            
            tx_pkt.velocity = global_velocity;
            tx_pkt.timestamp = global_timestamp;
            
            if (send(sn, (uint8_t*)&tx_pkt, sizeof(tx_pkt)) != sizeof(tx_pkt)) {
                connected = 0; break;
            }
            
            if (recv(sn, (uint8_t*)&rx_pkt, sizeof(rx_pkt)) != sizeof(rx_pkt)) {
                connected = 0; break;
            }
            
            global_control = rx_pkt.control;
            osThreadFlagsSet(tid_app_ctrl, FLAG_DATA_RX);
        }
        close(sn); //
        osThreadFlagsClear(FLAG_TICK);
    }
}

void Application_Loop(void) {
    // Yield the processor to other threads
    osThreadYield(); 
}

static void Timer_Callback(void *argument) {
    osThreadFlagsSet(tid_app_ctrl, FLAG_TICK);
}

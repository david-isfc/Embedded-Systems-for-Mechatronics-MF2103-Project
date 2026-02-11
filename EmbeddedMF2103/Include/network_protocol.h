#ifndef _NETWORK_PROTOCOL_H_
#define _NETWORK_PROTOCOL_H_
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief Data structure for transmitting velocity and timestamp from client to server
 */
typedef struct {
    int32_t velocity;      //!< Motor velocity in RPM
    uint32_t timestamp;    //!< Timestamp in milliseconds
} ClientData_t;

/**
 * @brief Data structure for transmitting control signal from server to client
 */
typedef struct {
    int32_t control;       //!< Control signal for motor
} ServerData_t;

// Server TCP port
#define SERVER_PORT 5000

#ifdef __cplusplus
}
#endif

#endif   // _NETWORK_PROTOCOL_H_

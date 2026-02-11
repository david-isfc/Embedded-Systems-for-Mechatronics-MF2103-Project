# Questions and Answers for Distributed Control System Assignment

This document answers all questions asked throughout the assignment that might be asked during the presentation.

---

## 1. What is the purpose of `#define _WIZCHIP_ W5500`?

**Answer:**
The `#define _WIZCHIP_ W5500` definition in `wizchip_conf.h` tells the ioLibrary driver which WIZnet chipset is being used. This is necessary because:

- Different WIZnet chipsets (W5100, W5100S, W5200, W5300, W5500) have different register layouts and capabilities
- The driver uses conditional compilation to include chipset-specific code paths
- This allows the same driver library to support multiple hardware variants
- The W5500 has improved features like independent socket buffers and better performance compared to earlier chipsets

**If using W5300 instead:**
You would change the definition to `#define _WIZCHIP_ W5300`. The driver would then use W5300-specific register definitions and initialization code. No other steps in the setup process would change, as the socket API abstraction layer remains the same.

---

## 2. An alternative to creating two targets would be to create two new MDK projects. How would you share common code?

**Answer:**
If using separate projects instead of targets, you would need to:

1. **Create a common library project** containing:
   - `controller.c/h` - PI controller implementation
   - `peripherals.c/h` - Motor and encoder drivers
   - `network_protocol.h` - Shared data structures
   - Any other shared modules

2. **Link the library** to both Client and Server projects:
   - Build the common library as a static library (.lib file)
   - Add the library to the linker settings of both projects
   - Include the header files in both projects' include paths

3. **Handle dependencies:**
   - Both projects would need access to HAL libraries and RTOS
   - Include paths must be configured in both projects
   - Preprocessor definitions (`_CLIENT_CONFIG` vs `_SERVER_CONFIG`) would be set per-project

**Advantages of targets approach (what we used):**
- Single project file to maintain
- Easy to switch between variants (Default, RTOS, Client, Server)
- Shared code is automatically included
- Consistent build settings across targets
- Easier debugging and version control

**Disadvantages of separate projects:**
- More complex build system
- Need to maintain library separately
- Potential for version mismatches between projects
- More files to manage

---

## 3. What does `#ifdef _ETHERNET_ENABLED` mean?

**Answer:**
`#ifdef _ETHERNET_ENABLED` is a preprocessor directive that means "if defined _ETHERNET_ENABLED". It enables conditional compilation:

- **If `_ETHERNET_ENABLED` is defined:** The code between `#ifdef` and `#endif` is included in compilation
- **If `_ETHERNET_ENABLED` is NOT defined:** The code is completely excluded (not even compiled)

**In our implementation:**
- `_ETHERNET_ENABLED` is defined in the preprocessor symbols for both Client and Server targets
- This allows the same `main.c` file to include Ethernet initialization code only when needed
- The Default and RTOS targets do NOT define this, so Ethernet code is excluded
- This prevents compilation errors and keeps code size small for non-Ethernet targets

**Where it comes from:**
Defined in Keil MDK under "Options for Target" → "C/C++" → "Preprocessor Symbols" → "Define"

---

## 4. Why did Wizchip decide to use callback functions in their driver?

**Answer:**
Wizchip uses callback functions (`Ethernet_CS_SEL`, `Ethernet_CS_DESEL`, `Ethernet_RB`, `Ethernet_WB`) for several important reasons:

1. **Hardware abstraction:** The driver doesn't know which SPI peripheral or GPIO pins you're using. Callbacks allow you to provide chipset-specific implementations.

2. **Portability:** The same driver code works across different microcontrollers (STM32, AVR, etc.) without modification. Each platform provides its own callback implementations.

3. **Flexibility:** You can use any SPI peripheral (SPI1, SPI2, SPI3) and any GPIO pin for chip select. The driver doesn't care about the implementation details.

4. **Separation of concerns:** The driver handles TCP/IP stack logic, while your code handles low-level hardware access.

**Why it makes code flow ambiguous:**
- The driver calls your functions at unpredictable times (during `send()`, `recv()`, etc.)
- It's not immediately obvious when callbacks are invoked
- Debugging can be harder because execution jumps between driver code and your callbacks
- However, this is a necessary trade-off for portability and abstraction

---

## 5. What is the effect of conditional compilation? How will the code differ between Client and Server targets?

**Answer:**
Conditional compilation allows the same source files to compile differently based on preprocessor definitions.

**In `main.c` (Ethernet configuration):**
```c
#ifdef _SERVER_CONFIG
  wiz_NetInfo netInfo = {
    .mac = {0x52,0x08,0xDC,0x12,0x34,0x57},
    .ip  = {192, 168, 0, 10},  // Server IP
    ...
#else
  wiz_NetInfo netInfo = {
    .mac = {0x52,0x08,0xDC,0x12,0x34,0x58},
    .ip  = {192, 168, 0, 11},  // Client IP
    ...
#endif
```

**Differences:**
- **Server target:** Gets IP 192.168.0.10, MAC ending in 57
- **Client target:** Gets IP 192.168.0.11, MAC ending in 58
- **Application files:** Only `app-client.c` compiles for Client, only `app-server.c` for Server
- **Functionality:** Client connects to server, Server listens and accepts connections

**Benefits:**
- Single codebase for both applications
- Easy to maintain shared code
- Consistent behavior across targets
- No code duplication

---

## 6. What could you do to avoid defining the same struct twice?

**Answer:**
We avoid defining the same struct twice by using a **shared header file** (`network_protocol.h`):

```c
// network_protocol.h
typedef struct {
    int32_t velocity;
    uint32_t timestamp;
} ClientData_t;

typedef struct {
    int32_t control;
} ServerData_t;
```

**Both `app-client.c` and `app-server.c` include this header:**
```c
#include "network_protocol.h"
```

**Benefits:**
- Single source of truth - struct definition exists in one place
- Type safety - compiler ensures both sides use identical structures
- Easy maintenance - change structure once, affects both applications
- Prevents errors from mismatched definitions

**Alternative approaches (not recommended):**
- Copy-paste struct definition (error-prone, hard to maintain)
- Define in separate headers per application (still duplication)
- Use macros to generate structs (complex, harder to read)

---

## 7. How is the connection established?

**Answer:**

**Server side (in `Application_Loop`):**
1. Create socket: `socket(AF_INET, SOCK_STREAM, 0)`
2. Bind to port 5000: `bind(server_socket, ...)`
3. Listen for connections: `listen(server_socket, 1)`
4. Accept connection (blocking): `accept(server_socket, ...)`
5. On success, start timers and signal communication thread

**Client side (in `Application_Loop`):**
1. Create socket: `socket(AF_INET, SOCK_STREAM, 0)`
2. Set server address: IP 192.168.0.10, port 5000
3. Connect to server: `connect(client_socket, ...)`
4. On success, start timer and signal communication thread

**TCP Handshake (automatic):**
- Client sends SYN
- Server responds with SYN-ACK
- Client sends ACK
- Connection established (handled by WIZnet chipset)

**Connection management:**
- Runs in `Application_Loop` (main thread) with low priority
- Continuously retries if connection fails
- Handles reconnection after connection loss
- Stops timers and resets controller on disconnect

---

## 8. Where are messages stored, and what information is sent over the network?

**Answer:**

**Message storage:**
- **Before transmission:** Messages are stored in local variables (`ClientData_t tx_data`, `ServerData_t rx_data`)
- **During transmission:** WIZnet chipset buffers data in its internal memory
- **After reception:** Data is copied into application variables (`velocity`, `control`, etc.)

**Information sent over network layers:**

**Application Layer:**
- Client → Server: `{velocity (int32_t), timestamp (uint32_t)}` = 8 bytes
- Server → Client: `{control (int32_t)}` = 4 bytes

**TCP Layer:**
- Adds TCP header (~20 bytes): source/dest ports, sequence numbers, ACK numbers, flags
- Provides reliable, ordered delivery

**IP Layer:**
- Adds IP header (~20 bytes): source/dest IP addresses, protocol type, checksum
- Routes packets across network

**Ethernet Layer:**
- Adds Ethernet header (14 bytes): source/dest MAC addresses, EtherType
- Adds CRC (4 bytes) at end
- Transmits over physical cable

**Total per packet:**
- Client data: ~8 + 20 (TCP) + 20 (IP) + 14 (Ethernet) + 4 (CRC) ≈ 66 bytes
- Server data: ~4 + 20 + 20 + 14 + 4 ≈ 62 bytes

**Actual data rate:**
- Control period: 50 ms
- Data per second: ~20 packets/sec × 66 bytes ≈ 1.3 KB/s (very low bandwidth)

---

## 9. Which registers are being read/written by the socket interface?

**Answer:**

**WIZnet W5500 registers (accessed via SPI):**

**Socket Registers (per socket, 8 sockets available):**
- **Sn_MR (Socket Mode Register):** Written during `socket()` to set TCP mode
- **Sn_CR (Socket Command Register):** Written to issue commands (OPEN, CONNECT, LISTEN, ACCEPT, CLOSE, SEND, RECV)
- **Sn_IR (Socket Interrupt Register):** Read to check status (CON, DISCON, RECV, SEND, TIMEOUT)
- **Sn_SR (Socket Status Register):** Read via `getsockopt(SO_STATUS)` to check connection state
- **Sn_PORT (Socket Port Register):** Written during `bind()` to set local port
- **Sn_DIPR (Socket Destination IP Register):** Written during `connect()` to set server IP
- **Sn_DPORT (Socket Destination Port Register):** Written during `connect()` to set server port
- **Sn_TX_WR (Socket TX Write Pointer):** Written during `send()` to indicate data location
- **Sn_RX_RD (Socket RX Read Pointer):** Read/written during `recv()` to manage receive buffer

**Common Registers:**
- **MR (Mode Register):** Chipset configuration
- **GAR (Gateway Address Register):** Written during `Ethernet_Config()`
- **SUBR (Subnet Mask Register):** Written during `Ethernet_Config()`
- **SHAR (Source Hardware Address Register):** MAC address, written during `Ethernet_Config()`
- **SIPR (Source IP Address Register):** IP address, written during `Ethernet_Config()`

**Access method:**
- All registers accessed via SPI through callback functions (`Ethernet_RB`, `Ethernet_WB`)
- Driver abstracts register-level access - application uses socket API

---

## 10. How does the control system send information between threads and devices?

**Answer:**

**Thread Communication (within each device):**

**Client:**
1. **Timer fires** → Signals `app_ctrl` thread with `FLAG_periodic`
2. **app_ctrl thread:**
   - Reads encoder, calculates velocity
   - Sets `FLAG_data_ready` to signal `app_comm` thread
   - Waits for `FLAG_control_received`
3. **app_comm thread:**
   - Waits for `FLAG_data_ready`
   - Sends `{velocity, timestamp}` to server via socket
   - Receives `{control}` from server
   - Sets `FLAG_control_received` to signal `app_ctrl`
4. **app_ctrl thread:**
   - Applies control signal to motor

**Server:**
1. **Timer fires** → Signals `app_ref` thread (reference) and `app_ctrl` thread (control timing)
2. **app_ref thread:** Flips reference direction every 4000 ms
3. **app_comm thread:**
   - Receives `{velocity, timestamp}` from client
   - Updates global variables
   - Sets `FLAG_new_data` to signal `app_ctrl`
4. **app_ctrl thread:**
   - Waits for `FLAG_new_data`
   - Calculates control using PI controller
   - Sets `FLAG_control_ready` to signal `app_comm`
5. **app_comm thread:**
   - Waits for `FLAG_control_ready`
   - Sends `{control}` to client

**Device-to-Device Communication:**
- TCP/IP socket connection over Ethernet
- Client sends velocity data → Server receives
- Server sends control data → Client receives
- Blocking `send()` and `recv()` calls ensure synchronization

**Synchronization mechanism:**
- RTOS thread flags for inter-thread signaling
- Global variables for data sharing (protected by flag synchronization)
- Blocking socket calls for device-to-device synchronization

---

## 11. What is the end-to-end delay?

**Answer:**

**End-to-end delay** is the time from sampling the encoder velocity to applying the control signal to the motor.

**Components of delay:**

1. **Encoder sampling:** ~0 ms (immediate read)
2. **Velocity calculation:** < 1 ms (simple arithmetic)
3. **Thread signaling:** < 1 ms (RTOS flag set)
4. **Network transmission (Client → Server):**
   - Serialization: < 0.1 ms
   - TCP/IP stack processing: ~1-2 ms
   - Ethernet transmission: ~0.1 ms (for 8 bytes)
   - **Total: ~2-3 ms**

5. **Server processing:**
   - Socket receive: ~1 ms
   - Control calculation: < 1 ms
   - **Total: ~2 ms**

6. **Network transmission (Server → Client):**
   - Similar to step 4: ~2-3 ms

7. **Client processing:**
   - Socket receive: ~1 ms
   - Motor actuation: < 0.1 ms
   - **Total: ~1 ms**

**Total end-to-end delay: ~7-9 ms**

**Measurement method:**
- Use System Analyzer to measure time between:
  - `Peripheral_Encoder_CalculateVelocity()` call
  - `Peripheral_PWM_ActuateMotor()` call
- Add network round-trip time (can be measured with timestamps)

**Impact on control:**
- Control period: 50 ms
- End-to-end delay: ~8 ms
- Delay ratio: 8/50 = 16% of control period
- **Acceptable** for this application (delay < 20% of period)

**Optimization possibilities:**
- Reduce network delay (faster Ethernet, shorter cable)
- Use UDP instead of TCP (lower latency, but less reliable)
- Optimize thread priorities
- Reduce buffer sizes

---

## 12. How is timing handled? Which device dictates the frequency?

**Answer:**

**Server dictates the control frequency:**

**Server side:**
- Uses virtual timer (`timer_ctrl`) with 50 ms period
- Timer fires → signals `app_ctrl` thread
- However, control calculation is **event-driven** by incoming data from client
- Reference generation uses separate timer (`timer_ref`) with 4000 ms period

**Client side:**
- Uses virtual timer (`timer_ctrl`) with 50 ms period
- Timer fires → `app_ctrl` reads encoder and calculates velocity
- Control application is **event-driven** by receiving control signal from server

**Why server dictates frequency:**
- Server generates reference signal (square wave)
- Server calculates control based on reference
- Client follows server's timing by responding to received control signals
- Prevents clock drift between devices

**Synchronization:**
- Client sends velocity data periodically (every 50 ms)
- Server receives data and calculates control
- Server sends control back
- Client applies control when received
- If server is slow, client waits (blocking `recv()`)
- If client is slow, server waits (blocking `recv()`)

**Preventing drift:**
- Only server uses timer for control loop timing
- Client is activated by incoming messages
- If messages stop, client detects timeout and stops motor

---

## 13. How is safety ensured? How does the motor stop within one sample?

**Answer:**

**Safety mechanisms:**

1. **Connection loss detection:**
   - `getsockopt(SO_STATUS)` checks socket state periodically
   - `send()` and `recv()` return error codes on failure
   - Timeout detection in thread flag waits

2. **Immediate motor stop:**
   ```c
   // In app-client.c, app_ctrl thread:
   uint32_t flags = osThreadFlagsWait(FLAG_control_received, osFlagsWaitAny, PERIOD_CTRL * 2);
   
   if (!(flags & FLAG_control_received) || connection_lost) {
     Peripheral_PWM_ActuateMotor(0);  // Stop immediately
     connection_lost = 1;
     osThreadFlagsSet(tid_app_main, FLAG_disconnected);
   }
   ```

3. **Connection loss handling:**
   - Communication thread sets `connection_lost = 1` on error
   - Control thread checks `connection_lost` flag
   - Main thread stops timers and disables motor
   - Controller is reset

4. **Timeout protection:**
   - Client waits max `PERIOD_CTRL * 2` (100 ms) for control signal
   - If timeout, motor stops immediately
   - Server waits max `PERIOD_CTRL` (50 ms) for control calculation
   - If timeout, connection marked as lost

**Why motor stops within one sample:**
- Control thread runs every 50 ms (one sample period)
- On each execution, it checks for connection loss
- If connection lost, motor stops before next sample
- Maximum delay: 50 ms (one control period)
- **Requirement met:** Motor stops within one sample

**Additional safety:**
- Motor disabled (`Peripheral_GPIO_DisableMotor()`) on disconnect
- Controller reset prevents integrator windup
- Timers stopped to prevent further execution
- Socket closed to free resources

---

## 14. How is the controller reset between sessions?

**Answer:**

**Controller reset occurs:**

1. **On new connection:**
   ```c
   // In Application_Loop, after successful connect/accept:
   Controller_Reset();
   ```

2. **On connection loss:**
   ```c
   // In Application_Loop, when connection_lost detected:
   Controller_Reset();
   ```

**What `Controller_Reset()` does:**
- Resets integrator to zero
- Resets previous time to zero
- Sets `first_call_after_reset = 1`
- Ensures clean start for next session

**Why reset is important:**
- Prevents initial overshoot from stale integrator value
- Ensures consistent behavior regardless of previous session state
- Sets t=0 for new control session
- Prevents undefined behavior from accumulated errors

**Timing:**
- Reset happens **before** starting timers
- Reset happens **before** enabling motor (client side)
- Ensures controller starts from known state

---

## 15. How does the system handle intermittent execution?

**Answer:**

**Control loop only runs when connected:**

**When disconnected:**
- Timers are stopped (`osTimerStop()`)
- Control threads wait but don't execute (check `connection_established` flag)
- Communication threads wait for `FLAG_connected`
- Motor is disabled
- Controller is reset

**When connected:**
- Timers are started (`osTimerStart()`)
- Control threads execute normally
- Communication threads enter active loop
- Motor is enabled
- Controller starts fresh (t=0)

**State transitions:**

```
DISCONNECTED → [Connection established] → CONNECTED
CONNECTED → [Connection lost] → DISCONNECTED
```

**Implementation:**
- `Application_Loop()` manages connection state
- Threads check `connection_established` and `connection_lost` flags
- Timers started/stopped dynamically
- No virtual timers running while disconnected (saves CPU)

**Benefits:**
- Saves power when disconnected
- Prevents unnecessary calculations
- Clean state transitions
- Easy to debug (clear on/off states)

---

## 16. What design choices were made and why?

**Answer:**

**1. Thread architecture:**
- **Separate communication thread:** Keeps network I/O non-blocking for control loop
- **High priority control thread:** Ensures timely motor control
- **Low priority main thread:** Connection management doesn't interfere with control

**2. Event-driven vs timer-driven:**
- **Server reference:** Timer-driven (needs periodic square wave)
- **Server control:** Event-driven (responds to incoming data)
- **Client sampling:** Timer-driven (needs periodic encoder reads)
- **Client control:** Event-driven (applies received control signal)

**3. Synchronization method:**
- **Thread flags:** Simple, low overhead, sufficient for this application
- **Global variables:** Shared data (protected by flag synchronization)
- **Blocking socket calls:** Natural synchronization between devices

**4. Safety approach:**
- **Timeout detection:** Prevents indefinite waiting
- **Immediate motor stop:** Priority on safety over smooth shutdown
- **Connection state flags:** Clear state machine

**5. Data structures:**
- **Structs for transmission:** Atomic data transfer, type safety
- **Shared header file:** Single source of truth

**Alternative approaches considered:**
- **Message queues:** More complex, unnecessary for simple data
- **Semaphores:** More overhead, flags sufficient
- **UDP instead of TCP:** Lower latency but less reliable (TCP chosen for reliability)
- **Non-blocking sockets:** More complex, blocking sufficient with RTOS threads

---

## 17. What are the limitations and potential improvements?

**Answer:**

**Current limitations:**

1. **No robustness to socket deadlocks:** If `close()` or `recv()` deadlocks, system hangs
2. **No automatic reconnection:** Manual reset may be needed
3. **Fixed IP addresses:** Not configurable at runtime
4. **No error recovery:** System stops on any network error
5. **Single client:** Server accepts only one connection
6. **No data validation:** No checksum or validation of received data

**Potential improvements:**

1. **Robustness (optional task):**
   - Separate thread to force disconnect on deadlock
   - State machine for all connection states
   - Automatic recovery from all failure modes

2. **Performance:**
   - Use UDP for lower latency (with application-level reliability)
   - Optimize thread priorities
   - Reduce buffer sizes

3. **Features:**
   - Multiple clients support
   - Configurable IP addresses
   - Data validation/checksums
   - Connection quality monitoring
   - Adaptive control period

4. **Safety:**
   - Watchdog timer for communication thread
   - Heartbeat mechanism
   - Graceful degradation

---

## Summary

This distributed control system successfully:
- ✅ Distributes functionality across two microcontrollers
- ✅ Uses TCP/IP socket communication
- ✅ Maintains control loop timing (50 ms period)
- ✅ Stops motor within one sample on connection loss
- ✅ Resets controller between sessions
- ✅ Uses RTOS for proper thread management
- ✅ Handles connection establishment and loss gracefully

The system demonstrates understanding of:
- Socket programming
- RTOS thread synchronization
- Distributed systems design
- Real-time control systems
- Safety-critical system design

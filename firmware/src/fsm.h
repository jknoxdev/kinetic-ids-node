#ifndef LIMA_FSM_H
#define LIMA_FSM_H

#include <zephyr/kernel.h>

/* ── State Definitions ───────────────────────────────────────────────────── */
typedef enum {
    STATE_BOOT,
    STATE_CALIBRATING,
    STATE_ARMED,
    STATE_EVENT_DETECTED,
    STATE_SIGNING,
    STATE_TRANSMITTING,
    STATE_COOLDOWN,
    STATE_FAULT,
    STATE_COUNT /* Useful for array bounds checking */
} lima_state_t;

/* ── Event Definitions ───────────────────────────────────────────────────── */
typedef enum {
    LIMA_EVT_INIT_COMPLETED,
    LIMA_EVT_SENSOR_TRIGGER,
    LIMA_EVT_SIGNING_DONE,
    LIMA_EVT_TX_COMPLETE,     /* Asynchronous BLE trigger */
    LIMA_EVT_TX_FAILED,       /* Broker retry logic trigger */
    LIMA_EVT_TIMEOUT,
    LIMA_EVT_ERROR
} lima_event_type_t;

/* ── Event Message Structure ─────────────────────────────────────────────── */
typedef struct {
    lima_event_type_t type;
    uint32_t timestamp;
    int code;                /* Optional error or status code */
} fsm_event_msg_t;

/* ── Public API ──────────────────────────────────────────────────────────── */

/**
 * @brief Initializes the FSM internal variables. 
 * Call this in main() before resuming threads.
 */
void fsm_init(void);

/**
 * @brief Core dispatcher. Routes events to state handlers.
 */
void fsm_dispatch(fsm_event_msg_t *evt);

/**
 * @brief Returns the current state. Useful for threads that need to poll 
 * state without being part of the FSM loop.
 */
lima_state_t fsm_get_state(void);

/**
 * @brief Helper to convert state enum to string for logging.
 */
const char* fsm_state_to_str(lima_state_t state);

/**
 * @brief Abstracted LED control. 
 * Allows the FSM logic to say "I'm in FAULT" without knowing which 
 * specific GPIO pin is the Red LED.
 */
void fsm_set_led_by_state(lima_state_t state);

#endif /* LIMA_FSM_H */
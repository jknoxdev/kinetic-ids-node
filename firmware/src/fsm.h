#ifndef LIMA_FSM_H
#define LIMA_FSM_H

#include <zephyr/kernel.h>
#include <stdbool.h>
#include "events.h"

/* ── State Definitions ───────────────────────────────────────────────────── */
typedef enum {
    STATE_BOOT,
    STATE_CALIBRATING,
    STATE_ARMED,
    STATE_LIGHT_SLEEP,    /* System ON: CPU idle, RAM on, fast wakeup */
    STATE_DEEP_SLEEP,     /* System OFF: Lowest power, RAM off, Reset wakeup */
    STATE_EVENT_DETECTED,
    STATE_SIGNING,
    STATE_TRANSMITTING,
    STATE_COOLDOWN,
    STATE_FAULT,
    STATE_COUNT,
    STATE_LOW_BATTERY
} lima_state_t;




// moved to events.h

// /* ── Event Definitions ───────────────────────────────────────────────────── */
// typedef enum {
//     LIMA_EVT_INIT_COMPLETED,
//     LIMA_EVT_SENSOR_TRIGGER,
//     LIMA_EVT_SIGNING_DONE,
//     LIMA_EVT_TX_COMPLETE,
//     LIMA_EVT_TX_FAILED,
//     LIMA_EVT_GOTO_SLEEP,      /* Trigger for Light Sleep */
//     LIMA_EVT_GOTO_DEEP_SLEEP, /* Trigger for System OFF */
//     LIMA_EVT_WAKEUP,
//     LIMA_EVT_TIMEOUT,
//     LIMA_EVT_ERROR

// } lima_event_type_t;

typedef struct {
    lima_event_type_t type;
    uint32_t timestamp;
} fsm_event_msg_t;

/* ── FSM Logic API ───────────────────────────────────────────────────────── */

void fsm_init(void);
void fsm_dispatch(const fsm_event_msg_t *evt);
lima_state_t fsm_get_state(void);
const char* fsm_state_to_str(lima_state_t state);

/* ── Hardware Abstraction Hooks (The "Stubs" for main.c) ────────────────── */

/**
 * @brief Light Sleep (System ON). 
 * CPU stops, but peripheral state and RAM are preserved.
 */
void fsm_hw_enter_sleep(void);

/**
 * @brief Deep Sleep (System OFF). 
 * chip powers down almost entirely. Next wakeup is a reboot.
 */
void fsm_hw_enter_deep_sleep(void);

/**
 * @brief Updates physical LEDs based on the FSM state.
 */
void fsm_hw_set_led(lima_state_t state);

const char* fsm_state_to_str(lima_state_t state);

#endif /* LIMA_FSM_H */
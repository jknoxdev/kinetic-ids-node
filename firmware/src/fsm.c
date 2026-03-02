#include <zephyr/logging/log.h>
#include "fsm.h"

LOG_MODULE_REGISTER(lima_fsm, LOG_LEVEL_INF);

/* ── Internal State (Private to this file) ──────────────────────────────── */
static lima_state_t current_state = STATE_BOOT;

/* ── Forward Declarations of Static Handlers ────────────────────────────── */
/* This stops the "Implicit Declaration" warnings */
static void state_boot_enter(void);
static void state_armed_enter(void);
static void state_armed_handle(const lima_event_t *evt);
// ... add the rest for TRANS_ENTER, etc ...

/* ── The Dispatcher ─────────────────────────────────────────────────────── */
void fsm_dispatch(const lima_event_t *evt) {
    while (1) {
        static lima_state_t last_state = (lima_state_t)-1;

        if (current_state != last_state) {
            last_state = current_state;

            switch (current_state) {
                case STATE_BOOT:        state_boot_enter();        break;
                case STATE_ARMED:       state_armed_enter();       break;
                case STATE_FAULT:       fsm_hw_set_led(STATE_FAULT); break;
                default: break;
            }
            continue; /* Run entry logic immediately */
        }
        break;
    }

    /* Route the event to the current state handler */
    switch (current_state) {
        case STATE_ARMED:   state_armed_handle(evt);   break;
        default: break;
    }
}

/* ── Implementation of Internal Handlers ────────────────────────────────── */
static void state_boot_enter(void) {
    LOG_INF("FSM: Entering BOOT");
    fsm_hw_set_led(STATE_BOOT);
}

static void state_armed_enter(void) {
    LOG_INF("FSM: System ARMED");
    fsm_hw_set_led(STATE_ARMED);
}

static void state_armed_handle(const lima_event_t *evt) {
    if (evt->type == LIMA_EVT_SENSOR_TRIGGER) {
        current_state = STATE_SIGNING; 
    }
}
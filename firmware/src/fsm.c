/*
 * L.I.M.A. — Local Integrity Multi-modal Architecture
 * fsm.c — State machine logic
 *
 * This file owns all state transition logic. It communicates with
 * hardware exclusively through the fsm_hw_* hooks implemented in main.c.
 * It posts events back to the queue via lima_post_event() declared in fsm.h.
 */

#include <zephyr/logging/log.h>
#include "fsm.h"

LOG_MODULE_REGISTER(lima_fsm, LOG_LEVEL_INF);

/* ── FSM Context (single instance) ──────────────────────────────────────── */
lima_fsm_ctx_t fsm = {
    .cooldown_ms   = 5000,
    .fault_retries = 0,
};

/* ── Internal State (Private to this file) ──────────────────────────────── */
static lima_state_t current_state = STATE_BOOT;

/* timeouts and their (handlers */
static struct k_work_delayable cooldown_work;
static struct k_work_delayable tx_timeout_work;

/* ── Forward Declarations ────────────────────────────────────────────────── */

static void transition(lima_state_t next);

/* Work/timer callbacks */
static void cooldown_expiry_cb(struct k_work *work);
static void tx_timeout_cb(struct k_work *work);

/* State entry functions */
static void state_boot_enter(void);
static void state_calibrating_enter(void);
static void state_armed_enter(void);
static void state_armed_exit(void);
static void state_light_sleep_enter(void);
static void state_deep_sleep_enter(void);
static void state_event_detected_enter(void);
static void state_signing_enter(void);
static void state_transmitting_enter(void);
static void state_cooldown_enter(void);
static void state_fault_enter(void);
static void state_low_battery_enter(void);

/* State event handlers */
static void state_armed_handle(const lima_event_t *evt);
static void state_light_sleep_handle(const lima_event_t *evt);
static void state_deep_sleep_handle(const lima_event_t *evt);
static void state_signing_handle(const lima_event_t *evt);
static void state_transmitting_handle(const lima_event_t *evt);
static void state_cooldown_handle(const lima_event_t *evt);
static void state_fault_handle(const lima_event_t *evt);
static void state_low_battery_handle(const lima_event_t *evt);

/* HAL stubs — real implementations declared in main.c via fsm.h */
/* These are the internal C wrappers around the hw_* functions in main.c.
   NOTE: hw_* functions must be wired to the
   fsm_hw_* trampolines below, or promoted to non-static + declared here. */

/* ── State Name Table ────────────────────────────────────────────────────── */

const char *fsm_state_to_str(lima_state_t state)
{
    static const char *names[STATE_COUNT] = {
        [STATE_BOOT]           = "BOOT",
        [STATE_CALIBRATING]    = "CALIBRATING",
        [STATE_ARMED]          = "ARMED",
        [STATE_LIGHT_SLEEP]    = "LIGHT_SLEEP",
        [STATE_DEEP_SLEEP]     = "DEEP_SLEEP",
        [STATE_EVENT_DETECTED] = "EVENT_DETECTED",
        [STATE_SIGNING]        = "SIGNING",
        [STATE_TRANSMITTING]   = "TRANSMITTING",
        [STATE_COOLDOWN]       = "COOLDOWN",
        [STATE_FAULT]          = "FAULT",
        [STATE_LOW_BATTERY]    = "LOW_BATTERY",
    };

    if (state >= STATE_COUNT) {
        return "UNKNOWN";
    }
    return names[state];
}




/* ── Transition Engine ───────────────────────────────────────────────────── */

static void transition(lima_state_t next)
{
    LOG_INF("FSM: %s -> %s",
            fsm_state_to_str(current_state),
            fsm_state_to_str(next));

    /* EXIT actions for the current state */
    switch (current_state) {
    case STATE_ARMED:
        state_armed_exit();
        break;
    case STATE_COOLDOWN:
        k_work_cancel_delayable(&cooldown_work);
        break;
    case STATE_TRANSMITTING:
        k_work_cancel_delayable(&tx_timeout_work);
        break;
    default:
        break;
    }

    current_state = next;
    fsm_hw_set_led(current_state);

    /* ENTRY actions for the new state */
    switch (current_state) {
    case STATE_BOOT:           state_boot_enter();           break;
    case STATE_CALIBRATING:    state_calibrating_enter();    break;
    case STATE_ARMED:          state_armed_enter();          break;
    case STATE_LIGHT_SLEEP:    state_light_sleep_enter();    break;
    case STATE_DEEP_SLEEP:     state_deep_sleep_enter();     break;
    case STATE_EVENT_DETECTED: state_event_detected_enter(); break;
    case STATE_SIGNING:        state_signing_enter();        break;
    case STATE_TRANSMITTING:   state_transmitting_enter();   break;
    case STATE_COOLDOWN:       state_cooldown_enter();       break;
    case STATE_FAULT:          state_fault_enter();          break;
    case STATE_LOW_BATTERY:    state_low_battery_enter();    break;
    default:                                                 break;
    }
}





/* ── Work Queue Callbacks ────────────────────────────────────────────────── */

static void cooldown_expiry_cb(struct k_work *work)
{
    ARG_UNUSED(work);
    lima_event_t e = {
        .type         = LIMA_EVT_COOLDOWN_EXPIRED,
        .timestamp_ms = k_uptime_get_32(),
    };
    LOG_INF("COOLDOWN: timer expired");
    lima_post_event(&e);
}

static void tx_timeout_cb(struct k_work *work)
{
    ARG_UNUSED(work);
    lima_event_t e = {
        .type         = LIMA_EVT_TX_TIMEOUT,
        .timestamp_ms = k_uptime_get_32(),
    };
    LOG_WRN("TRANSMITTING: timeout -> forcing COOLDOWN");
    lima_post_event(&e);
}





static void state_calibrating_enter(void)
{
    LOG_INF("CALIBRATING: enter function reached");  // ← add this
    // led_blink(2);
    LOG_INF("CALIBRATING: warming up sensors");

    int rc = hw_calibrate();
    if (rc != 0) {
        LOG_ERR("CALIBRATING: sensor error rc=%d -> FAULT", rc);
        transition(STATE_FAULT);
        return;
    }

    LOG_INF("CALIBRATING: baseline ready -> ARMED");
    transition(STATE_ARMED);

    // /* kick FSM thread to run ARMED entry */
    // lima_event_t e = {
    //     .type         = LIMA_EVT_POLL_TICK,
    //     .timestamp_ms = k_uptime_get_32(),
    // };
    // lima_post_event(&e);
    LOG_INF("CALIBRATING: kicked event");

}

/*
 * STATE_ARMED
 * Active monitoring state.
 * Sensor thread handles the actual polling and posts events here.
 * Either sensor alone is sufficient to trigger EVENT_DETECTED.
 */
// static void state_armed_enter(void)
// {    
//     // led_blink(3);
//     LOG_INF("ARMED: Sensors active. Heartbeat started.");
//     gpio_pin_set_dt(&led_r, 0);
//     gpio_pin_set_dt(&led_g, 0);

//     // /* Blink every 2 seconds (100ms on, then stays off until next cycle) */
//     // k_timer_start(&heartbeat_timer, K_MSEC(2000), K_MSEC(2000));
//     /* Timer now runs every 100ms to handle the rapid pulse pattern */
//     k_timer_start(&heartbeat_timer, K_MSEC(100), K_MSEC(100));

//     fsm.armed_since_ms = k_uptime_get_32();
//     hw_enable_irqs();
// }

static void state_armed_exit(void)
{
    k_timer_stop(&heartbeat_timer);
    // gpio_pin_set_dt(&led, 0); // Ensure LED is off when leaving ARMED
    gpio_pin_set_dt(&led_r, 0);
    gpio_pin_set_dt(&led_g, 0);
    gpio_pin_set_dt(&led_b, 0);
    LOG_DBG("ARMED: Heartbeat stopped.");
}


static void state_armed_handle(const lima_event_t *evt) {
    switch (evt->type) {
        case LIMA_EVT_PRESSURE_BREACH:
        case LIMA_EVT_MOTION_DETECTED:
        case LIMA_EVT_DUAL_BREACH:
        case LIMA_EVT_TAMPER_DETECTED:
            /* We don't need to copy to fsm.last_event anymore because 
               the event is passed into the next state if needed */
            transition(STATE_EVENT_DETECTED);
            break;

        case LIMA_EVT_POLL_TICK:
            transition(STATE_LIGHT_SLEEP);
            break;

        case LIMA_EVT_LOW_BATTERY:
        case LIMA_EVT_CRITICAL_BATTERY:
            transition(STATE_LOW_BATTERY);
            break;

        case LIMA_EVT_SENSOR_FAULT:
            transition(STATE_FAULT);
            break;

        default:
            LOG_WRN("ARMED: unhandled event type=0x%02X", evt->type);
            break;
    }
}

/*
 * STATE_LIGHT_SLEEP
 * Low power, sensor IRQs still active.
 * Returns to ARMED on any sensor candidate.
 * Falls to DEEP_SLEEP after inactivity threshold.
 */
static void state_light_sleep_enter(void)
{
    LOG_INF("LIGHT SLEEP: low-power, sensor IRQs active");
    hw_enter_light_sleep();
}

static void state_light_sleep_handle(const lima_event_t *evt)
{
    switch (evt->type) {
    case LIMA_EVT_PRESSURE_BREACH:
    case LIMA_EVT_MOTION_DETECTED:
    case LIMA_EVT_DUAL_BREACH:
    case LIMA_EVT_TAMPER_DETECTED:
        fsm.last_event = *evt;
        transition(STATE_EVENT_DETECTED);
        break;

    case LIMA_EVT_POLL_TICK:
        if ((k_uptime_get_32() - fsm.armed_since_ms) > SLEEP_INACTIVITY_MS) {
            lima_event_t e = {
                .type         = LIMA_EVT_SLEEP_TIMER_EXPIRY,
                .timestamp_ms = k_uptime_get_32(),
            };
            lima_post_event(&e);
        } else {
            transition(STATE_ARMED);
        }
        break;

    case LIMA_EVT_SLEEP_TIMER_EXPIRY:
        transition(STATE_DEEP_SLEEP);
        break;

    case LIMA_EVT_LOW_BATTERY:
    case LIMA_EVT_CRITICAL_BATTERY:
        transition(STATE_LOW_BATTERY);
        break;

    default:
        LOG_WRN("LIGHT_SLEEP: unhandled event type=%d", evt->type);
        break;
    }
}

/*
 * STATE_DEEP_SLEEP
 * BLE off, all peripherals suspended.
 * RTC wakeup only -> back to ARMED.
 */
static void state_deep_sleep_enter(void)
{
    LOG_INF("DEEP SLEEP: BLE off, RTC wakeup only");
    hw_ble_stop();
    hw_enter_deep_sleep();
}

static void state_deep_sleep_handle(const lima_event_t *evt)
{
    switch (evt->type) {
    case LIMA_EVT_RTC_WAKEUP:
        LOG_INF("DEEP SLEEP: RTC wakeup -> ARMED");
        /* TODO: reinit BLE stack */
        transition(STATE_ARMED);
        break;

    case LIMA_EVT_LOW_BATTERY:
    case LIMA_EVT_CRITICAL_BATTERY:
        transition(STATE_LOW_BATTERY);
        break;

    default:
        LOG_WRN("DEEP_SLEEP: unhandled event type=%d", evt->type);
        break;
    }
}

/*
 * STATE_EVENT_DETECTED
 * Latch trigger type + timestamp, hand off to SIGNING.
 * The event data is already in fsm.last_event from the transition.
 */
static void state_event_detected_enter(void)
{
    /* Heartbeat timer was stopped by state_armed_exit() automatically */
    /* Turn LED solid ON to indicate a 'Triggered' state */

    // gpio_pin_set_dt(&led, 1);

    /* Red Alert! */
    gpio_pin_set_dt(&led_r, 1);
    gpio_pin_set_dt(&led_g, 0);
    gpio_pin_set_dt(&led_b, 0);

    LOG_INF("EVENT DETECTED: type=%d at t=%u ms",
            fsm.last_event.type,
            fsm.last_event.timestamp_ms);



    hw_sign_event(&fsm.last_event);

    /* In real impl, CryptoCell posts SIGNING_COMPLETE async.
     * Stub posts it synchronously for now. */
    lima_event_t e = {
        .type         = LIMA_EVT_SIGNING_COMPLETE,
        .timestamp_ms = k_uptime_get_32(),
    };
    lima_post_event(&e);

    transition(STATE_SIGNING);
}

/*
 * STATE_SIGNING
 * CryptoCell-310 ECDSA-P256 sign + nonce.
 * On completion -> TRANSMITTING.
 */
static void state_signing_enter(void)
{
    LOG_INF("SIGNING: waiting for CryptoCell completion");
}

static void state_signing_handle(const lima_event_t *evt)
{
    switch (evt->type) {
    case LIMA_EVT_SIGNING_COMPLETE:
        LOG_INF("SIGNING: payload ready -> TRANSMITTING");
        transition(STATE_TRANSMITTING);
        break;

    case LIMA_EVT_SENSOR_FAULT:
        LOG_WRN("SIGNING: fault during signing -> FAULT");
        transition(STATE_FAULT);
        break;

    default:
        LOG_WRN("SIGNING: unhandled event type=%d", evt->type);
        break;
    }
}

/*
 * STATE_TRANSMITTING
 * BLE 5.0 Coded PHY advertisement with signed payload.
 * On TX confirm -> COOLDOWN. On max retries -> FAULT.
 */
static void state_transmitting_enter(void)
{
    LOG_INF("TRANSMITTING: advertising signed payload via BLE");


    /* Arm the timeout first */
    k_work_reschedule(&tx_timeout_work, K_MSEC(TX_TIMEOUT_MS));

    int rc = hw_ble_advertise(&fsm.last_event);
    if (rc != 0) {
        LOG_ERR("TRANSMITTING: BLE failed rc=%d -> FAULT", rc);
        lima_event_t e = {
            .type                   = LIMA_EVT_BLE_FAULT,
            .timestamp_ms           = k_uptime_get_32(),
            .data.fault.fault_code  = (uint8_t)rc,
        };
        lima_post_event(&e);
        return;
    }
    
    lima_event_t e = {
        .type         = LIMA_EVT_TX_COMPLETE,
        .timestamp_ms = k_uptime_get_32(),
    };
    lima_post_event(&e);
}

static void state_transmitting_handle(const lima_event_t *evt)
{
    switch (evt->type) {
    case LIMA_EVT_TX_COMPLETE:
        k_work_cancel_delayable(&tx_timeout_work);
        LOG_INF("TRANSMITTING: confirmed -> COOLDOWN");
        transition(STATE_COOLDOWN);
        break;
    
    case LIMA_EVT_TX_TIMEOUT:
    /* no confirm, but don’t stall forever */
    transition(STATE_COOLDOWN);
    break;

    case LIMA_EVT_BLE_FAULT:
        fsm.fault_retries = 0;
        transition(STATE_FAULT);
        break;

    default:
        LOG_WRN("TRANSMITTING: unhandled event type=%d", evt->type);
        break;
    }
}

/*
 * STATE_COOLDOWN
 * Suppress new events for configurable duration.
 * Prevents event storms (vehicle bounce, rack vibration).
 * TODO: replace k_sleep with k_work_delayable (non-blocking)
 */
static void state_cooldown_enter(void)
{
    uint32_t ms = fsm.cooldown_ms > 0 ? fsm.cooldown_ms : 5000;
    LOG_INF("COOLDOWN: suppressing events for %u ms", ms);

    k_work_reschedule(&cooldown_work, K_MSEC(ms));
    
    // k_sleep(K_MSEC(ms));   /* TODO: make non-blocking with k_work_delayable */


    lima_event_t e = {
        .type         = LIMA_EVT_COOLDOWN_EXPIRED,
        .timestamp_ms = k_uptime_get_32(),
    };
    // lima_post_event(&e);
}

static void state_cooldown_handle(const lima_event_t *evt)
{
    switch (evt->type) {
    case LIMA_EVT_COOLDOWN_EXPIRED:
        LOG_INF("COOLDOWN: expired -> ARMED [cite: 60]");
        transition(STATE_ARMED);
        break;

    case LIMA_EVT_LOW_BATTERY:
    case LIMA_EVT_TAMPER_DETECTED:
        /* 💡 ADDED: Immediate reaction to priority events  */
        LOG_WRN("COOLDOWN: priority event detected! Aborting timer.");
        k_work_cancel_delayable(&cooldown_work);
        
        if (evt->type == LIMA_EVT_TAMPER_DETECTED) {
            fsm.last_event = *evt;
            transition(STATE_EVENT_DETECTED);
        } else {
            transition(STATE_LOW_BATTERY);
        }
    //     break;
    // case LIMA_EVT_CRITICAL_BATTERY:
    //     transition(STATE_LOW_BATTERY);
    //     break;

    default:
        /* Suppress sensor events during cooldown -- by design */
        LOG_DBG("COOLDOWN: suppressed event type=%d", evt->type);
        break;
    }
}

/*
 * STATE_FAULT
 * Log to flash, assert fault LED, attempt recovery.
 * Recovery success -> CALIBRATING.
 * Max retries exceeded -> watchdog reset.
 */
static void state_fault_enter(void)
{
    LOG_ERR("FAULT: entered (retry %d/%d)", fsm.fault_retries, MAX_FAULT_RETRIES);
    hw_assert_fault_led();

    /* TODO: log fault type to flash */
    /* TODO: broadcast fault over BLE if stack is up */

    if (fsm.fault_retries >= MAX_FAULT_RETRIES) {
        LOG_ERR("FAULT: max retries exceeded -> watchdog reset");
        hw_watchdog_reset();
        return;
    }

    int rc = hw_try_recover();
    fsm.fault_retries++;

    lima_event_t e = {
        .type         = (rc == 0) ? LIMA_EVT_RECOVERY_SUCCESS
                                   : LIMA_EVT_RECOVERY_FAILED,
        .timestamp_ms = k_uptime_get_32(),
    };
    lima_post_event(&e);
}

static void state_fault_handle(const lima_event_t *evt)
{
    switch (evt->type) {
    case LIMA_EVT_RECOVERY_SUCCESS:
        LOG_INF("FAULT: recovery succeeded -> CALIBRATING");
        fsm.fault_retries = 0;
        transition(STATE_CALIBRATING);
        break;

    case LIMA_EVT_RECOVERY_FAILED:
        LOG_ERR("FAULT: recovery failed -> watchdog reset");
        hw_watchdog_reset();
        break;

    default:
        LOG_WRN("FAULT: unhandled event type=%d", evt->type);
        break;
    }
}

/*
 * STATE_LOW_BATTERY
 * Reduce poll rate, notify gateway, stay alert.
 * Restored -> ARMED. Critical -> shutdown.
 */
static void state_low_battery_enter(void)
{
    LOG_WRN("LOW BATTERY: reducing poll rate, notifying gateway");
    hw_notify_low_battery();
    /* TODO: reduce poll frequency */
    /* TODO: disable deep sleep cycling */
}

static void state_low_battery_handle(const lima_event_t *evt)
{
    switch (evt->type) {
    case LIMA_EVT_BATTERY_RESTORED:
        LOG_INF("LOW BATTERY: Vbat restored -> ARMED");
        transition(STATE_ARMED);
        break;

    case LIMA_EVT_CRITICAL_BATTERY:
        LOG_ERR("LOW BATTERY: critical Vbat -> shutdown");
        /* TODO: safe shutdown sequence */
        break;

    case LIMA_EVT_PRESSURE_BREACH:
    case LIMA_EVT_MOTION_DETECTED:
    case LIMA_EVT_DUAL_BREACH:
    case LIMA_EVT_TAMPER_DETECTED:
        /* Still detect and report events even on low battery */
        fsm.last_event = *evt;
        transition(STATE_EVENT_DETECTED);
        break;

    default:
        LOG_WRN("LOW_BATTERY: unhandled event type=%d", evt->type);
        break;
    }
}




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

// static void state_armed_handle(const lima_event_t *evt) {
//     if (evt->type == LIMA_EVT_SENSOR_TRIGGER) {
//         current_state = STATE_SIGNING; 
//     }
// }


/**
 * @brief Initializes the FSM to the starting state.
 */
void fsm_init(void) {
    LOG_INF("FSM: Initializing...");

    /* Start at the very beginning */
    current_state = STATE_BOOT;

    /* Optional: Trigger any specific startup hardware logic here, 
       like turning on a 'System Booting' LED */
    fsm_hw_set_led(STATE_BOOT);

    LOG_INF("FSM: Entered %s", fsm_state_to_str(current_state));
}




/**
 * @brief Thread-safe getter for the current FSM state
 */
lima_state_t fsm_get_state(void) {
    return current_state;
}


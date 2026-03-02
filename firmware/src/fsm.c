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




/* ── State: BOOT ─────────────────────────────────────────────────────────── */

static void state_boot_enter(void)
{
    LOG_INF("BOOT: initializing");
    /* Hardware init is handled by main.c before fsm_init() is called.
       If sensors failed, main.c should post LIMA_EVT_ERROR before this runs.
       On success, post INIT_COMPLETE to drive the transition. */
    lima_event_t e = {
        .type         = LIMA_EVT_INIT_COMPLETE,
        .timestamp_ms = k_uptime_get_32(),
    };
    lima_post_event(&e);
}







/* ── State: CALIBRATING ──────────────────────────────────────────────────── */

static void state_calibrating_enter(void)
{
    LOG_INF("CALIBRATING: warming up sensors");
    /* Calibration is a stub; a real impl would be async and post
       LIMA_EVT_BASELINE_READY when done. For now post it directly. */
    lima_event_t e = {
        .type         = LIMA_EVT_BASELINE_READY,
        .timestamp_ms = k_uptime_get_32(),
    };
    lima_post_event(&e);
}








/* ── State: ARMED ────────────────────────────────────────────────────────── */

static void state_armed_enter(void)
{
    LOG_INF("ARMED: sensors active, heartbeat started");
    fsm.armed_since_ms = k_uptime_get_32();
    /* LED heartbeat is started by fsm_hw_set_led(STATE_ARMED) in main.c */
}

static void state_armed_exit(void)
{
    LOG_DBG("ARMED: exit — heartbeat stopped");
    /* Heartbeat stop is handled by fsm_hw_set_led() for the next state,
       but we call into main.c's hook explicitly to be safe. */
    fsm_hw_enter_sleep(); /* no-op unless overridden */
}

static void state_armed_handle(const lima_event_t *evt) 
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



/* ── State: LIGHT_SLEEP ──────────────────────────────────────────────────── */

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
        LOG_WRN("LIGHT_SLEEP: unhandled event 0x%02X", evt->type);
        break;
    }
}


/* ── State: DEEP_SLEEP ───────────────────────────────────────────────────── */

static void state_deep_sleep_enter(void)
{
    LOG_INF("DEEP SLEEP: BLE off, RTC wakeup only");
    fsm_hw_enter_deep_sleep();
}

static void state_deep_sleep_handle(const lima_event_t *evt)
{
    switch (evt->type) {
    case LIMA_EVT_RTC_WAKEUP:
        LOG_INF("DEEP SLEEP: RTC wakeup -> ARMED");
        transition(STATE_ARMED);
        break;

    case LIMA_EVT_LOW_BATTERY:
    case LIMA_EVT_CRITICAL_BATTERY:
        transition(STATE_LOW_BATTERY);
        break;

    default:
        LOG_WRN("DEEP_SLEEP: unhandled event 0x%02X", evt->type);
        break;
    }
}

/* ── State: EVENT_DETECTED ───────────────────────────────────────────────── */

static void state_event_detected_enter(void)
{
    LOG_INF("EVENT DETECTED: type=0x%02X at t=%u ms",
            fsm.last_event.type,
            fsm.last_event.timestamp_ms);

    /* Kick off async signing; stub posts SIGNING_COMPLETE synchronously */
    lima_event_t e = {
        .type         = LIMA_EVT_SIGNING_COMPLETE,
        .timestamp_ms = k_uptime_get_32(),
    };
    lima_post_event(&e);

    transition(STATE_SIGNING);
}


/* ── State: SIGNING ──────────────────────────────────────────────────────── */

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
        LOG_WRN("SIGNING: unhandled event 0x%02X", evt->type);
        break;
    }
}

/* ── State: TRANSMITTING ─────────────────────────────────────────────────── */

static void state_transmitting_enter(void)
{
    LOG_INF("TRANSMITTING: advertising signed payload via BLE");

    k_work_reschedule(&tx_timeout_work, K_MSEC(TX_TIMEOUT_MS));

    /* Stub: real impl fires callback on BLE completion */
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
        LOG_WRN("TRANSMITTING: timeout -> COOLDOWN");
        transition(STATE_COOLDOWN);
        break;

    case LIMA_EVT_BLE_FAULT:
        fsm.fault_retries = 0;
        transition(STATE_FAULT);
        break;

    default:
        LOG_WRN("TRANSMITTING: unhandled event 0x%02X", evt->type);
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


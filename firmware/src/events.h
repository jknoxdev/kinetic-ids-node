/*
 * L.I.M.A. — Local Integrity Multi-modal Architecture
 * events.h — Event types and payload struct
 *
 * All inter-thread communication flows through lima_event.
 * ISRs and sensor threads post to the FSM message queue;
 * the FSM thread consumes and transitions state.
 */

#ifndef LIMA_EVENTS_H
#define LIMA_EVENTS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Event types ──────────────────────────────────────────────────────────── */

typedef enum {
    /* ── Sensor Triggers (0x01 - 0x07) ───────────────────────────────────── */
    LIMA_EVT_PRESSURE_BREACH    = 0x01,  /* BMP280: pressure delta > threshold  */
    LIMA_EVT_MOTION_DETECTED    = 0x02,  /* MPU6050: acceleration > threshold   */
    LIMA_EVT_DUAL_BREACH        = 0x03,  /* Both sensors triggered              */
    LIMA_EVT_SENSOR_TRIGGER     = 0x04,  /* General/Generic sensor trigger      */
    LIMA_EVT_TAMPER_DETECTED    = 0x05,  /* Case open / voltage spike           */
    LIMA_EVT_WAKEUP             = 0x06,  /* General wakeup (RTC or GPIO)        */
    LIMA_EVT_RTC_WAKEUP         = 0x07,  /* RTC only woke from deep sleep       */

    /* ── Node Health & Battery (0x08 - 0x0F) ─────────────────────────────── */
    LIMA_EVT_LOW_BATTERY        = 0x08,  /* Vbat < low threshold                */
    LIMA_EVT_CRITICAL_BATTERY   = 0x09,  /* Vbat < critical threshold -> shutdown*/
    LIMA_EVT_BATTERY_RESTORED   = 0x0A,  /* Vbat recovered                      */
    LIMA_EVT_INIT_COMPLETED     = 0x0B,  /* Node init completed                 */


    /* ── Lifecycle & Timing (0x10 - 0x1F) ────────────────────────────────── */
    LIMA_EVT_INIT_COMPLETE      = 0x10,  /* Boot init done, watchdog armed      */
    LIMA_EVT_BASELINE_READY     = 0x11,  /* Calibration complete                */
    LIMA_EVT_POLL_TICK          = 0x12,  /* Poll interval elapsed (no event)    */
    LIMA_EVT_GOTO_SLEEP         = 0x13,  /* Manual trigger for Light Sleep      */
    LIMA_EVT_GOTO_DEEP_SLEEP    = 0x14,  /* Manual trigger for System OFF       */
    LIMA_EVT_SLEEP_TIMER_EXPIRY = 0x15,  /* Inactivity -> deep sleep            */
    LIMA_EVT_COOLDOWN_EXPIRED   = 0x16,  /* Cooldown timer done -> rearm        */
    LIMA_EVT_TIMEOUT            = 0x17,  /* General purpose FSM timeout         */

    /* ── BLE Broker & Signing (0x20 - 0x2F) ──────────────────────────────── */
    LIMA_EVT_SIGNING_COMPLETE   = 0x20,  /* Payload signed and ready            */
    LIMA_EVT_TX_COMPLETE        = 0x21,  /* BLE advertisement confirmed         */
    LIMA_EVT_TX_TIMEOUT         = 0x22,  /* TX did not confirm in time          */
    LIMA_EVT_TX_FAILED          = 0x23,  /* Advertisement failed (max retries)  */

    /* ── Faults & Recovery (0x30 - 0x3F) ─────────────────────────────────── */
    LIMA_EVT_SENSOR_FAULT       = 0x30,  /* I2C error / sensor dropout          */
    LIMA_EVT_BLE_FAULT          = 0x31,  /* BLE TX failed (max retries)         */
    LIMA_EVT_RECOVERY_SUCCESS   = 0x32,  /* Fault recovery succeeded            */
    LIMA_EVT_RECOVERY_FAILED    = 0x33,  /* Unrecoverable -> watchdog reset     */
    LIMA_EVT_ERROR              = 0x3F   /* General system logic error          */

} lima_event_type_t;


/* ── Sensor payload variants ─────────────────────────────────────────────── */

typedef struct {
    float   delta_pa;        /* Pressure delta from baseline (Pascals) */
    float   abs_hpa;         /* Absolute reading (hPa)                 */
} lima_baro_data_t;

typedef struct {
    float   accel_g;         /* Peak acceleration magnitude (g)        */
    float   gyro_dps;        /* Peak gyro magnitude (deg/s)            */
} lima_imu_data_t;

typedef struct {
    uint8_t fault_code;      /* Hardware fault identifier              */
    uint8_t retry_count;     /* How many recovery attempts so far      */
} lima_fault_data_t;

typedef struct {
    uint16_t mv;             /* Battery voltage in millivolts          */
} lima_battery_data_t;

/* ── Master event struct ─────────────────────────────────────────────────── */

typedef struct {
    lima_event_type_t   type;
    uint32_t            timestamp_ms;   /* k_uptime_get_32() at event creation */
    uint8_t             node_id[6];     /* BLE MAC address of this node        */

    union {
        lima_baro_data_t    baro;
        lima_imu_data_t     imu;
        lima_fault_data_t   fault;
        lima_battery_data_t battery;
    } data;
} lima_event_t;

#ifdef __cplusplus
}
#endif

#endif /* LIMA_EVENTS_H */

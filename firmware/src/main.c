/*
 * ============================================================================
 * Project: L.I.M.A. (Local Integrity Multi-modal Architecture)
 * File:    main.c
 * Author:  Justin Knox
 * Date:    Feb 2026
 *
 * Description: 
 * Event-driven FSM for edge integrity monitoring. Uses a decoupled 
 * thread model to separate high-frequency sensor polling from 
 * cryptographic signing and BLE transmission states.
 *
 * Architecture:
 * ├── #includes & defines     (System & Zephyr RTOS config)
 * ├── Hardware Globals        (LEDs, MPU6050, Message Queues)
 * ├── HAL & Stubs             (I2C Recovery, IMU Read, Crypto/BLE stubs)
 * ├── FSM Engine              (State transitions & Entry/Exit logic)
 * ├── fsm_dispatch()          (Event-to-State routing)
 * ├── Thread Functions        (sensor_thread_fn, fsm_thread_fn)
 * ├── K_THREAD_DEFINE x2      (Static thread allocation)
 * └── main()                  (Hardware init & Thread resumption)
 * ============================================================================
 */

#include <zephyr/kernel.h>
#include <zephyr/kernel/thread.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <math.h>
#include "events.h"
#include "fsm.h"

LOG_MODULE_REGISTER(lima_main, LOG_LEVEL_INF);

#define I2C0_SCL_PIN 19
#define I2C0_SDA_PIN 20

/* ── Board definitions ───────────────────────────────────────────────────── */

#define LED_R_NODE DT_ALIAS(led0)
#define LED_G_NODE DT_ALIAS(led1)
#define LED_B_NODE DT_ALIAS(led2)

/* ── Configuration ───────────────────────────────────────────────────────── */

#define FSM_MSGQ_DEPTH          16
#define FSM_THREAD_PRIORITY     5
#define SENSOR_THREAD_PRIORITY  6
#define POLL_INTERVAL_MS        60      /* 16.67 Hz — matches your working blinky */
#define COOLDOWN_MS_DEFAULT     5000    /* 5s default; tune per deployment         */
#define SLEEP_INACTIVITY_MS     30000   /* 30s no event → deep sleep               */
#define MAX_FAULT_RETRIES       3
#define MOTION_THRESHOLD_G      0.80     /* 1.1 good for table top     */
#define FSM_STACK_SIZE          8192  // Double it again
#define SENSOR_STACK_SIZE       4096  // Double it again
#define TX_TIMEOUT_MS           1500     /* tune later */

/* ── Hardware globals ────────────────────────────────────────────────────── */

static const struct gpio_dt_spec led_r = GPIO_DT_SPEC_GET(LED_R_NODE, gpios);
static const struct gpio_dt_spec led_g = GPIO_DT_SPEC_GET(LED_G_NODE, gpios);
static const struct gpio_dt_spec led_b = GPIO_DT_SPEC_GET(LED_B_NODE, gpios);

static const struct device *mpu;
static struct sensor_value accel[3];

/* ── Message queue ───────────────────────────────────────────────────────── */

K_MSGQ_DEFINE(fsm_msgq, sizeof(lima_event_t), FSM_MSGQ_DEPTH, 4);

int lima_post_event(const lima_event_t *evt)
{
    return k_msgq_put(&fsm_msgq, evt, K_NO_WAIT);
}



/* Non-blocking timer for cooldown */
static struct k_work_delayable cooldown_work; 

/* Non-blocking timer tx timeout */
static struct k_work_delayable tx_timeout_work;



/* Threads */
static void sensor_thread_fn(void *p1, void *p2, void *p3);
static void fsm_thread_fn(void *p1, void *p2, void *p3);



/* ── Heartbeat timer (ARMED state blue pulse) ────────────────────────────── */

static void heartbeat_expiry_fn(struct k_timer *timer_id);
K_TIMER_DEFINE(heartbeat_timer, heartbeat_expiry_fn, NULL);

static void heartbeat_expiry_fn(struct k_timer *timer_id)
{
    ARG_UNUSED(timer_id);
    static uint8_t tick = 0;

    /* Double-blink pattern: on at tick 0 and 2, off otherwise.
     * Period = 20 ticks × 100 ms = 2 seconds. */
    bool led_on = (tick == 0 || tick == 2);
    gpio_pin_set_dt(&led_b, led_on ? 1 : 0);
    tick = (tick + 1) % 20;
}





/* ── Message queue ───────────────────────────────────────────────────────── */

K_MSGQ_DEFINE(fsm_msgq, sizeof(lima_event_t), FSM_MSGQ_DEPTH, 4);

/* Change the parameter to a const pointer */
int lima_post_event(const lima_event_t *evt) {
    /* Send the actual data to the queue */
    return k_msgq_put(&fsm_msgq, evt, K_NO_WAIT); 
}















/* ── Hardware drivers ────────────────────────────────────────────────────── */

/*
 * hw_init_sensors()
 * Real implementation — replaces stub.
 * Initializes MPU6050 and LED from device tree.
 */
static int hw_init_sensors(void)
{
    /* MPU6050 */
    mpu = DEVICE_DT_GET_ANY(invensense_mpu6050);

    if (mpu == NULL) {
        LOG_ERR("MPU6050 device not found in devicetree!");
        return -ENODEV;
    }
    
    if (!device_is_ready(mpu)) {
        LOG_ERR("MPU6050 not ready!");
        return -ENODEV;
    }
    LOG_INF("MPU6050 ready");



    // /* LED heartbeat */
    // if (!gpio_is_ready_dt(&led)) {
    //     LOG_ERR("LED not ready!");
    //     return -ENODEV;
    // }
    // gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
    // LOG_INF("LED ready");

    return 0;
}

/*
 * hw_read_imu()
 * Fetch a fresh accel sample from the MPU6050.
 * Returns magnitude in g, populates global accel[].
 */
static double hw_read_imu(void)
{
    sensor_sample_fetch(mpu);
    sensor_channel_get(mpu, SENSOR_CHAN_ACCEL_XYZ, accel);

    double ax = sensor_value_to_double(&accel[0]);
    double ay = sensor_value_to_double(&accel[1]);
    double az = sensor_value_to_double(&accel[2]);

     // Magnitude in m/s^2
    double mag_ms2 = sqrt(ax*ax + ay*ay + az*az);
    
    double current_g = mag_ms2 / 9.80665;

    // Normalize: Subtract Earth's gravity (approx 9.806)
    // and convert to Gs. This makes 'rest' approximately 0.0g.

    // magnitue
    // double mag_g = fabs((mag_ms2 / 9.80665) - 1.0);
    // LOG_DBG("accel x:%.2f y:%.2f z:%.2f | G:%.2f", ax, ay, az, mag_g);
    // return mag_g;

    // g's
    double motion_g = fabs(current_g - 1.0);
    LOG_DBG("Raw: %.2f m/s2 | Motion: %.2f G", mag_ms2, motion_g);
    return motion_g;
}

/* ── Remaining stubs (swap these in one at a time) ───────────────────────── */

static int hw_calibrate(void)
{
    /* TODO: collect baseline pressure + accel samples, store to SRAM */
    LOG_INF("[STUB] calibrate — using MPU6050 ready check as baseline");
    return 0;
}

static int hw_enable_irqs(void)
{
    /* TODO: configure MPU6050 interrupt pin via GPIO */
    LOG_INF("[STUB] enable_sensor_irqs");
    return 0;
}

static int hw_enter_light_sleep(void)
{
    /* TODO: disable non-critical peripherals via Zephyr PM */
    LOG_INF("[STUB] light_sleep");
    return 0;
}

static int hw_enter_deep_sleep(void)
{
    /* TODO: suspend all peripherals, BLE off, RTC wakeup only */
    LOG_INF("[STUB] deep_sleep");
    return 0;
}

static int hw_sign_event(lima_event_t *e)
{
    /* TODO: CryptoCell-310 ECDSA-P256 sign + per-event nonce */
    LOG_INF("[STUB] sign_event type=%d", e->type);
    return 0;
}

static int hw_ble_advertise(lima_event_t *e)
{
    /* TODO: BLE 5.0 Coded PHY advertisement with signed payload */
    LOG_INF("[STUB] ble_advertise type=%d", e->type);
    return 0;
}

static int hw_ble_stop(void)
{
    /* TODO: disable BLE stack for deep sleep */
    LOG_INF("[STUB] ble_stop");
    return 0;
}

static void hw_assert_fault_led(void)
{
    /* TODO: solid fault LED pattern (vs heartbeat blink) */
    LOG_INF("[STUB] assert_fault_led");
}

static int hw_try_recover(void)
{
    /* TODO: attempt I2C re-init x3 then watchdog reset */
    LOG_INF("[STUB] try_recover");
    return 0;
}

static void hw_watchdog_reset(void)
{
    /* TODO: trigger watchdog reset via Zephyr WDT API */
    LOG_INF("[STUB] watchdog_reset");
}

static void hw_notify_low_battery(void)
{
    /* TODO: BLE notify gateway of low battery state */
    LOG_INF("[STUB] notify_low_battery");
}


static void hw_i2c_bus_recovery(void)
{
    LOG_INF("BOOT: Performing I2C bus recovery on P0.19/P0.20...");

    /* Get the device pointer for GPIO Port 0 */
    const struct device *gpio0_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));

    if (!device_is_ready(gpio0_dev)) {
        LOG_ERR("GPIO0 device not ready for recovery");
        return;
    }

    /* Configure SCL as output, SDA as input */
    gpio_pin_configure(gpio0_dev, I2C0_SCL_PIN, GPIO_OUTPUT_HIGH);
    gpio_pin_configure(gpio0_dev, I2C0_SDA_PIN, GPIO_INPUT);

    /* Clock out 9 pulses to release SDA */
    for (int i = 0; i < 9; i++) {
        gpio_pin_set_raw(gpio0_dev, I2C0_SCL_PIN, 1);
        k_busy_wait(5);
        gpio_pin_set_raw(gpio0_dev, I2C0_SCL_PIN, 0);
        k_busy_wait(5);
    }
    
    /* Leave SCL high (idle state) */
    gpio_pin_set_raw(gpio0_dev, I2C0_SCL_PIN, 1);
    
    /* 💡 CRITICAL: We must "unconfigure" the pins so the I2C driver 
       can take control of them again when it initializes later. */
    gpio_pin_configure(gpio0_dev, I2C0_SCL_PIN, GPIO_DISCONNECTED);
    gpio_pin_configure(gpio0_dev, I2C0_SDA_PIN, GPIO_DISCONNECTED);

    LOG_INF("BOOT: I2C recovery complete");
}



/* ── Sensor thread — real MPU6050 polling at 16.67 Hz ───────────────────── */

static void sensor_thread_fn(void *p1, void *p2, void *p3)
{
    LOG_INF("FSM sensor thread resumed!");  // ← add this as very first line
    ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);

    LOG_INF("Sensor thread started");

    /* Wait for FSM to finish BOOT + CALIBRATING before polling */
    k_sleep(K_MSEC(500));

    while (1) {
        /* Only poll when FSM is in a watching state */
        if (fsm_get_state() == STATE_ARMED || fsm_get_state() == STATE_LIGHT_SLEEP) {

            double magnitude = hw_read_imu();

            /* Toggle LED as heartbeat — visible proof the node is alive */
            // gpio_pin_toggle_dt(&led);

            if (magnitude > MOTION_THRESHOLD_G) {
                LOG_INF("MOTION: magnitude=%.2f g (threshold=%.2f)",
                        magnitude, MOTION_THRESHOLD_G);

                lima_event_t e = {
                    .type              = LIMA_EVT_MOTION_DETECTED,
                    .timestamp_ms      = k_uptime_get_32(),
                    .data.imu.accel_g  = (float)magnitude,
                    .data.imu.gyro_dps = 0.0f, /* TODO: add gyro channel read */
                };
                lima_post_event(&e);
            }
        }

        k_msleep(POLL_INTERVAL_MS); /* 60ms = 16.67 Hz */
    }
}

/* ── FSM thread ──────────────────────────────────────────────────────────── */

static void fsm_thread_fn(void *p1, void *p2, void *p3)
{
    LOG_INF("FSM thread resumed!");  // ← add this as very first line
    ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);

    LOG_INF("LIMA FSM thread started");

 

    lima_event_t msg;
    fsm_init(); // Set initial state inside fsm.c

    while (1) {
        // Wait for message from queue...
        if (k_msgq_get(&fsm_msgq, &msg, K_FOREVER) == 0) {
            fsm_dispatch(&msg); // Send it to the brain
        }
    }
}

//  * Thread startup:
//  *   Threads are defined with K_FOREVER (no auto-start).
//  *   main() blinks LED to let USB settle, then calls:
//  *       k_thread_start(fsm_thread);
//  *       k_thread_start(sensor_thread);
//  *   This prevents the USB suspend/reset cycle from
//  *   interrupting FSM boot sequencing.

K_THREAD_DEFINE(fsm_thread, FSM_STACK_SIZE,
                fsm_thread_fn, NULL, NULL, NULL,
                FSM_THREAD_PRIORITY, 0, 0);

K_THREAD_DEFINE(sensor_thread, SENSOR_STACK_SIZE,
                sensor_thread_fn, NULL, NULL, NULL,
                SENSOR_THREAD_PRIORITY, 0, 0);



/* ── FSM Hardware Stubs ────────────────────────────────────────────────── */
/* These will be filled in once the o-scope arrives! */

void fsm_hw_enter_sleep(void) {
    LOG_INF("HAL: Entering Light Sleep (System ON)");
    // k_cpu_idle(); 
}

void fsm_hw_enter_deep_sleep(void) {
    LOG_INF("HAL: Entering Deep Sleep (System OFF)");
    // sys_power_state_set();
}

/**
 * @brief Physical hardware response to state changes.
 * Called by fsm.c during transitions.
 */
void fsm_hw_set_led(lima_state_t state) {
    /* Reset all colors first for a clean slate */
    gpio_pin_set_dt(&led_r, 0);
    gpio_pin_set_dt(&led_g, 0);
    gpio_pin_set_dt(&led_b, 0);

    switch (state) {
        case STATE_BOOT:
            /* White (R+G+B) for initialization */
            gpio_pin_set_dt(&led_r, 1);
            gpio_pin_set_dt(&led_g, 1);
            gpio_pin_set_dt(&led_b, 1);
            break;

        case STATE_ARMED:
            /* Start heartbeat (Blue pulse) */
            k_timer_start(&heartbeat_timer, K_MSEC(100), K_MSEC(100));
            break;

        case STATE_EVENT_DETECTED:
        case STATE_SIGNING:
        case STATE_TRANSMITTING:
            /* Solid Red during activity */
            gpio_pin_set_dt(&led_r, 1);
            break;

        case STATE_COOLDOWN:
            /* Yellow (R+G) for cooldown/suppression */
            gpio_pin_set_dt(&led_r, 1);
            gpio_pin_set_dt(&led_g, 1);
            break;

        case STATE_FAULT:
            /* Rapid Red flash or solid red (Safety first) */
            k_timer_stop(&heartbeat_timer);
            gpio_pin_set_dt(&led_r, 1);
            LOG_ERR("HARDWARE: Fault LED Asserted");
            break;

        default:
            /* Dark for sleep states */
            break;
    }
}


/* ── main ────────────────────────────────────────────────────────────────── */

int main(void)
{
    // suspend threads immediately before USB chaos starts
    k_thread_suspend(fsm_thread);
    k_thread_suspend(sensor_thread);

    // 2. Clear the I2C bus manually before doing ANYTHING else
    // This prevents a hung sensor from blocking the driver later
    hw_i2c_bus_recovery();

    /* Init RGB Pins */
    gpio_pin_configure_dt(&led_r, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&led_g, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&led_b, GPIO_OUTPUT_INACTIVE);

    // /* Temporary test: Force the LED to stay on regardless of FSM */
    // gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE); 
    // gpio_pin_set_dt(&led, 1); // If ACTIVE_LOW is set in DTS, this should light it up.
    
    LOG_INF("L.I.M.A. node firmware starting");
    
    // 3. Increase the wait and ensure USB is stable
    // We wait 6 seconds now to be absolutely sure the host has finished enumeration
    for (int i = 0; i < 6; i++) {
        // led_blink(1);
        k_msleep(1000);
        LOG_INF("USB Settle Loop: %d/6", i+1);
    }
    
    LOG_INF("Starting LIMA Threads...");
    k_thread_resume(fsm_thread);
    k_thread_resume(sensor_thread);

    // initialize the tx_timeout for non-BLE mode
    k_work_init_delayable(&tx_timeout_work, tx_timeout_cb);

    return 0;
}

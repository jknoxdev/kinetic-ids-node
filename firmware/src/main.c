#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

/* L.I.M.A. Status Blinker: 3.03Hz*/
#define SLEEP_TIME_MS 110
#define LED0_NODE DT_ALIAS(led0)

LOG_MODULE_REGISTER(lima, LOG_LEVEL_DBG);

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

int main(void)
{
    LOG_INF("LIMA node starting...");

    if (!gpio_is_ready_dt(&led)) {
        return 0;
    }

    gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);

    while (1) {
        gpio_pin_toggle_dt(&led);
        k_msleep(SLEEP_TIME_MS); // Cooperative sleep allows BT stack to run
        // LOG_DBG("heartbeat");   
        printk(".");
    }
    return 0;
}
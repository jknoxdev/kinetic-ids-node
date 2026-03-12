# LIMA Wiremap: MDK USB Dongle → nRF52840-DK (PCA10056)

Maps dongle GPIO assignments as closely as possible to equivalent DK header pins.
All pins are on P0/P1 headers (connectors P1/P2 on the DK).

---

## Peripheral Assignments (LIMA)

| Function         | Dongle Pin | DK Header Pin | DK Connector | Notes                        |
|------------------|------------|---------------|--------------|------------------------------|
| **I2C SDA**      | P0.05      | P0.05         | P1.03        | MPU6050 + BME280             |
| **I2C SCL**      | P0.04      | P0.04         | P1.05        | MPU6050 + BME280             |
| **UART TX**      | P0.06      | P0.06         | P1.07        | USB CDC debug (was CDC)      |
| **UART RX**      | P0.07      | P0.08         | P1.09        | P0.07 used for LED on DK¹    |
| **INT (MPU)**    | P0.03      | P0.03         | P1.04        | MPU6050 interrupt line       |
| **GPIO spare**   | P0.02      | P0.02         | P1.02        | Available for future use     |
| **GPIO spare**   | P0.28      | P0.28         | P2.12        | ADC-capable                  |
| **GPIO spare**   | P0.29      | P0.29         | P2.13        | ADC-capable                  |
| **GPIO spare**   | P0.30      | P0.30         | P2.14        | ADC-capable                  |
| **GPIO spare**   | P0.31      | P0.31         | P2.15        | ADC-capable                  |

---

## Onboard Peripherals (no wires needed — already on DK)

| Function              | Dongle Pin | DK Equivalent  | Active |
|-----------------------|------------|----------------|--------|
| **RGB LED - Red**     | P0.23      | LED1 → P0.13   | LOW    |
| **RGB LED - Green**   | P0.22      | LED2 → P0.14   | LOW    |
| **RGB LED - Blue**    | P0.24      | LED3 → P0.15   | LOW    |
| **User/Reset Button** | P0.18      | BTN1 → P0.11   | LOW    |

> ⚠️ DK LEDs and buttons are different pins than the dongle. Update your `board.rs`
> or DTS overlay `aliases` to match — do NOT assume they are the same.

---

## Power & Debug

| Signal    | Dongle         | DK                          |
|-----------|----------------|-----------------------------|
| 3V3       | Edge pin (out) | P1.01 (VDD) or Arduino 3V3  |
| GND       | Edge pin       | Any GND on P1/P2            |
| VIN       | 3.3–5.5V in    | USB powered — no VIN needed |
| SWDCLK    | Edge pad       | On-board J-Link (built-in)  |
| SWDIO     | Edge pad       | On-board J-Link (built-in)  |
| RESET     | Edge pad       | On-board reset button       |

> ✅ The DK has a built-in J-Link — no external debug probe needed for flashing.
> Your J-Link EDU Mini is still useful for targeting the dongle itself later.

---

## Notes

¹ P0.07 on the DK is not broken out to a header — closest safe swap is P0.08.
  Update your `uart0` DTS overlay node to use `tx-pin = <6>; rx-pin = <8>;`

² DK connector reference: P1 = "Arduino/long header", P2 = "short 2x8 header"
  Full mapping: https://docs.nordicsemi.com/bundle/ug_nrf52840_dk/page/UG/dk/intro.html

---

## DTS Overlay Snippet (nrf52840dk_nrf52840.overlay)

```dts
&i2c0 {
    compatible = "nordic,nrf-twi";
    status = "okay";
    sda-pin = <5>;   /* P0.05 */
    scl-pin = <4>;   /* P0.04 */
};

&uart0 {
    compatible = "nordic,nrf-uarte";
    status = "okay";
    tx-pin = <6>;    /* P0.06 */
    rx-pin = <8>;    /* P0.08 — swapped from dongle P0.07 */
};
```

---

*Generated for LIMA project — nRF52840-DK (PCA10056) migration sprint*

# lima-node
L.I.M.A. > (Local Integrity Multi-modal Architecture)
The lima-node is a low-power kinetic / physical intrusion detection system. Utilizing a Zephyr RTOS micro-kernel on the nRF52840, the system implements hardware-accelerated motion discrimination via an MPU6050 IMU. This node sends telemetry through an encrypted BLE-to-MQTT gateway, providing a resilient audit trail for physical access events.

### Hardware Specification
* **Edge Node:** Nordic Semiconductor nRF52840-DK (Cortex-M4F)
* **Gateway:** Raspberry Pi Zero (Nano) running a hardened Linux stack
* **Connectivity:** Bluetooth Low Energy (BLE) 5.0 with Coded PHY support for extended range

1. The LIMA Node (The "Edge")
• Hardware: nRF52840 + MPU6050 + Barometric Sensor.
• Software: Zephyr RTOS + CryptoCell-310.

• Function: Detects "Integrity Events" (Door open, Case tamper) and signs the alert.

2. The LIMA Gateway (The "Bridge")
◇ Hardware: Raspberry Pi or an nRF7002 (Wi-Fi) development kit.
◇ Software: MQTT Broker + Python/Node.js Logic.

◇ Function: Translates the local BLE/Thread signal into a phone notification (via Pushbullet or Pushover).


-----

folder layout:
```
├── src/
│   ├── firmware/       # nRF52840 C++/Arduino code
│   └── gateway/        # Python bridge for RPi
├── docs/               # The "Senior Engineer" stuff
│   ├── architecture/   # Diagrams and Schematics
│   └── analysis/       # Power and Threat models
├── tests/              # Validation scripts
├── LICENSE
└── README.md
```

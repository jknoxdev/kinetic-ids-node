# lima-node
L.I.M.A. > (Local Integrity Multi-modal Architecture)
The lima-node is a low-power kinetic / physical intrusion detection system. Utilizing a Zephyr RTOS micro-kernel on the nRF52840, the system implements hardware-accelerated motion discrimination via an MPU6050 IMU. This node sends telemetry through an encrypted BLE-to-MQTT gateway, providing a resilient audit trail for physical access events.

### Hardware Specification
* **Edge Node:** Nordic Semiconductor nRF52840-DK (Cortex-M4F)
* **Gateway:** Raspberry Pi Zero (Nano) running a hardened Linux stack
* **Connectivity:** Bluetooth Low Energy (BLE) 5.0 with Coded PHY support for extended range


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

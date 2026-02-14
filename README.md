# physical-ids-telemetry

Low-power Physical Intrusion Detection System (PIDS) utilizing nRF52840 (BLE) and Raspberry Pi (MQTT) for secure OT environmental monitoring


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

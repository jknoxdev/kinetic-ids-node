# lima-node
L.I.M.A. > (Local Integrity Multi-modal Architecture)
The lima-node is a low-power kinetic / physical intrusion detection system. Utilizing a Zephyr RTOS micro-kernel on the nRF52840, the system implements hardware-accelerated motion discrimination via an MPU6050 IMU. This node sends telemetry through an encrypted BLE-to-MQTT gateway, providing a resilient audit trail for physical access events.

### Hardware Specification
* **Edge Node:** Nordic Semiconductor nRF52840-DK (Cortex-M4F)
* **Gateway:** Raspberry Pi Zero (Nano) running a hardened Linux stack
* **Connectivity:** Bluetooth Low Energy (BLE) 5.0 with Coded PHY support for extended range

## System Overview
1. The LIMA Node (The "Edge")
* Hardware: nRF52840 + MPU6050 + Barometric Sensor.
* Software: Zephyr RTOS + CryptoCell-310.

◇ Function: Detects "Integrity Events" (Door open, Case tamper) and signs the alert.

2. The LIMA Gateway (The "Bridge")
* Hardware: Raspberry Pi or an nRF7002 (Wi-Fi) development kit.
* Software: MQTT Broker + Python/Node.js Logic.

◇ Function: Translates the local BLE/Thread signal into a phone notification (via Pushbullet or Pushover).

## Bringing Up the L.I.M.A. Node
This project uses a Local Manifest topology. Follow these steps to initialize the workspace and install the necessary dependencies for the nRF52840.

1. Prerequisite: Python Environment
We recommend using a virtual environment to avoid dependency drift.
Bash

### Create and activate a clean environment

```bash
python3 -m venv .venv
source .venv/bin/activate
```


### Install West (Zephyr's meta-tool)

```bash

pip install west
```



2. Initialize the Workspace
The west.yml in this repository acts as the master blueprint for the entire SDK.
Bash

### Initialize the workspace using this repo as the local manifest

```bash

west init -l lima-node
```


### Pull the Nordic Connect SDK (NCS), Zephyr, and HAL modules

```bash

west update
```

3. Install SDK Requirements
Once the modules are downloaded, install the specific toolchain requirements:
Bash


```bash

pip install -r zephyr/scripts/requirements.txt
pip install -r nrf/scripts/requirements.txt
pip install -r bootloader/mcuboot/scripts/requirements.txt
```



4. Build & Verify
Test the toolchain by building the firmware for the nRF52840 MDK Dongle:
Bash


```bash

west build -b nrf52840_mdk lima-node/firmware
```



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

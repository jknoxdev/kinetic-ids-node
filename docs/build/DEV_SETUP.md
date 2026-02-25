# Developer Environment Setup

> **Tested against:** NCS v3.2.0-rc1 · Zephyr 4.3.99  
> **Board:** `nrf52840_mdk_usb_dongle/nrf52840`

---

## 1. Install System Dependencies

### Python 3.10+

<details>
<summary>🐧 Linux (Ubuntu / Debian)</summary>

```bash
sudo apt-get update
sudo apt-get install -y python3 python3-pip python3-venv git cmake ninja-build
```
</details>

<details>
<summary>🍎 macOS</summary>

```bash
brew install python3 cmake ninja git
```
</details>

<details>
<summary>🪟 Windows</summary>

Install via [python.org](https://www.python.org/downloads/) — check **"Add to PATH"** during install.  
Install [Git for Windows](https://git-scm.com/download/win) and [CMake](https://cmake.org/download/).  
Install [Ninja](https://ninja-build.org/) and add to PATH.
</details>

---

## 2. Install NCS Toolchain Manager

Nordic bundles the ARM compiler with NCS — no separate ARM GCC install needed. The toolchain is versioned alongside the SDK for reproducible builds.

<details>
<summary>🐧 Linux / 🍎 macOS</summary>

```bash
# Toolchain is pulled automatically by west update in Step 4
# No manual ARM GCC install required
```

Alternatively install via [nRF Connect for Desktop](https://www.nordicsemi.com/Products/Development-tools/nRF-Connect-for-Desktop) → Toolchain Manager.
</details>

<details>
<summary>🪟 Windows</summary>

Install [nRF Connect for Desktop](https://www.nordicsemi.com/Products/Development-tools/nRF-Connect-for-Desktop) → open **Toolchain Manager** → install the NCS version matching `west.yml`.  
This installs ARM GCC, CMake, Ninja, and all build tools automatically.
</details>

> ⚠️ Do NOT install standalone `gcc-arm-none-eabi` separately — it can conflict with the NCS bundled toolchain.

---

## 3. Install West + Python Virtual Environment

<details>
<summary>🐧 Linux / 🍎 macOS</summary>

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install west
```
</details>

<details>
<summary>🪟 Windows (PowerShell)</summary>

```powershell
python -m venv .venv
.venv\Scripts\Activate.ps1
pip install west
```
</details>

> ⚠️ Always activate the venv before any `west` commands. If west commands fail with import errors — you forgot this step.

---

## 4. Initialize the NCS Workspace

```bash
# All platforms — run from your projects directory
west init -l lima-node
west update
```

This pulls Zephyr, Nordic NCS, MCUboot, and all HAL modules. Takes 5-15 minutes on first run.

---

## 5. Install SDK Python Requirements

```bash
# All platforms — venv must be active
pip install -r zephyr/scripts/requirements.txt
pip install -r nrf/scripts/requirements.txt
pip install -r bootloader/mcuboot/scripts/requirements.txt
```

---

## 6. Install UF2 Flashing Tools

<details>
<summary>🐧 Linux</summary>

```bash
sudo apt-get install -y python3-serial
```

Also install `screen` or `minicom` for serial monitoring:
```bash
sudo apt-get install -y screen minicom
```

Add yourself to the `dialout` group for serial port access:
```bash
sudo usermod -aG dialout $USER
# Log out and back in for this to take effect
```
</details>

<details>
<summary>🍎 macOS</summary>

```bash
pip install pyserial
brew install minicom
```

Serial port will appear as `/dev/tty.usbmodemXXXX` — check with:
```bash
ls /dev/tty.usb*
```
</details>

<details>
<summary>🪟 Windows</summary>

Install [PuTTY](https://www.putty.org/) for serial monitoring.  
Dongle serial port appears as `COMx` in Device Manager.  
UF2 flashing is drag-and-drop to the `UF2BOOT` drive — no extra tools needed.
</details>

---

## 7. Recommended IDE Setup

### VS Code (recommended)

Install extensions:

| Extension | Purpose |
|---|---|
| `nordic-semiconductor.nrf-connect` | nRF Connect for VS Code — build, flash, debug |
| `ms-vscode.cmake-tools` | CMake integration |
| `ms-vscode.cpptools` | C/C++ intellisense |
| `mhutchie.git-graph` | Git history visualization |

<details>
<summary>🐧 Linux / 🍎 macOS</summary>

```bash
code --install-extension nordic-semiconductor.nrf-connect
code --install-extension ms-vscode.cmake-tools
code --install-extension ms-vscode.cpptools
```
</details>

<details>
<summary>🪟 Windows</summary>

Install extensions via VS Code Extensions panel (`Ctrl+Shift+X`) — search each extension name above.
</details>

---

## 8. Verify Setup

Build the blinky sample to confirm everything is working:

<details>
<summary>🐧 Linux / 🍎 macOS</summary>

```bash
source .venv/bin/activate
west build -b nrf52840_mdk_usb_dongle/nrf52840 zephyr/samples/basic/blinky \
  -- -DCONFIG_BUILD_OUTPUT_UF2=y
```
</details>

<details>
<summary>🪟 Windows (PowerShell)</summary>

```powershell
.venv\Scripts\Activate.ps1
west build -b nrf52840_mdk_usb_dongle/nrf52840 zephyr/samples/basic/blinky `
  -- -DCONFIG_BUILD_OUTPUT_UF2=y
```
</details>

Expected output — memory report + `merged.hex` generated:
```
Memory region    Used Size   Region Size   % Used
FLASH:           53180 B     1020 KB       5.09%
RAM:             15544 B     256 KB        5.93%
```

If this builds cleanly — you're ready. See [`FLASHING.md`](FLASHING.md) to get it onto the board.

---

## Troubleshooting

| Error | Fix |
|---|---|
| `west: command not found` | venv not activated — `source .venv/bin/activate` |
| `CMake Error: CMAKE_C_COMPILER not found` | ARM toolchain not installed or not in PATH |
| `west update` fails mid-pull | Network issue — re-run `west update`, it resumes |
| `Permission denied /dev/ttyACM0` | Not in `dialout` group — see Step 6 Linux |
| `Import error: No module named ...` | Run SDK pip requirements again (Step 5) |
| VS Code can't find Zephyr SDK | Point nRF Connect extension to your workspace root |

---

*Next step: [FLASHING.md](FLASHING.md)*
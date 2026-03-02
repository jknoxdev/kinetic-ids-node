# Security Policy

## Overview

LIMA (Local Integrity Multi-modal Architecture) is an open-source Physical Intrusion Detection System. Security is central to the project's mission. Responsible disclosure of vulnerabilities is welcomed and appreciated.

---

## Supported Versions

| Component | Status |
|---|---|
| nRF52840 Firmware | Active development — reports accepted |
| Raspberry Pi Gateway | Active development — reports accepted |
| Documentation / Architecture | Active development — reports accepted |

> Note: This project is currently in early development. There are no stable releases yet. All components should be considered pre-release.

---

## Scope

The following are **in scope** for security reports:

- **Firmware** — nRF52840 Zephyr RTOS code, CryptoCell-310 usage, cryptographic implementation issues
- **BLE communication** — pairing, encryption, replay attacks, spoofing, interception of telemetry
- **MQTT gateway** — authentication, authorization, broker hardening, message integrity
- **Physical security assumptions** — flaws in the threat model or detection logic that could allow an attacker to defeat PIDS detection
- **Dependency vulnerabilities** — critical CVEs in Zephyr RTOS, Nordic Connect SDK, or other dependencies that directly affect LIMA

The following are **out of scope**:

- Vulnerabilities in third-party services (Pushbullet, Pushover, etc.) not specific to LIMA's integration
- Physical attacks requiring direct hardware access beyond the stated threat model (LIMA assumes physical access by an adversary is the *event being detected*, not a vulnerability)
- Generic Zephyr RTOS or Nordic SDK vulnerabilities not specific to LIMA's implementation

---

## Reporting a Vulnerability

Please **do not** open a public GitHub issue for security vulnerabilities.

Report vulnerabilities via email:

**security@nullsec.systems**  
*(PGP key available on request)*

Please include in your report:

- A clear description of the vulnerability
- The affected component (firmware, gateway, BLE stack, etc.)
- Steps to reproduce or a proof-of-concept if available
- Your assessment of severity and potential impact
- Any suggested mitigations if you have them

---

## Response Commitments

| Milestone | Target |
|---|---|
| Acknowledgement | Within 72 hours |
| Initial triage and severity assessment | Within 7 days |
| Critical/High severity fix or mitigation | Within 30 days |
| Medium/Low severity fix | Next scheduled release |
| Public disclosure coordination | Mutually agreed upon timeline |

---

## Disclosure Policy

LIMA follows a **coordinated disclosure** model:

1. Reporter submits vulnerability privately
2. Maintainer acknowledges and triages
3. Fix is developed and tested
4. Reporter is given opportunity to review the fix
5. Public disclosure occurs after fix is available, or after a maximum of **90 days** from initial report (whichever comes first)

If a vulnerability is actively being exploited in the wild, the timeline may be accelerated with reporter notification.

---

## Recognition

Researchers who responsibly disclose valid vulnerabilities will be acknowledged in the project's release notes and CREDITS file (unless they prefer to remain anonymous).

---

## Contact

For non-security questions, please use [GitHub Issues](https://github.com/jknoxdev/lima-node/issues).

For security issues: **security@nullsec.systems**

---

*This security policy will be updated as the project matures toward stable releases.*

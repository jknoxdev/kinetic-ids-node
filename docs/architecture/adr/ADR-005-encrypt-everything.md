# ADR-005: Payload Security — AES-256-GCM Encryption + ECDSA-P256 Signing

**Status:** Active  
**Date:** 2026-02-25  
**Author:** jknoxdev

---

## Context

LIMA node BLE advertisements carry integrity event payloads from sensor to gateway. The question was whether to sign only, or sign AND encrypt, and whether encryption created legal exposure as an open source project.

Two concerns were evaluated:

1. **Engineering** — is encryption worth the overhead on a battery-powered embedded device?
2. **Legal** — does shipping AES-256 in a public open source repo create export control liability?

---

## Decision

**Encrypt and sign everything. AES-256-GCM + ECDSA-P256 on every payload.**

Both operations are hardware accelerated on the CryptoCell-310. No meaningful power or performance penalty over signing alone.

---

## Legal Research — Export Controls

Concern: US export control law (EAR) has historically restricted strong encryption. This was a real problem in the 1990s — Phil Zimmermann's PGP distribution triggered a federal investigation, and the Kerberos and GPG ecosystems faced serious restrictions.

Current state as of 2026:

- The **Bernstein v. Department of State** case (EFF, 1995-2003) established that publishing encryption source code is protected speech
- **As of 2021**, BIS eliminated email notification requirements for open source projects using standard cryptography — AES-256, ChaCha20-Poly1305, ECDSA all qualify as standard
- Public open source repos on GitHub are covered under Microsoft/GitHub's platform-level BIS notification — no individual action required
- There is **no "unexportable" level of standard encryption** for public open source projects

**Bottom line:** The 90s crypto wars are over. Shipping AES-256 in a public GitHub repo requires zero legal action. The concern was valid historically but is not applicable to LIMA.

---

## Engineering Decision

The CryptoCell-310 supports:
- AES-256 (GCM, CCM, CBC, CTR)
- ChaCha20-Poly1305
- ECDSA P-256 / P-384
- SHA-256 / SHA-512

All hardware accelerated. The payload already passes through CryptoCell-310 for ECDSA signing — AES-256-GCM encryption is a free rider on hardware that's already engaged.

**Selected: AES-256-GCM** over ChaCha20-Poly1305 because:
- GCM provides authenticated encryption (AEAD) — encryption and MAC in one pass
- AES-256-GCM is more universally recognized in SIEM contexts
- Hardware acceleration eliminates ChaCha20's primary advantage (software performance on cores without AES-NI)

---

## Security Posture

- **Signing** — proves authenticity, detects tampering
- **Encryption** — hides payload contents, prevents traffic analysis
- **Per-event nonce** — prevents replay attacks
- **Default on** — encrypt everything, no configuration required to enable

"Encrypt everything by default, make it optional to disable" is a stronger posture than "encrypt optionally." Defense in depth.

---

## Consequences

- **Good:** Full AEAD protection on every payload, hardware accelerated, no battery impact
- **Good:** No legal exposure — standard crypto, public open source, GitHub platform covered
- **Good:** Stronger security story for deployments where payload confidentiality matters
- **Neutral:** Key management strategy TBD — provisioning node keys at flash time is a v1.1 concern
- **Neutral:** Gateway must hold decryption keys — key storage on RPi Zero to be documented

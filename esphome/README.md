# Color Shadow Lamp — ESPHome Firmware

This directory contains the ESPHome-based firmware for the Color Shadow Lamp. It preserves the manual RGB mode of the original firmare (pots for RGB, button press for on off, and safe‑mode power limits) while adding a first‑class Home Assistant light entity with proper color and brightness control and OTA updates.  There is no need to switch modes, it's always in ESPHome mode and manual control will always work.

## Overview

- Exposes the lamp as a standard RGB light in Home Assistant (HA).
- Retains manual control using the three potentiometers (pots) and the button.
- “Safe Mode” limits overall power exactly as the original firmware did and is toggleable from HA.
- Captive portal onboarding: if no Wi‑Fi is configured, the device brings up an AP so you can provision credentials without reflashing.
- 11‑bit high‑frequency PWM with the original color trims and the hardware red/blue channel swap accounted for.

Key hardware mappings (handled by firmware — no action needed):
- PWM pins: R=GPIO5, G=GPIO6, B=GPIO7 (with red/blue channels swapped internally to match the board wiring).
- Pots: left/middle/right to ADC inputs, averaged and noise‑filtered for stable control.
- Button: GPIO9 (inverted). Short press toggles the light, long press (>15s) and release factory resets.

## Behaviour and Parity with the Original Firmware

- RGB trims: Red 0.95, Green 1.00, Blue 0.40.
- PWM resolution/frequency: 11‑bit at ~19kHz.
- Safe Mode power caps: Locked 30%, Unlocked 60%. This matches the original “safe/unlocked” behaviour. Safe Mode defaults to ON on first boot and after a factory reset.
- Manual pots control the three channels directly when the lamp is ON. Pots are moving‑averaged; small noise does not cause flicker.
- OFF latch: When the lamp is turned OFF (via HA or a short button press), all LEDs are driven to zero and pot changes will NOT turn it back on. Turn ON again via HA or a short button press; the next pot update will apply once ON.

## What You’ll See in Home Assistant

The device is auto‑discovered by the ESPHome integration and creates:

- Light entity `Color Shadow Lamp`
  - Standard HA color wheel/selector
  - Brightness slider (scales LED output brightness correctly)
- Switch `Color Shadow Safe Mode`
  - ON = 30% power cap (safe)
  - OFF = 60% cap (unlocked)

Brightness, color, and on/off from HA are respected even while the pots are connected. If both HA and pots are used, the most recent action wins while the lamp is ON.

## Manual Controls

- Button (GPIO9):
  - Short press: Toggle light ON/OFF. OFF fully cuts LED output; pots won’t turn the lamp back on by themselves.
  - Long press (~15 seconds): Factory reset (see below).
- Pots (left/middle/right):
  - Map to Red/Green/Blue respectively.
  - Smoothed with a moving average; small jitter won’t cause flicker.
  - Take effect when the lamp is ON. While OFF, pot moves are buffered and will apply after you turn it ON again.

## Factory Reset (15‑second hold)

- Press and hold the button for >15 seconds and then release.
- The lamp flashes three quick times to confirm the reset was accepted.
- Safe Mode is restored to ON.
- All stored preferences are cleared, including Wi‑Fi credentials.
- The device reboots into AP mode so you can provision Wi‑Fi again.

## Setup and Provisioning

First flash (USB):
1. Install ESPHome CLI (Python 3.11+): `pip install esphome`.
2. Put the lamp into download mode (hold BOOT, tap RESET, release RESET, then release BOOT), plug into USB.
3. From this repo root:
   - `esphome run esphome/color_shadow.yaml --device /dev/cu.usbmodemXXXX`

Wi‑Fi onboarding:
1. After first flash or a factory reset, connect to `Color Shadow Setup`.
2. Open `http://192.168.4.1` and enter your home Wi‑Fi SSID/password.
3. The lamp reboots and joins your network; HA should discover it via ESPHome.

OTA updates:
- Once the lamp is on your network, you can update firmware over the air:
  - `esphome upload esphome/color_shadow.yaml`

## Expected Entities and Controls in HA

- `light.color_shadow_lamp` — Change color and brightness, turn on/off.
- `switch.color_shadow_safe_mode` — Toggle safe vs. unlocked power cap (30% vs 60%). The selection is persisted across reboots.

Notes:
- The brightness slider in HA scales overall output correctly (it’s combined with the internal safe‑mode cap and color trims).
- Transitions use HA’s defaults; you can change the transition length or gamma in the YAML if desired.

## Files in This Folder

- `color_shadow.yaml` — ESPHome configuration (entities, Wi‑Fi AP onboarding, long‑press reset, safe mode switch).
- `components/color_shadow_light/` — Custom light driver:
  - 11‑bit LEDC PWM at ~19kHz
  - Original color trims and red/blue swap
  - Safe/Unlocked power caps with persistence
  - Pot smoothing and OFF latch behaviour


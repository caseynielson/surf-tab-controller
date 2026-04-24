# Surf Tab Controller

Custom surf tab controller for **2007 Malibu Wakesetter 247**.
Replaces the Bennett Marine OBI9000-E Bolt.

## Hardware

| Component | Part | Notes |
|---|---|---|
| Actuators | Lenco 15054-001 | 2-wire, 12V, no built-in feedback |
| Motor drivers | IBT-2 (BTS7960 43A H-bridge) | 1 per actuator |
| Controller MCU | ESP32-WROOM-32 dev board | Motor control + logic |
| UI display | Elegoo CYD (ESP32-2432S028R) | 2.8" touch LCD, via UART |
| Final UI | Waveshare ESP32-S3 touch LCD | Planned boat install |

## Architecture

```
Boat 12V ──┬── IBT-2 (port)  ── Lenco port actuator
           └── IBT-2 (stbd)  ── Lenco stbd actuator
                ↑
           ESP32 controller ──── UART2 ──── CYD UI display
```

Position tracking is **time-based** (ms from home). No external sensors.
Home position (fully retracted) is detected by motor stall current via IBT-2 IS pins.

## Pin Assignments (ESP32 controller)

### Port IBT-2
| Signal | GPIO | Notes |
|--------|------|-------|
| RPWM (extend) | 25 | PWM output |
| LPWM (retract) | 26 | PWM output |
| R_EN | 27 | Forward enable |
| L_EN | 14 | Reverse enable |
| R_IS | 34 | Current sense ADC (input only) |
| L_IS | 35 | Current sense ADC (input only) |

### Starboard IBT-2
| Signal | GPIO | Notes |
|--------|------|-------|
| RPWM (extend) | 32 | PWM output |
| LPWM (retract) | 33 | PWM output |
| R_EN | 12 | Forward enable |
| L_EN | 13 | Reverse enable |
| R_IS | 36 | Current sense ADC (input only) |
| L_IS | 39 | Current sense ADC (input only) |

### UART to CYD UI
| Signal | GPIO |
|--------|------|
| TX (to CYD RX) | 17 |
| RX (from CYD TX) | 16 |

## IBT-2 Current Sense Wiring

The IBT-2 R_IS and L_IS pins are open-drain current sources. Add a **680Ω resistor** from each IS pin to GND to produce a readable voltage:

```
IS pin ──── 680Ω ──── GND
              │
           ESP32 ADC
```

This is **required** for stall detection to work.

## Presets

| Preset | Port | Stbd |
|--------|------|------|
| NEUTRAL | 0% | 0% |
| SURF LEFT | 80% | 20% |
| SURF RIGHT | 20% | 80% |
| FULL DOWN | 95% | 95% |

Percentages are of `ACTUATOR_FULL_TRAVEL_MS` defined in `config.h`.
**Calibrate `ACTUATOR_FULL_TRAVEL_MS` on the bench before installing on the boat.**

## Serial Commands (USB console + UI)

| Command | Action |
|---------|--------|
| `N` / `NEUTRAL` | Go to neutral (tabs up) |
| `L` / `SURF_LEFT` | Surf left preset |
| `R` / `SURF_RIGHT` | Surf right preset |
| `D` / `FULL_DOWN` | Full down |
| `STOP` | Immediate stop |
| `HOME` | Re-home (retract to stall) |
| `STATUS` | Print current positions |
| `HELP` | List commands |

## UI Protocol (UART, 115200 baud)

Controller → UI:
```
READY
ERROR:<reason>
PRESET:<idx>:<name>
STATUS:<portMs>:<stbdMs>:<portState>:<stbdState>
```

UI → Controller:
```
N | L | R | D | STOP | HOME
```

## Setup & Calibration

1. Wire IBT-2 modules per pin table, add 680Ω IS resistors
2. Connect actuators to IBT-2 B+/B- terminals
3. Apply 12V to IBT-2 power terminals
4. Upload sketch — board will home on boot
5. Open serial monitor (115200), type `STATUS` to verify
6. Time full travel: `D` then watch serial output for extend time
7. Update `ACTUATOR_FULL_TRAVEL_MS` in `config.h`
8. Adjust preset percentages in `config.h` to taste
9. Tune `STALL_THRESHOLD` — raise if false stalls, lower if stall isn't detected

## References
- [Wakegarage ESP32 surf controller thread](https://www.wakegarage.com/forums/topic/1147-esp32-surf-controller-automanual-control-lcd-screen-pitchroll-wifi-mosfet-motor-controllers/)
- [IBT-2 / BTS7960 datasheet](https://www.infineon.com/dgdl/bts7960b-pb-final.pdf)
- [CYD repo](https://github.com/caseynielson/esp_28_lcd_exp)

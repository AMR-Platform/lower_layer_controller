# ATmega32U4 controller codebase

# AVR Lower-Layer Controller for AMR Robot

**Author:** Kiran Gunathilaka  ·  **Target MCU:** ATmega32U4 @ 16 MHz  ·  **Firmware language:** C (AVR-GCC)

> This firmware delivers real-time motion control, sensing, and safety for the autonomous mobile robot (AMR). **It runs solely on the ATmega32U4 board**—all Jetson-side logic lives elsewhere.

---

## ✨ Feature Highlights

| Area | Key Points |
|------|------------|
| Motion Control | Dual NEMA-24 stepper drivers (STEP/DIR), trapezoidal profile generator, 50 Hz update loop |
| Odometry | Mixed INT + PCINT quadrature decoding (32-bit counters), forward & yaw estimation |
| IMU | BNO055 low-level driver, health watchdog with soft/hard reset, Euler + gyro + accel feed |
| ADC | Dual battery rails + three cliff sensors, software averaging, mV conversion |
| Timing | 64-bit, 1 µs global clock (Timer-0), dedicated timers for both motors + main loop |
| Comms | USB-CDC, binary command & telemetry packets, CRC-16, ≈ 50 Hz upstream |
| Safety | 200 ms command-loss stop, emergency GPIO, IMU watchdog, brown-out fuse ready |

---

## 🧩 Pin Mapping Overview (from `config.h`)

| Function | Arduino Label | AVR Pin | Notes |
|----------|---------------|---------|-------|
| **LEFT PUL** | D5 | PC6 / OC3A | Timer-3 toggle |
| **LEFT DIR** | D12 | PD6 | Forward / Reverse |
| **LEFT ENA** | D10 | PB6 | Active-HIGH (inverted logic) |
| **RIGHT PUL** | D9 | PB5 / OC1A | Timer-1 toggle |
| **RIGHT DIR** | D8 | PB4 | Forward / Reverse |
| **RIGHT ENA** | D6 | PD7 | Active-HIGH |
| **ENC L A** | D1 | PD2 / INT2 |
| **ENC L B** | D0 | PD3 / INT3 |
| **ENC R B** | D7 | PE6 / INT6 |
| **ENC R A** | D8 | PB4 / PCINT4 |
| **EMERGENCY** | D11 | PB7 / PCINT7 | Active-LOW |
| **BNO055 RST** | — | PF7 | Pull HIGH for reset |
| **ADC** | PF0, PF1, PF4-6 | — | VBAT & cliff sensors |

---

## 🕒 Timer & Peripheral Allocation

| Timer | Mode / Prescale | Purpose | Source File |
|-------|-----------------|---------|-------------|
| **Timer-0 (8-bit)** | Free-running, ÷8 → 0.5 µs/tick | 64-bit microsecond clock | `systime.c` |
| **Timer-1 (16-bit)** | CTC, OC1A toggle, ÷1024 | RIGHT motor STEP (PB5) | `motors.c` |
| **Timer-3 (16-bit)** | CTC, OC3A toggle, ÷1024 | LEFT motor STEP (PC6) | `motors.c` |
| **Timer-4 (10-bit enhanced)** | CTC, OCR4A = 156, ÷1024 | 100 Hz main loop interrupt | `main.c` |
| **TWI (I²C)** | 100 kHz | BNO055 IMU | `bno055_ll.c` |

---

## 🎮 Control & Debug Modes

### Control Modes (`controlMode` enum)

| Value | Meaning |
|-------|---------|
| `0` – `AUTONOMOUS` | MCU executes pre-planned trapezoidal profiles sent from host |
| `1` – `TELEOPERATOR` | Host sends discrete f/b/l/r bits; MCU handles acceleration & braking |

### Debug Modes (`debugMode` enum)

| Value | Description |
|-------|-------------|
| `0` – `DEBUG_OFF` | No extra serial output |
| `1` – `MOTION_DEBUG` | Streams wheel speeds, odom, etc. |
| `2` – `RX_ECHO` | Echo raw command packets |
| `3` – `MD_AND_ECHO` | Combine 1 + 2 |

---

## 🔄 Tele-op Mealy State Machine

The tele-operator logic is a compact Mealy machine with **four states** and **four primary transitions**:

1. **`NONETELEOP`** → no active command  
2. **`ACCELERATING`** → ramp up to target speed/profile  
3. **`CONSTANT`** → cruise at steady velocity  
4. **`DECELERATING`** → brake to a smooth stop  

![State Machine](images/state.jpg)

Each new key-press combination (`f b l r`) triggers a transition. The handler decides whether to regenerate a fresh profile or decelerate gently before accepting the next command—keeping ride smooth and preventing stepper stalls.

---

## 📂 Module-by-Module Breakdown

### `config.h`
Defines clocks, prescalers, **all pin macros**, wheel geometry, and motion constants. Central point for hardware adaptation—no magic numbers outside this file.

### `main.c`
Initialises every subsystem, arms **Timer-4** for 100 Hz scheduling, parses USB packets, executes the Mealy machine, and pushes telemetry. Keeps application logic separate from drivers.

### `motors.c`
Pure-C driver for two stepper modules:
* Direction bits + enable gating  
* Speed-to-frequency helper converts mm/s → timer TOP  
* Independent timers let wheels spin at arbitrary RPMs while MCU remains free.

### `encoder.c`
Robust 4-channel quadrature decoder:
* INT2/INT3 (left) + INT6/PCINT4 (right)  
* Emergency button multiplexed on PCINT0 ISR  
* Computes Δticks, mm/s, deg/s and integrates distance/heading.

### `profiler.c`
Motion planner providing dual **Profile** instances (linear + angular). Implements a trapezoidal/triangular ramp with automatic braking-distance check. Feeds velocity/omega to `motors_update()`.

### `systime.c`
Implements 64-bit microsecond counter via Timer-0 overflow; utility `micros64()` is used everywhere timing matters.

### `analog.c`
ADC multiplexer, 4-sample software average, milli-volt conversion using configurable divider ratios.

### `bno055_ll.c`
Bare-metal I²C driver plus convenience wrappers; watchdog function initiates graceful soft reset and GPIO hard reset if the sensor becomes unresponsive for > 300 ms.

### `usb_serial.c`
Minimal CDC-ACM wrapper; DMA-less TX buffer ensures predictable latency at 115 200 Bd.

---

## 🗨️ Serial Protocol (recap)

* **Commands:** structured binary with CRC-16 (see `serial_format.h`).  
* **Telemetry:** fixed-width ASCII line for easy logging/plotting & optional debug fields.

---

## 🛡️ Safety

* **Command-loss watchdog:** disables steppers after 200 ms silence.  
* **Emergency button:** hardware-level pull-down on PB7 triggers immediate `motors_stop_all()`.  
* **IMU Watchdog:** toggles between soft and hard reset based on failure duration.  

---

## 🛠️ Customisation & Porting

Change only `include/config.h` when migrating to a new chassis:  
* Update pin macros, wheel diameter/base, encoder PPR & gear ratio.  
* Tweak trapezoid limits (`TELEOP_SPEED`, `TELEOP_ACC`, etc.).  

No other file should require edits—**one-file re-target** by design.

---

## 🧱 Modularity & Code Quality

* **Single-responsibility drivers**—each peripheral lives in its own `.c`/`.h` pair.  
* Interrupt isolation: ISRs are kept minimal; heavy math runs in the main thread.  
* All maths use `float`, but critical timing uses integer TOP calculations to keep ISR jitter low.  
* Consistent naming conventions and exhaustive `static inline` helpers promote readability.  

---

## 📜 License & Support

Released under the **MIT License**.  
For questions, open an issue or email **kiran.gunathilaka&nbsp;[at]&nbsp;example.com**.


## upload command (Using avrdude)

All the locations are respect to my machine. change accordingly

C:\Users\META\AppData\Local\Arduino15\packages\arduino\tools\avrdude\6.3.0-arduino17/bin/avrdude.exe -C"C:\Users\META\AppData\Local\Arduino15\packages\arduino\tools\avrdude\6.3.0-arduino17/etc/avrdude.conf" -v -V -patmega32u4 -cavr109 "-PCOM5" -b57600 -D -Uflash:w:"D:\Downloads\AMR\GithubOrg\lower_layer_controller\avr_controller\Debug\avr_controller.hex":i
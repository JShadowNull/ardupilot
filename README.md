# Dark-Pilot — ArduPilot fork for the Thunder Tiger H743

A customized fork of **ArduPilot Copter 4.6.3** targeting the **Thunder Tiger**
flight controller (OEM hardware: Tritium / Ewing Aerospace "BraveH7", board ID
`AP_HW_TRITIUM_EAH743`). It adds a dedicated board target, an RC-dimmable PWM
light driver, OSD relay panels, VTX power/pit-mode handling, and a
"loiter-or-land" failsafe action.

> This is a downstream fork. The full upstream project, documentation, and
> community live at **[ardupilot.org](https://ardupilot.org)** and
> **[github.com/ArduPilot/ardupilot](https://github.com/ArduPilot/ardupilot)**.
> Everything here is GPLv3, the same as upstream.

---

## Hardware

| Block      | Part                              | Bus / Pins              |
|------------|-----------------------------------|-------------------------|
| MCU        | STM32H743 @ 8 MHz ext. oscillator | —                       |
| IMU        | BMI270 (alt: ICM42688)            | SPI1, CS `PA4`          |
| OSD        | MAX7456 (analog)                  | SPI2, CS `PB12`         |
| Dataflash  | W25Q128                           | SPI3, CS `PA15`         |
| Baro       | BMP390 (BMP388 driver)            | I2C1 @ `0x76`           |
| Power      | Voltage / current sense           | ADC1 `PC0` / `PC1`      |
| Outputs    | 6× ESC                            | TIM1 / TIM3 / TIM15     |
| Light      | PWM LED driver (dimmer input)     | `PA2` / TIM2_CH3 (PWM7) |

App starts at `0x08020000` (128 KB bootloader reserve). Parameter storage uses
the last two 128 KB flash sectors (`STORAGE_FLASH_PAGE 14`).

### Board targets

- **`ThunderTiger7in`** — 7-inch build
- **`ThunderTiger15in`** — 15-inch build (same FC, different default/tuning params)

Both share identical hardware; they differ only in their bundled
`defaults.parm` (tuning, failsafe, frame setup).

---

## What this fork adds on top of ArduPilot 4.6.3

### Board support
- `ThunderTiger7in` and `ThunderTiger15in` hwdefs, board ID `AP_HW_ThunderTigerH743` (11065).
- Per-board `defaults.parm` (CH9 = light control, CH10 = VTX power, IRC Tramp VTX on SERIAL3 half-duplex, battery setup, failsafe).

### `AP_PWMLight` — RC-dimmable light driver (`libraries/AP_PWMLight/`)
- Drives a single PWM output (`PA2` / TIM2_CH3) at an exact duty cycle for an external LED driver.
- Brightness mapped from an RC input channel (`LIGHT_RCIN`, default CH9).
- Parameters: `LIGHT_ENABLE`, `LIGHT_OUT`, `LIGHT_FREQ`, `LIGHT_RCIN`.

### OSD relay panels (`libraries/AP_OSD/`)
- Adds `OSDn_RELAY1..6` display elements with selectable preset labels, so relay/switch states can be shown on the analog OSD.

### VTX improvements (`libraries/AP_VideoTX/`)
- Adds 2 W / 4 W power levels (range extended to 4000 mW).
- `VTX_OPTIONS` bit to force **pit mode on RC failsafe** (kills video TX power when the link is lost).

### "Loiter-or-Land" failsafe (`ArduCopter/`)
- New failsafe action (option **8**) selectable on `FS_THR_ENABLE`, `FS_GCS_ENABLE`, and the battery monitor: loiter if a position estimate is available, otherwise land.
- VTX is put into pit mode on RC failsafe.

---

## Building

Requires the ArduPilot toolchain (see
[upstream build setup](https://ardupilot.org/dev/docs/building-setup-linux.html)).

```bash
# main firmware
./waf configure --board ThunderTiger7in     # or ThunderTiger15in
./waf copter
# output: build/ThunderTiger7in/bin/arducopter.apj

# bootloader
./waf configure --board ThunderTiger7in --bootloader
./waf bootloader
# output: build/ThunderTiger7in/bin/AP_Bootloader.bin
```

## Flashing

The board ships with its OEM bootloader. Normal updates go through it via
Mission Planner / `uploader.py` using the `.apj` file.

**Recovery / direct flash (always available):** the STM32H743 ROM DFU lives in
mask ROM and cannot be erased. Hold the **BOOT** pin while connecting USB to get
`0483:df11`, then:

```bash
# app at 0x08020000 (does NOT touch the bootloader)
dfu-util -a 0 -s 0x08020000 -D build/ThunderTiger7in/bin/arducopter.bin

# bootloader at 0x08000000 (sector 0 only)
dfu-util -a 0 -s 0x08000000 -D build/ThunderTiger7in/bin/AP_Bootloader.bin
```

> ⚠️ **Back up the existing bootloader before overwriting it:**
> `dfu-util -a 0 -s 0x08000000:0x20000 -U oem_bl_backup.bin`
> Because ROM DFU is always reachable via the BOOT pin, a bad flash is
> recoverable — it is not a permanent brick.

---

## Upstream

Forked from [ArduPilot](https://github.com/ArduPilot/ardupilot) at the 4.6.3
release. See [`ardupilot.org`](https://ardupilot.org) for the autopilot
documentation, and the upstream `README` history for full project credits.
ArduPilot is free software under the
[GNU GPLv3](https://www.gnu.org/licenses/gpl-3.0.en.html).

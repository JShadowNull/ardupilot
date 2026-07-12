# Branch notes — `arducopter-4.6.3_vtx3w_v1.1.0`

This is our fork of ArduCopter 4.6.3. It carries a handful of changes on top of
upstream, mostly for the **SkystarsF405v2** FPV board and its analog IRC Tramp
VTX. Read this before you build or flash so nothing here catches you off guard.

Everything below is relative to `master` (upstream 4.6.3). If a param or flag
isn't mentioned here, assume it's stock.

## What this branch is for

Two things, really:

1. A tuned SkystarsF405v2 target that flies as an FPV quad with the sensors we
   don't use turned off.
2. Support for the analog IRC Tramp VTX (the `vtx3w` in the branch name — but the
   actual hardware maxes at **2500 mW / 2.5 W**, not 3 W). We drive both its
   **power** (6-position switch) and **pitmode** from the radio. See
   [VTX power + pitmode](#vtx-power--pitmode-irc-tramp-serial6).

On top of that there are 4G-modem, GPS, and boat-launch build variants (see
below), plus a couple of behaviour tweaks (auto mode, arming, motor spool).

## Key files

Everything this branch touches, in one place (links are clickable).

**Board config** — all under
[`libraries/AP_HAL_ChibiOS/hwdef/SkystarsF405v2/`](libraries/AP_HAL_ChibiOS/hwdef/SkystarsF405v2):

| File | What it is |
|------|-----------|
| [`hwdef.dat`](libraries/AP_HAL_ChibiOS/hwdef/SkystarsF405v2/hwdef.dat) | board definition — pins, sensors, compile-time flags |
| [`defaults.parm`](libraries/AP_HAL_ChibiOS/hwdef/SkystarsF405v2/defaults.parm) | base default params (FPV, no GPS) |
| [`gps.params`](libraries/AP_HAL_ChibiOS/hwdef/SkystarsF405v2/gps.params) | GPS-on-SERIAL5 variant |
| [`4gmodem.params`](libraries/AP_HAL_ChibiOS/hwdef/SkystarsF405v2/4gmodem.params) | 4G modem variant |
| [`boats.params`](libraries/AP_HAL_ChibiOS/hwdef/SkystarsF405v2/boats.params) | boat-launch (gyro) variant — no GPS |
| [`boat_gps.params`](libraries/AP_HAL_ChibiOS/hwdef/SkystarsF405v2/boat_gps.params) | boat + GPS variant |

**Modified source:**

| File | Change |
|------|--------|
| [`AP_VideoTX.cpp`](libraries/AP_VideoTX/AP_VideoTX.cpp) / [`.h`](libraries/AP_VideoTX/AP_VideoTX.h) | VTX power table + power/pitmode switching |
| [`AP_BLHeli.cpp`](libraries/AP_BLHeli/AP_BLHeli.cpp) | ESC passthrough handler (context for `SERVO_BLH_MASK`) |
| [`events.cpp`](ArduCopter/events.cpp), [`Copter.h`](ArduCopter/Copter.h), [`defines.h`](ArduCopter/defines.h), [`Parameters.cpp`](ArduCopter/Parameters.cpp) | Loiter-or-AltHold failsafe (action `8`) |
| [`mode_auto.cpp`](ArduCopter/mode_auto.cpp) | Auto-with-no-mission → Guided |
| [`config.h`](ArduCopter/config.h), [`AP_MotorsMulticopter.h`](libraries/AP_Motors/AP_MotorsMulticopter.h) | faster arm / spool |
| [`AP_StandbyPower.cpp`](libraries/AP_Vehicle/AP_StandbyPower.cpp) / [`.h`](libraries/AP_Vehicle/AP_StandbyPower.h), [`AP_Vehicle.cpp`](libraries/AP_Vehicle/AP_Vehicle.cpp) / [`.h`](libraries/AP_Vehicle/AP_Vehicle.h) | low-power USB standby / RC wake (`STBY_*`) |
| [`minimize_fpv_osd.inc`](libraries/AP_HAL_ChibiOS/hwdef/include/minimize_fpv_osd.inc) | low-flash feature trims the board pulls in |

## Build configurations

There are **five** ways to build the SkystarsF405v2. The difference is purely in
which default-parameter file gets baked into the firmware — same code either way.

**Boat and GPS are independent** (a boat may or may not carry a GPS), so they're
separate `.params` files. For a board that needs both, `boat_gps.params` chains
them: `boat_gps.params` → `gps.params` → `defaults.parm`. `@include`d files
compose and later lines win, so a child override beats the inherited value.

| Build | File | GPS? | Boat gyro? |
|-------|------|------|-----------|
| Default / FPV | [`defaults.parm`](libraries/AP_HAL_ChibiOS/hwdef/SkystarsF405v2/defaults.parm) | no | no |
| 4G modem | [`4gmodem.params`](libraries/AP_HAL_ChibiOS/hwdef/SkystarsF405v2/4gmodem.params) | MAVLink | no |
| GPS | [`gps.params`](libraries/AP_HAL_ChibiOS/hwdef/SkystarsF405v2/gps.params) | SERIAL5 | no |
| Boat | [`boats.params`](libraries/AP_HAL_ChibiOS/hwdef/SkystarsF405v2/boats.params) | no | yes |
| Boat + GPS | [`boat_gps.params`](libraries/AP_HAL_ChibiOS/hwdef/SkystarsF405v2/boat_gps.params) | SERIAL5 | yes |

### 1. Default / FPV build

```
./waf configure --board SkystarsF405v2
./waf copter
```

Uses the board's `defaults.parm`. This is the "normal" quad: no compass, no GPS,
arming checks off. Good for line-of-sight FPV.

### 2. 4G modem build

```
./waf configure --board SkystarsF405v2 \
    --default-parameters "$PWD/libraries/AP_HAL_ChibiOS/hwdef/SkystarsF405v2/4gmodem.params"
./waf copter
```

`4gmodem.params` pulls in everything from `defaults.parm` (via an `@include`
line at the top) and then overrides the bits needed to fly over a 4G link:

| Param | Value | Why |
|-------|-------|-----|
| `SERIAL1_PROTOCOL` | 2 | MAVLink2 to the modem |
| `SERIAL1_BAUD` | 115 | 115200 |
| `GPS1_TYPE` | 14 | GPS injected over MAVLink (`GPS_INPUT`) |
| `EK3_SRC1_POSXY` | 3 | EKF horizontal position from GPS |
| `FS_THR_ENABLE` | 8 | custom RC failsafe (see below) |
| `FS_GCS_ENABLE` | 8 | custom GCS failsafe (see below) |

Two gotchas that bit us, so they're written down:

- **The `--default-parameters` path has to be absolute.** A repo-relative path
  silently falls back to `defaults.parm` with no warning. That's why the command
  above uses `$PWD`.
- **`processed_defaults.parm` is only regenerated at `configure` time**, not on
  `./waf copter`. If you edit a `.params` file, re-run configure or the change
  won't end up in the firmware. Easy to miss because the build still "succeeds".

### 3. GPS build

```
./waf configure --board SkystarsF405v2 \
    --default-parameters "$PWD/libraries/AP_HAL_ChibiOS/hwdef/SkystarsF405v2/gps.params"
./waf copter
```

`gps.params` `@include`s `defaults.parm` and turns on a physical GPS + compass so
a GPS drone comes up ready with no per-board live config. **SERIAL5 is the board's
GPS UART** (`DEFAULT_SERIAL5_PROTOCOL SerialProtocol_GPS` in the hwdef).

| Param | Value | Why |
|-------|-------|-----|
| `SERIAL5_PROTOCOL` | 5 | GPS on UART5 (explicit; also the hwdef default) |
| `GPS1_TYPE` | 1 | u-blox auto-detect + auto-baud |
| `COMPASS_ENABLE` / `COMPASS_USE` | 1 | GPS module's mag on I2C |
| `EK3_SRC1_POSXY` / `VELXY` | 3 | EKF horizontal pos/vel from GPS (yaw from compass) |
| `ARMING_NEED_LOC` | 0 | GPS lock **not** required to arm |
| `FLTMODE1..6` | Stab/AltHold/Loiter | ch6, low→high |

GPS lock isn't needed to arm (`ARMING_CHECK` stays 0) — it's for Loiter/autonomy
once locked; Stab/AltHold fly without it. Compass + accel still calibrate per drone.

### 4. Boat-launch build

```
./waf configure --board SkystarsF405v2 \
    --default-parameters "$PWD/libraries/AP_HAL_ChibiOS/hwdef/SkystarsF405v2/boats.params"
./waf copter
```

`boats.params` `@include`s `defaults.parm` (**no GPS** — boats don't always carry
one) and adds only the gyro overrides for launching off a rocking boat:

| Param | Value | Why |
|-------|-------|-----|
| `INS_GYR_CAL` | 0 | never calibrate gyros at boot (`GYRO_CAL_NEVER`) — a moving boat corrupts the start-up cal; use saved offsets instead |
| `ARMING_CHECK` | 0 | all arming checks off (incl. INS bit 4 = gyro/accel) so a disturbed gyro can't block arming |

**One-time setup:** do a gyro calibration on solid, level ground first so good
`INS_GYROFFS_*` offsets are saved — with `INS_GYR_CAL=0` the FC uses those
instead of recalibrating each boot. Same two gotchas as the 4G build apply
(absolute `--default-parameters` path; re-run `configure` after editing).

### 5. Boat + GPS build

```
./waf configure --board SkystarsF405v2 \
    --default-parameters "$PWD/libraries/AP_HAL_ChibiOS/hwdef/SkystarsF405v2/boat_gps.params"
./waf copter
```

For a boat that **does** carry a GPS. `boat_gps.params` chains
`gps.params` → `defaults.parm` and then adds the same `INS_GYR_CAL=0` /
`ARMING_CHECK=0` gyro overrides — i.e. the GPS build plus the boat build, in one
file (since `--default-parameters` takes only one). This is the build that's on
board `205A34733441`.

## SkystarsF405v2 changes

### Tuned defaults, sensors off

The board's `defaults.parm` carries our full tune and turns off what we don't
fly with. The headline ones:

- `COMPASS_ENABLE 0`, `COMPASS_USE/USE2/USE3 0` — no mag
- `GPS1_TYPE 0` — no GPS in the default build (the 4G build turns this back on)
- `EK3_SRC1_POSXY 0` / `VELXY 0` — EKF runs without a horizontal position source
- `ARMING_CHECK 0` — checks disabled (FPV, we arm on the bench a lot)
- `FRAME_CLASS 1`, `FRAME_TYPE 12` — quad, betaflight X motor ordering
- `INS_GYRO_FILTER 42`, `AHRS_EKF_TYPE 3`

If you're using this board for something that actually needs GPS or compass,
don't start from the default build — start from the 4G params or turn those
back on yourself.

### Camera switch on RELAY3 (CAM_SW / GPIO 82)

The `CAM_SW` pin (PC3 / GPIO 82) is wired up as RELAY3 so you can switch cameras
from an RC channel.

- `hwdef.dat`: `PC3 CAM_SW OUTPUT GPIO(82) LOW`, plus `RELAY3_PIN_DEFAULT 82`
- `defaults.parm`: `RELAY3_PIN 82`, `RELAY3_FUNCTION 1`, `RELAY3_DEFAULT 0`,
  `RELAY3_INVERTED 1`

Heads up on the polarity: the relay output is `DEFAULT XOR INVERTED`. With
`DEFAULT 0` and `INVERTED 1` the pin idles **high** once AP_Relay takes over
(the hwdef holds it low for the brief moment before that). If you want it low
at boot *and* inverted, set `RELAY3_DEFAULT 1`.

### MAVLink GPS backend re-enabled

This one is non-obvious and cost us some head-scratching. The board includes
`minimize_fpv_osd.inc` to save flash, which disables every GPS backend except a
couple — and the MAVLink one is not in that list. So `GPS1_TYPE 14` pointed at a
driver that wasn't compiled in, and GPS-over-MAVLink just silently did nothing
even though the packets arrived.

Fix is one line at the bottom of `hwdef.dat`, after the minimize include:

```
define AP_GPS_MAV_ENABLED 1
```

Costs about 1 KB of flash. Because it's a compile-time flag it can't live in the
params file, so it's on for *all* SkystarsF405v2 builds, not just the 4G one.

### ESC configurator passthrough (`SERVO_BLH_MASK`)

Stock ArduPilot ships `SERVO_BLH_MASK 0`, which means the BLHeli/4-way-if
pass-thru protocol handler never gets installed
(`AP_BLHeli::init()` in [`AP_BLHeli.cpp`](libraries/AP_BLHeli/AP_BLHeli.cpp)) — so a GCS-based ESC
configurator (Mission Planner, BLHeli Suite) can't talk to the ESCs at all, even
though the feature is fully compiled in (`HAL_SUPPORT_RCOUT_SERIAL` defaults to
1 and nothing in `hwdef.dat` / `minimize_fpv_osd.inc` strips it). Betaflight
doesn't have an equivalent gate, which is why passthrough "just works" there but
not on a stock ArduPilot build.

We now default `SERVO_BLH_MASK` to `15` (M1–M4) so passthrough is enabled out
of the box.

Note this isn't a serial-port setting — the ESCs on M1–M4 aren't on a UART,
they're on the motor output pins, and pass-thru bit-bangs those pins directly.
`SERIAL3` (`USART3`) carries one-way ESC *telemetry* only (RPM/voltage/temp),
not configurator traffic. The only serial-like knob involved is
`SERVO_BLH_PORT`, which picks which **MAVLink connection** (not a UART) relays
the pass-thru frames — it defaults to `0`, matching the USB MAVLink link
(`SERIAL0`), and that's correct as-is since `SERIAL2`/`SERIAL3` don't run
MAVLink in this config (RCIN and ESC telemetry respectively).

## Custom failsafe action: Loiter-or-AltHold

We added a new failsafe action for the **RC/throttle** and **GCS** failsafes.
`FS_THR_ENABLE` and `FS_GCS_ENABLE` now **default to 8** (this action) on **all
builds** — it's baked into `defaults.parm`, not just the 4G variant. On the
default no-GPS build `position_ok()` is false, so it lands in **AltHold**; with
GPS (4G build) it holds **Loiter**.

Behaviour: if we have a usable position estimate (`position_ok()`), switch to
**Loiter**. If we don't, fall back to **AltHold**. If AltHold can't start
either, Land as a last resort.

The point is the 4G case. The cellular link is the command path *and* it's
feeding GPS, so if it drops you may also be losing position. Loiter would be the
wrong call with a stale position, so it degrades to AltHold instead of holding
a position it can't trust.

Touched files:
- [`Copter.h`](ArduCopter/Copter.h) — `FailsafeAction::LOITER_OR_ALTHOLD = 8`
- [`defines.h`](ArduCopter/defines.h) — `FS_THR_ENABLED_LOITER_OR_ALTHOLD` / `FS_GCS_ENABLED_LOITER_OR_ALTHOLD`
- [`events.cpp`](ArduCopter/events.cpp) — the switch cases, the dispatcher case, and
  `set_mode_loiter_or_althold()`
- [`Parameters.cpp`](ArduCopter/Parameters.cpp) — added `8:...` to the `FS_THR_ENABLE` / `FS_GCS_ENABLE`
  value lists so it shows in the GCS dropdown

Note this is RC and GCS only. The EKF failsafe (`FS_EKF_ACTION`) is untouched
and still does AltHold — which is correct, since an EKF failsafe means position
is already untrusted.

## VTX power + pitmode (IRC Tramp, SERIAL6)

The VTX is IRC Tramp on SERIAL6 (`SERIAL6_PROTOCOL 44`, half-duplex via
`SERIAL6_OPTIONS 4`, forced 9600 baud by `AP_TRAMP_UART_BAUD`). A live Tramp
probe of the actual hardware (2026-07-06) reported **max 2500 mW** (not 3 W/4 W)
and **pit mode works** (device returned pit=1 / act 0 mW).

We **restored ArduPilot's stock power switching** (an earlier commit had gutted
it to pitmode-only). Now both power and pitmode work off the 6-position RC
switch (`RC10_OPTION 94`):

- `AP_VideoTX::update_power()` and `change_power()` reverted to stock behaviour
  (stock `change_power` already spreads the active power levels across the
  switch, with the 0 mW level = pitmode at the bottom).
- Power table (`AP_VideoTX::_power_levels[]`) curated to this VTX: active levels
  **0(PIT)/25/200/500/1000/2500 mW**, everything else marked `Inactive`. Tramp
  has no discrete power table — it snaps the requested mW to its nearest internal
  step — so these are just the setpoints we cycle through.
- `VTX_MAX_POWER` / `VTX_POWER` default **2500** (was 3000, capped at the real
  hardware max). `VTX_POWER` param description reverted to the stock meaning.
- Stock safety kept: pitmode via switch only engages when disarmed.

Files: [`AP_VideoTX.cpp`](libraries/AP_VideoTX/AP_VideoTX.cpp) / [`.h`](libraries/AP_VideoTX/AP_VideoTX.h).

## Parameter persistence — defaults.parm must stay small (READ THIS)

**Symptom we hit:** params set over MAVLink read back fine but revert to default
after every reboot — nothing saves. This is upstream issue #27196 and it is
**not board-specific**.

**Cause:** `defaults.parm` was a full 1042-param / 17 KB dump. The board's param
storage is only 16 KB (`HAL_STORAGE_SIZE`), and an oversized embedded defaults
file breaks the storage commit — new (appended) storage entries fail with
`eeprom_full` (`AP_Param.cpp:1235`), so runtime saves are lost. `erase_all` does
NOT fix it; only shrinking the file does.

**Fix:** `defaults.parm` trimmed to **223 params / 3.7 KB** — only the real
deviations from ArduPilot stock (tune, OSD layout, frame, sensors-off, VTX,
serial, RC options+cal, RSSI, failsafe, battery). Calibration/instance params
(`INS_ACCOFFS`, `INS_*_ID`, `COMPASS_*` cal, `BARO*_GND*`/`BARO*_DEVID`,
`AHRS_TRIM`, `MOT_THST_HOVER`, `STAT_*`) were deliberately **left out** — those
belong in EEPROM storage, saved per-board. Keep this file lean; if you need to
add a lot (e.g. a new OSD layout) watch the size.

The trimmed file was built by a **guaranteed reconciliation**: flash a
no-defaults "stock reference" build, dump the board's true code+hwdef defaults,
diff the committed original (plus intended VTX/BLH/FS overrides) against it, and
keep every real deviation (including `VTX_ENABLE`-gated children etc.). Verified
end-to-end: **0 mismatches** across all 1038 intended non-cal params, and
runtime saves persist across reboot. (An earlier quick trim had silently dropped
`RSSI_*`, `RELAY1_*`, `FS_OPTIONS`, and `FS_DR_ENABLE` — the reconciliation
restored them.)

**Two gotchas that cost real time here:**

1. **Editing `defaults.parm` alone does nothing** — waf only re-embeds it on
   `./waf configure`, not on `./waf copter`. It silently flashes the stale old
   defaults otherwise. Always:
   ```
   rm build/SkystarsF405v2/processed_defaults.parm build/SkystarsF405v2/hwdef.h
   python3 waf configure --board SkystarsF405v2 && python3 waf copter
   ```
   then confirm `build/SkystarsF405v2/processed_defaults.parm` shows your values.
2. **Reliable param erase** = `MAV_CMD_PREFLIGHT_STORAGE` with `param1=2`. The
   old `FORMAT_VERSION=0` reset trick does NOT work on current Copter.

On an already-configured board, `defaults.parm` values do NOT overwrite params
already saved in storage — do a `PREFLIGHT_STORAGE param1=2` erase to make the
board pick up the (trimmed) embedded defaults cleanly, then redo accel cal.

## ESC configurator passthrough is now actually enabled

`SERVO_BLH_MASK` defaults to **15** (M1–M4). Note: because of the reconfigure
gotcha above, this only reached the firmware once we forced a reconfigure — an
earlier flash still had the stale `SERVO_BLH_MASK 0`, so passthrough silently
didn't work until then. See [[ardupilot-defaults-parm-persistence]].

## Low-power USB standby / RC wake (`STBY_*`)

New in this branch: [`AP_StandbyPower`](libraries/AP_Vehicle/AP_StandbyPower.cpp),
compiled in only for SkystarsF405v2 (`AP_STANDBY_POWER_ENABLED` in `hwdef.dat`).

The use case: the drone sits with the main battery **disconnected**, FC + RX
powered from a detachable USB-C power bank. A MOSFET device connects the main
battery when its gate is driven to 3.3 V by Relay 1 (pin 54 / PB6). The bank
detaches at liftoff.

With `STBY_EN=1`, if the FC boots and sees **no main battery** (PC0 ADC) and
**no USB host**, it halts initialisation immediately after parameter load —
before sensors, OSD, logging, the scheduler and the watchdog — and sits in a
minimal loop parsing CRSF straight off SERIAL2. The MCU idles in WFI between
frames; the only activity is the RC listen and a short LED blink every 2 s.
The AT7456E OSD chip is also put in its lowest-power state (software reset +
video buffer disable over SPI) — `AP_OSD_MAX7456::init()` fully reconfigures
it when boot resumes, so there's nothing to undo.

Wake sequence:

1. Wake channel (`STBY_CHAN`, default 7 — same channel that drives Relay 1 via
   `RC7_OPTION=28`) held above `STBY_TRIG` (default 1800) for 0.5 s of fresh,
   CRC-valid frames.
2. Relay 1's pin is driven high → battery connects.
3. Once PC0 confirms battery voltage (`STBY_BATT_V`, default 7 V), normal boot
   **continues in place** — no reset, so the gate never glitches. If the
   battery doesn't appear within 10 s (or the switch is released), the pin is
   released and it re-arms.

Escape hatches — both resume a completely normal boot:

- battery already present at power-up (normal field use)
- a USB **host** enumerates (bench configuration; a dumb power bank never
  enumerates)

Params (all under `STBY_`): `_EN`, `_CHAN`, `_TRIG`, `_RELAY` (relay instance,
1-based), `_UART` (serial port carrying CRSF, default 2), `_BATT_V`.

Two coupled changes to be aware of:

- `RELAY1_DEFAULT` moved `0 → 2` (**no change**) in `defaults.parm`. Relay init
  must not slam the pin low after the standby code set it high. Side effect: on
  a normal boot the pin is left floating instead of driven low — the MOSFET
  device's gate pulldown keeps it off, same net behaviour as before.
- Firmware **cannot** protect against an in-flight FC reboot: during MCU reset
  the pin goes hi-Z and main power drops. That hazard predates this feature —
  it's inherent to gating the battery through an FC pin.

## Behaviour tweaks

### Auto mode with no mission

[`mode_auto.cpp`](ArduCopter/mode_auto.cpp): if you switch to Auto with no mission loaded (`num_commands <= 1`),
it now drops into **Guided** instead of sitting there. It also:

- refuses the Auto switch if armed-and-landed and the mission doesn't start with
  a takeoff (prints `Auto: Missing Takeoff Cmd`), to cut down on flip-on-arm
- clears the mission on completion

This is mainly for the over-the-link / companion-controlled flying where Guided
is what we actually want.

### Snappier arming / spool

- [`config.h`](ArduCopter/config.h): `ARMING_DELAY_SEC` 2.0 → **0.0**
- [`AP_MotorsMulticopter.h`](libraries/AP_Motors/AP_MotorsMulticopter.h): `AP_MOTORS_SPOOL_UP_TIME_DEFAULT` 0.5 → **0.05**

Faster arm and spool-up. Fine for a small FPV quad; you probably don't want
these on anything big or heavy.

## Stray files (so they're not a surprise)

A few things rode along that aren't really part of the feature work. Calling
them out rather than pretending they're not there:

- `libraries/AP_HAL_ChibiOS/hwdef/SkystarsF405v2/hwdef.dat.bak` — a backup of
  the hwdef that got committed. Harmless, but it's clutter and can be deleted.
- `Tools/bootloaders/SequreH743_bl.{bin,hex}` — changed, unrelated to Skystars.
  Almost certainly a side effect of building another target. Worth reverting if
  you don't want it in the branch.
- `modules/littlefs` — submodule pointer added.

## Flashing

Standard ArduPilot. With the board in STM32 DFU mode (`0483:df11`):

```
arm-none-eabi-objcopy -I ihex -O binary \
  build/SkystarsF405v2/bin/arducopter_with_bl.hex /tmp/fw.bin
dfu-util -a 0 -d 0483:df11 -s 0x08000000:leave -D /tmp/fw.bin
```

Use the `_with_bl.hex` (includes the bootloader, base `0x08000000`). The board
re-enumerates as a USB serial device after the `:leave`.

If `dfu-util -l` shows nothing while the DFU LED is on, it's almost always the
USB cable (charge-only) or a tired USB connector on the board — power is fine
(LED), data isn't reaching the host. Swap the cable before assuming anything's
broken.

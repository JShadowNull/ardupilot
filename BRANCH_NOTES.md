# Branch notes — `arducopter-4.6.3_vtx3w_v1.1.0`

This is our fork of ArduCopter 4.6.3. It carries a handful of changes on top of
upstream, mostly for the **SkystarsF405v2** FPV board and a 3W VTX setup. Read
this before you build or flash so nothing here catches you off guard.

Everything below is relative to `master` (upstream 4.6.3). If a param or flag
isn't mentioned here, assume it's stock.

## What this branch is for

Two things, really:

1. A tuned SkystarsF405v2 target that flies as an FPV quad with the sensors we
   don't use turned off.
2. Support for a 3W analog VTX (the `vtx3w` in the branch name) where the VTX
   manages its own power and we only toggle pitmode from the radio.

On top of that there's a 4G-modem build variant for flying over a cellular
link with GPS fed in over MAVLink, plus a couple of behaviour tweaks (auto
mode, arming, motor spool).

## Build configurations

There are **two** ways to build the SkystarsF405v2. The difference is purely in
which default-parameter file gets baked into the firmware — same code either way.

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

## Custom failsafe action: Loiter-or-AltHold

We added a new failsafe action for the **RC/throttle** and **GCS** failsafes.
Set `FS_THR_ENABLE` or `FS_GCS_ENABLE` to **8** to use it. The 4G build does
this; the default build leaves the stock values.

Behaviour: if we have a usable position estimate (`position_ok()`), switch to
**Loiter**. If we don't, fall back to **AltHold**. If AltHold can't start
either, Land as a last resort.

The point is the 4G case. The cellular link is the command path *and* it's
feeding GPS, so if it drops you may also be losing position. Loiter would be the
wrong call with a stale position, so it degrades to AltHold instead of holding
a position it can't trust.

Touched files:
- `Copter.h` — `FailsafeAction::LOITER_OR_ALTHOLD = 8`
- `defines.h` — `FS_THR_ENABLED_LOITER_OR_ALTHOLD` / `FS_GCS_ENABLED_LOITER_OR_ALTHOLD`
- `events.cpp` — the switch cases, the dispatcher case, and
  `set_mode_loiter_or_althold()`
- `Parameters.cpp` — added `8:...` to the `FS_THR_ENABLE` / `FS_GCS_ENABLE`
  value lists so it shows in the GCS dropdown

Note this is RC and GCS only. The EKF failsafe (`FS_EKF_ACTION`) is untouched
and still does AltHold — which is correct, since an EKF failsafe means position
is already untrusted.

## 3W VTX support

For the analog 3W VTX. The firmware does **not** set VTX power any more — the VTX
runs at its own internal setting and we only toggle pitmode from the radio.

- Power table extended up to 3000 mW (1.2/1.6/2.0/2.5/3.0 W entries, SmartAudio
  2.1 only). `VTX_MAX_POWER` and `VTX_POWER` default to 3000.
- `VTX_MAX_POWER_LEVELS` raised from 10 to 15 to fit the new entries.
- RC switch behaviour: position 0 enables pitmode, positions 1–5 disable it.
  Pitmode can be toggled at any time.
- The `VTX_POWER` parameter description now says outright that this firmware
  ignores it, so nobody goes looking for a power change that won't happen.

Files: `AP_VideoTX.cpp/.h`, `AP_CRSF_Telem.cpp`.

## Behaviour tweaks

### Auto mode with no mission

`mode_auto.cpp`: if you switch to Auto with no mission loaded (`num_commands <= 1`),
it now drops into **Guided** instead of sitting there. It also:

- refuses the Auto switch if armed-and-landed and the mission doesn't start with
  a takeoff (prints `Auto: Missing Takeoff Cmd`), to cut down on flip-on-arm
- clears the mission on completion

This is mainly for the over-the-link / companion-controlled flying where Guided
is what we actually want.

### Snappier arming / spool

- `config.h`: `ARMING_DELAY_SEC` 2.0 → **0.0**
- `AP_MotorsMulticopter.h`: `AP_MOTORS_SPOOL_UP_TIME_DEFAULT` 0.5 → **0.05**

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

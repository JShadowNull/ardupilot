# Orqa FC 3030 H7 QuadCore Flight Controller

The Orqa FC 3030 H7 QuadCore is a flight controller produced by [Orqa](https://enterprise.orqafpv.com/).

## MRM2-10 boat-launch Copter build

The `MRM2-10` branch starts from official upstream `ArduPilot-4.6` at
`913e171eca7a37549d9f02e442698edfd4490435`. This base identifies itself as
**ArduCopter 4.6.3 official**; the MR2 changes below make this a custom build.

The board explicitly enables `AP_COMPASS_IIS2MDC_ENABLED` for external
IIS2MDC/LIS2MDL compasses. Both I2C buses retain external compass probing.

The build uses the parameter defaults defined by ArduPilot 4.6.3 and this
board's hardware definition. No aircraft parameter snapshot is embedded as a
`defaults.parm` override. In particular, learned hover thrust, PID gains,
calibrations, battery settings and arming-check overrides are not imported.
The upstream hover-thrust default is `MOT_THST_HOVER=0.35`; automatic hover
learning remains available with upstream `MOT_HOVER_LEARN=2`.

Build from the repository root:

```sh
./waf configure --board OrqaH7QuadCore
./waf copter
```

Firmware outputs: `build/OrqaH7QuadCore/bin/arducopter.apj` for ground-station
updates and `build/OrqaH7QuadCore/bin/arducopter_with_bl.hex` for flashing with
the bootloader. The distributed bootloader-inclusive file is named `MRM2-10.hex`.
Previously saved parameters take precedence over firmware defaults. Flashing
this build does not reset an existing controller's parameters or calibration.
Configure and calibrate each aircraft before flight; compilation does not
verify its hardware setup or flight behavior.

### MR2 firmware features

- RC and GCS loss support optional action 8: try Loiter with a usable position
  estimate, otherwise AltHold, then Land if AltHold cannot start. AltHold can
  drift horizontally. Existing failsafe priorities and `FS_OPTIONS` still
  apply. Upstream failsafe defaults are retained; select action 8 explicitly
  with `FS_THR_ENABLE` / `FS_GCS_ENABLE` when desired.
- Entering Auto without a mission switches to Guided from the run loop. If
  Guided rejects entry, it falls back to Land. Completed missions are cleared;
  the existing completion loiter/landing behavior is preserved. Upload a
  mission again to repeat it.
- Orqa's requested zero post-arm delay (`ARMING_DELAY_SEC`) remains in the
  hardware definition. `MOT_SPOOL_TIME` uses the upstream default; a 0.05-second
  motor ramp can be selected explicitly. Other boards retain existing timing.
- Battery consumption learning is available with `BATT_LRN=2` (learn and save),
  but defaults to disabled. Enable and position `OSD1_TIMEREM` explicitly to
  display estimated minutes/seconds remaining. The panel blinks below the
  `OSD_W_TIMEREM` threshold (default 60 seconds). MAVLink reports the estimate
  too. This display does not trigger a failsafe.
- The estimate uses measured consumed mAh and average current, blending saved
  consumption with the current armed period. Learning occurs at disarm after
  at least 60 seconds armed; ground time while armed also contributes. Accurate
  current calibration and capacity are required. Set `BATT_CAPACITY` and
  `BATT_LOW_MAH` for the actual battery and desired reserve.
- External IIS2MDC/LIS2MDL compass support remains enabled. Detection,
  orientation and calibration must be verified on each aircraft.

To select the optional MR2 behaviors, configure `FS_THR_ENABLE`,
`FS_GCS_ENABLE`, `MOT_SPOOL_TIME`, `BATT_LRN` and `OSD1_TIMEREM_EN/X/Y` on the
controller as appropriate. Do not erase calibration to apply defaults.

Feature sources: Skystars failsafe/Auto changes through `d02d52edf1`, battery
timer `5241253c69`. No VTX, standby-power or Thunder Tiger subsystem changes
are included.

Validation: the Orqa Copter target builds successfully. SITL regression tests
`MR2Failsafe`, `MR2Auto` and `MR2BatteryTime` cover RC/GCS loss to Loiter,
RC loss without horizontal aiding to AltHold, empty-Auto redirection,
completed-mission clearing without leaving its hold, battery reserve arithmetic,
and learned consumption persistence. Run them with
`Tools/autotest/autotest.py --speedup=4 test.Copter.MR2Failsafe test.Copter.MR2Auto test.Copter.MR2BatteryTime`
after building SITL. Hardware compass detection, OSD appearance and physical
motor response still require bench verification.

## Features

 - MCU - STM32H743 32-bit processor
 - IMU - dual ICM42688
 - Barometer - DPS310
 - OSD - AT7456E
 - Onboard Flash: 128Mbit
 - microSD card slot
 - 1x CAN
 - 4x UARTs
 - 10x PWM Outputs (8 Motor Output)
 - Battery input voltage: 2S-8S
 - BEC 5V 2A
 - BEC 10V 2.5A

## Pinout

![Orqa FC 3030 H7 Quadcore Top](OrqaH7QuadCoreTop.png "Orqa FC 3030 H7 QuadCOre Top")
![Orqa FC 3030 H7 QuadCore Bottom](OrqaH7QuadCoreBottom.png "Orqa FC 3030 H7 Quadcore Bottom")

## UART Mapping

The UARTs are marked Rn and Tn in the above pinouts. The Rn pin is the
receive pin for UARTn. The Tn pin is the transmit pin for UARTn.

 - SERIAL0 -> USB
 - SERIAL3 -> UART3 (RX/GHST, DMA-enabled)
 - SERIAL6 -> UART6 (Sik/MAVLink, DMA-enabled)
 - SERIAL7 -> UART7 (GPS)
 - SERIAL8 -> UART8 (ESC Telemetry)

## RC Input

RC input is configured for half duplex protocls (GHST/SRXL2) by default on the T3 (UART3_TX) pin using half-duplex. To support other RC protocols set SERIAL1_OPTIONS to 0 and connect to R3/T3 as required (see https://ardupilot.org/plane/docs/common-rc-systems.html). This will support all serial RC protocols but not PPM.
  
## OSD Support

The Orqa FC 3030 H7 QuadCore supports OSD using OSD_TYPE 1 (MAX7456 driver). HD OSD and other ODSDs can be supported using a UART (see https://ardupilot.org/plane/docs/common-osd-boards-on-screen-display.html)

## PWM Output

The Orqa FC 3030 H7 QuadCore supports up to 10 PWM outputs in the JST-GH ESC and MFC connectors and Servo pads.

The PWM is in 5 groups:

 - PWM 1-2 in group1
 - PWM 3-4 in group2
 - PWM 5-6 in group3
 - PWM 7-8 in group4
 - PWM 9-10 in group5

Channels within the same group need to use the same output rate. If
any channel in a group uses DShot then all channels in the group need
to use dshot. Channels 1-8 support dshot and bi-directional dshot.
All channels can also be used for GPIO.

## Battery Monitoring

The board has a builting voltage and current sensor. The current
sensor can read up to 130 Amps. The voltage sensor can handle up to 6S
LiPo batteries.

The correct battery setting parameters are:

 - BATT_MONITOR 4
 - BATT_VOLT_PIN 13
 - BATT_CURR_PIN 12
 - BATT_VOLT_MULT 8.3
 - BATT_AMP_PERVLT 92.6

## VTX Control

Switching between Camera1 and Camera2 can be achieved via GPIO pin 74 which is configured on RELAY2

## GPIO5
The GPIO5 pin is assigned as GPIO 75
## Compass

The Orqa FC 3030 H7 QuadCore does not have a builtin compass, but you can attach an external compass using I2C on the SDA and SCL pin.

## Loading Firmware
Firmware is found under the "OrqaH7QuadCore" folders on the firmware server: https://firmware.ardupilot.org/

Initial firmware load can be done with DFU by plugging in USB with the
bootloader button pressed. Then you should load the "with_bl.hex"
firmware, using your favourite DFU loading tool.

Once the initial firmware is loaded you can update the firmware using
any ArduPilot ground station software. Updates should be done with the
*.apj firmware files.

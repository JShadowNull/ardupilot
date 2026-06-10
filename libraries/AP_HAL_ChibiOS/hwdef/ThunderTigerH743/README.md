# ThunderTigerH743 Flight Controller

The ThunderTigerH743 is a custom STM32H743 flight controller based on the
Rotor Riot / Ewing Aerospace "Brave H7" OEM board (its USB bootloader
enumerates as `BraveH7-BL`). It is a private target and is not in upstream
ArduPilot.

## Features

 - STM32H743 microcontroller
 - BMI270 IMU (SPI1), mounted ROLL_180
 - BMP390 barometer on I2C (uses the BMP388 driver)
 - MAX7456 analog OSD (SPI2)
 - W25Q128 dataflash for logging (SPI3), no SD card
 - 7 PWM outputs
 - no on-board compass (external on I2C is supported)

## Building

    ./waf configure --board ThunderTigerH743
    ./waf copter
    ./waf --upload copter

The board ID is 11065 and must match the on-board bootloader, otherwise it
will reject the firmware (see "Bootloader" below).

## UART Mapping

The order of the UARTs is OTG1, USART1, USART2, USART3, UART4, UART5, USART6.

 - SERIAL0 -> USB
 - SERIAL1 -> USART1 (GPS), TX PB6 / RX PB7
 - SERIAL2 -> USART2, RX PA3 only - TX (PA2) is used as the LED PWM output, see below
 - SERIAL3 -> USART3, TX PB10 / RX PB11
 - SERIAL4 -> UART4 (RCIN), TX PA0 / RX PA1
 - SERIAL5 -> UART5 (ESC telemetry), TX PC12 / RX PD2
 - SERIAL6 -> USART6, TX PC6 / RX PC7

USART2 has its TX pin repurposed for the LED driver, so only RX is left on
that port. Don't put a bidirectional protocol (MAVLink, GPS) on SERIAL2.

## RC Input

RC input defaults to UART4 (SERIAL4, protocol 23). Connect the receiver to
PA1 (RX) / PA0 (TX).

## OSD Support

Analog OSD is supported via the on-board MAX7456 (OSD_TYPE 1).

## PWM Output

There are 7 PWM outputs. Outputs that share a timer share an update rate:

 - PWM 1, 2   TIM1  (PA9, PA8)
 - PWM 3, 4   TIM3  (PC9, PC8)
 - PWM 5, 6   TIM15 (PE6, PE5)
 - PWM 7      TIM2  (PA2)

PWM 1-4 are the quad motors. PWM 1 and 3 are BIDIR DShot capable. PWM 5 and 6
are spare. PWM 7 is on its own timer and is used by the LED driver (below).

## LED Driver Output (PWM 7 / PA2)

PA2 (USART2 TX) drives the DIM input of a MEAN WELL NLDD-H constant-current
LED driver. It is not a normal servo output - the `AP_PWMLight` library
(`libraries/AP_PWMLight/`) owns the pin, sets the timer frequency and writes
the duty cycle directly so the brightness doesn't depend on SERVOx_MIN/MAX
scaling. The library is enabled by adding it to
`Tools/ardupilotwaf/ardupilotwaf.py`.

It is controlled by the `LIGHT_` parameters:

 - LIGHT_ENABLE (default 1) - enable the output
 - LIGHT_OUT    (default 7) - output channel to drive
 - LIGHT_RCIN   (default 11) - RC input channel, full travel maps to 0-100%
 - LIGHT_FREQ   (default 400) - PWM frequency in Hz

LIGHT_FREQ is clamped to 100-400Hz. The HAL caps normal PWM groups at 400Hz,
which is well inside the NLDD-H's 100-1000Hz dimming range.

While the light is enabled it forces SERVO7_FUNCTION to 0 so the servo layer
won't drive the same pin, and SERVO7_MIN/MAX/TRIM/REVERSED have no effect. To
use PA2 for something else, set LIGHT_ENABLE=0 first.

Wiring: PA2 / TX2 pad to DIM, FC ground to the driver -Vin (shared ground).

## Battery Monitoring

Voltage is on PC0 (pin 10) and current on PC1 (pin 11). BATT_MONITOR defaults
to 4 (analog voltage and current). The default scaling is BATT_VOLT_MULT 11.1
and BATT_AMP_PERVLT 100; adjust to match your power module.

## Compass

There is no on-board compass. External compasses are probed on I2C1
(SCL PB8, SDA PB9). ALLOW_ARM_NO_COMPASS is set so the board can arm without
one.

## GPIOs

The PWM outputs and a couple of spare pads are available as GPIOs:

 - 50-56  PWM outputs 1-7
 - 80     buzzer (PC13)
 - 90     status LED (PC14, active high)
 - 91     spare input (PA10, "CC")
 - 92     spare input (PB0, "PIT")

## Bootloader

The on-board bootloader is in the OpenDroneID range (board id 11065 =
10000 + 1065), which means it verifies firmware before booting. The hwdef
sets `AP_CHECK_FIRMWARE_ENABLED 1` so the app descriptor (image CRC and board
id) gets embedded; without it the bootloader returns FAIL_REASON_NO_APP_SIG.

`AP_BOOTLOADER_FLASHING_ENABLED 0` keeps the firmware from touching the
bootloader - the board keeps the OEM one. A bootloader build (`hwdef-bl.dat`,
prebuilt in `Tools/bootloaders/ThunderTigerH743_bl.*`) is included for
flashing manually over SWD if it ever needs replacing.

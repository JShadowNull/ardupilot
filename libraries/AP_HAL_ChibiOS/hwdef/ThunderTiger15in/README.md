# ThunderTiger 15in Platform

The 15 inch ThunderTiger platform. The flight-controller hardware is
**identical** to the [ThunderTiger7in](../ThunderTiger7in/README.md) board -
the only difference between the two is the default tune.

This target is defined entirely by inheritance:

 - `hwdef.dat` does `include ../ThunderTiger7in/hwdef.dat`, so the hardware
   definition (pins, peripherals, `APJ_BOARD_ID` 11065) is shared verbatim.
 - `hwdef-bl.dat` likewise includes the 7in bootloader definition.
 - `defaults.parm` does `@include ../ThunderTiger7in/defaults.parm` and then
   overrides **only** the airframe tune (rate/angle PIDs, accel limits,
   harmonic notch, motor thrust model).

Because both platforms build with board ID 11065, either firmware will flash
the same physical hardware - pick the firmware that matches the airframe.

## Tuning

The tune block at the bottom of `defaults.parm` is seeded from the 7in values
as a starting point and **must be retuned for the 15in airframe** before
flight. To change a shared (non-tune) default, edit the 7in `defaults.parm`
and both platforms pick it up.

## Building

    ./waf configure --board ThunderTiger15in
    ./waf copter
    ./waf --upload copter

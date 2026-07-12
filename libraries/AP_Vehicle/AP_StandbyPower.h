/*
  low-power standby mode for USB-powered pre-flight state

  When enabled, and the vehicle boots with no main battery present (and
  no USB host attached), initialisation is halted very early - before
  sensors, logging, OSD and the watchdog are started. The MCU sits in a
  minimal loop listening to the RC receiver (CRSF) directly. When the
  configured wake channel is held high, the configured relay pin is
  driven to connect the main battery; once battery voltage is confirmed
  the normal boot continues in-place (no reset, so the relay pin never
  glitches).

  Escape hatches:
   - main battery already present at boot -> normal boot
   - USB host enumerates (bench configuration) -> normal boot
 */
#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef AP_STANDBY_POWER_ENABLED
#define AP_STANDBY_POWER_ENABLED 0
#endif

#if AP_STANDBY_POWER_ENABLED

#include <AP_Param/AP_Param.h>
#include <AP_HAL/AP_HAL.h>

class AP_StandbyPower {
public:
    AP_StandbyPower();

    /* Do not allow copies */
    CLASS_NO_COPY(AP_StandbyPower);

    static const struct AP_Param::GroupInfo var_info[];

    // called once, very early in AP_Vehicle::setup() after parameters
    // are loaded. Blocks in standby until a wake condition is met.
    void check(void);

private:
    AP_Int8  enable;          // STBY_EN
    AP_Int8  channel;         // STBY_CHAN: RC channel that wakes the vehicle
    AP_Int16 trigger_us;      // STBY_TRIG: channel PWM threshold
    AP_Int8  relay_instance;  // STBY_RELAY: relay instance (1..6) driving the battery switch
    AP_Int8  uart_num;        // STBY_UART: serial port number carrying RC input
    AP_Float batt_volt;       // STBY_BATT_V: voltage above which the battery is "present"

    bool battery_present(void);
    bool resolve_relay_pin(int16_t &pin, bool &inverted) const;
    void set_relay(int16_t pin, bool inverted, bool on) const;
    void disable_osd(void) const;
    bool parse_byte(uint8_t b);
    void decode_channels(const uint8_t *payload);

    AP_HAL::AnalogSource *batt_source;

    // CRSF parser state
    uint8_t  buf[64];
    uint8_t  buf_len;
    uint16_t channels_us[16];
    uint32_t last_frame_ms;
};

#endif // AP_STANDBY_POWER_ENABLED

#pragma once

/*
  drive a single PWM output at an exact duty cycle for an external
  LED driver (e.g. MEAN WELL NLDD-H DIM input), with brightness
  controlled from an RC channel
 */

#include <AP_Param/AP_Param.h>
#include <AP_HAL/AP_HAL_Boards.h>

class AP_PWMLight {
public:
    AP_PWMLight();

    /* Do not allow copies */
    AP_PWMLight(const AP_PWMLight &other) = delete;
    AP_PWMLight &operator=(const AP_PWMLight &) = delete;

    // periodic update, called at 50Hz from the vehicle scheduler
    void update();

    static const struct AP_Param::GroupInfo var_info[];

private:
    AP_Int8  _enable;
    AP_Int8  _out_chan;   // output channel to drive, 1-based
    AP_Int8  _rc_chan;    // RC input channel for brightness, 1-based
    AP_Int16 _freq_hz;    // PWM frequency

    uint16_t _set_freq_hz;  // last frequency programmed into the timer
};

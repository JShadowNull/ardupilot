#include "AP_PWMLight.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <RC_Channel/RC_Channel.h>
#include <SRV_Channel/SRV_Channel.h>

// the HAL caps normal PWM groups at 400Hz
#define PWMLIGHT_FREQ_MIN 100
#define PWMLIGHT_FREQ_MAX 400

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_PWMLight::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: PWM light enable
    // @Description: Enable PWM light output
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO_FLAGS("ENABLE", 1, AP_PWMLight, _enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: OUT
    // @DisplayName: PWM light output channel
    // @Description: Output channel driven as the dimming PWM. The channel's SERVOx_FUNCTION is forced to 0 (Disabled) while the light is enabled.
    // @Range: 1 16
    // @User: Standard
    AP_GROUPINFO("OUT", 2, AP_PWMLight, _out_chan, 7),

    // @Param: RCIN
    // @DisplayName: PWM light RC input channel
    // @Description: RC input channel whose full travel maps to 0 to 100% duty cycle
    // @Range: 1 16
    // @User: Standard
    AP_GROUPINFO("RCIN", 3, AP_PWMLight, _rc_chan, 11),

    // @Param: FREQ
    // @DisplayName: PWM light frequency
    // @Description: PWM output frequency
    // @Range: 100 400
    // @Units: Hz
    // @User: Standard
    AP_GROUPINFO("FREQ", 4, AP_PWMLight, _freq_hz, 400),

    AP_GROUPEND
};

AP_PWMLight::AP_PWMLight() :
    _set_freq_hz(0)
{
    AP_Param::setup_object_defaults(this, var_info);
}

void AP_PWMLight::update()
{
    if (!_enable) {
        return;
    }

    const uint8_t out = (uint8_t)_out_chan;
    if (out < 1 || out > NUM_SERVO_CHANNELS) {
        return;
    }
    const uint8_t ch = out - 1;

    // keep the servo function disabled so the servo layer doesn't
    // drive the same pin
    SRV_Channel *c = SRV_Channels::srv_channel(ch);
    if (c != nullptr && c->get_function() != SRV_Channel::k_none) {
        c->function_set_and_save(SRV_Channel::k_none);
    }

    const uint16_t freq = (uint16_t)constrain_int16(_freq_hz, PWMLIGHT_FREQ_MIN, PWMLIGHT_FREQ_MAX);

    // program the timer frequency once on change. Anything above 50Hz
    // also marks the group fast so SERVO_RATE can't reset it
    if (_set_freq_hz != freq) {
        hal.rcout->set_freq(1U << ch, freq);
        hal.rcout->enable_ch(ch);
        _set_freq_hz = freq;
    }

    // map the RC channel's full travel to duty cycle, off without valid input
    float duty = 0.0f;
    const RC_Channel *rcin = rc().channel(_rc_chan - 1);
    if (rcin != nullptr && rc().has_valid_input()) {
        duty = (rcin->norm_input_ignore_trim() + 1.0f) * 0.5f;
    }
    duty = constrain_float(duty, 0.0f, 1.0f);

    // write the pulse width directly, bypassing SERVOx_MIN/MAX scaling
    const float period_us = 1.0e6f / (float)freq;
    SRV_Channels::set_output_pwm_chan(ch, (uint16_t)lroundf(duty * period_us));
}

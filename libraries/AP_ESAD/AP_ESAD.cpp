/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "AP_ESAD.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Relay/AP_Relay.h>
#include <AP_Baro/AP_Baro.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_ESAD::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: ESAD enable
    // @Description: Enable the Electronic Safe-Arm Device liftoff timer, arm/fire sequencing and relay interlock
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO_FLAGS("ENABLE", 1, AP_ESAD, _enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: ARM_RLY
    // @DisplayName: ESAD arm relay
    // @Description: Relay instance DRIVEN as the ARM output (0 = RELAY1, 1 = RELAY2, ...). The firmware is the sole driver of this relay; do not also map it to an RC RELAY aux function.
    // @Range: 0 5
    // @User: Standard
    AP_GROUPINFO("ARM_RLY", 2, AP_ESAD, _arm_relay, 0),

    // @Param: FIRE_RLY
    // @DisplayName: ESAD fire relay
    // @Description: Relay instance DRIVEN as the FIRE output. The firmware is the sole driver of this relay; do not also map it to an RC RELAY aux function.
    // @Range: 0 5
    // @User: Standard
    AP_GROUPINFO("FIRE_RLY", 3, AP_ESAD, _fire_relay, 1),

    // @Param: TIMER
    // @DisplayName: ESAD post-liftoff arming countdown
    // @Description: Arming countdown in seconds. Starts at liftoff; both relays are locked off and any arm/fire request is an error until it completes.
    // @Range: 0 600
    // @Units: s
    // @User: Standard
    AP_GROUPINFO("TIMER", 4, AP_ESAD, _timer_s, 60),

    // @Param: LIFT_ALT
    // @DisplayName: ESAD liftoff altitude
    // @Description: Height (from the barometer, relative to the altitude captured at arming) that must be exceeded to count as liftoff and start the arming countdown.
    // @Range: 0.5 20
    // @Units: m
    // @User: Standard
    AP_GROUPINFO("LIFT_ALT", 5, AP_ESAD, _lift_alt, 2.0f),

    // @Param: OSD_EN
    // @DisplayName: ESAD OSD enable
    // @Description: Show the combined ESAD OSD element. It renders the status (E:S safe, E:A armed, E:F fired, ERR error) and, while the post-liftoff countdown runs, appends the remaining seconds (e.g. "E:S 60s").
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("OSD_EN", 6, AP_ESAD, _osd_en, 1),

    // @Param: OSD_X
    // @DisplayName: ESAD OSD X
    // @Description: Horizontal position of the ESAD element on screen
    // @Range: 0 29
    // @User: Standard
    AP_GROUPINFO("OSD_X", 7, AP_ESAD, _osd_x, 1),

    // @Param: OSD_Y
    // @DisplayName: ESAD OSD Y
    // @Description: Vertical position of the ESAD element on screen
    // @Range: 0 15
    // @User: Standard
    AP_GROUPINFO("OSD_Y", 8, AP_ESAD, _osd_y, 14),

    // @Param: SAFETY
    // @DisplayName: ESAD safety enforcement
    // @Description: When enabled the firmware adds the post-liftoff arming countdown (both relays locked off until it completes) on top of the arm-then-fire ordering interlock. When disabled the countdown is skipped, but the arm-then-fire order is STILL enforced: relay 2 (fire) is never energized before relay 1 (arm), and a fire-before-arm request latches an error. The OSD element shows in both modes.
    // @Values: 0:Disabled (ordering interlock only),1:Enabled (ordering interlock + countdown)
    // @User: Standard
    AP_GROUPINFO("SAFETY", 9, AP_ESAD, _safety, 1),

    AP_GROUPEND
};

AP_ESAD *AP_ESAD::_singleton;

AP_ESAD::AP_ESAD()
{
    AP_Param::setup_object_defaults(this, var_info);
    _singleton = this;
}

void AP_ESAD::set_relay(uint8_t instance, bool on)
{
    AP_Relay *relay = AP::relay();
    if (relay == nullptr) {
        return;
    }
    if (on) {
        relay->on(instance);
    } else {
        relay->off(instance);
    }
}

void AP_ESAD::set_arm_output(bool on)
{
    set_relay((uint8_t)_arm_relay, on);
}

void AP_ESAD::set_fire_output(bool on)
{
    set_relay((uint8_t)_fire_relay, on);
}

uint16_t AP_ESAD::countdown_remaining_s() const
{
    if (!_lifted_off || _timer_done) {
        return 0;
    }
    const uint32_t total_ms = (uint32_t)_timer_s.get() * 1000UL;
    const uint32_t elapsed = AP_HAL::millis() - _liftoff_ms;
    if (elapsed >= total_ms) {
        return 0;
    }
    // round up so the display shows the full second until it actually elapses
    return (uint16_t)((total_ms - elapsed + 999UL) / 1000UL);
}

void AP_ESAD::update()
{
    if (_enable == 0) {
        return;
    }

    // Safety enforcement disabled: skip the vehicle-armed gating and the
    // post-liftoff countdown, but STILL enforce the arm-then-fire relay order.
    // Relay 2 (fire) is never energized before relay 1 (arm), and a
    // fire-before-arm request latches ERROR so neither relay activates until
    // both requests return low. The external ESAD device adds its own safety on
    // top of this ordering interlock.
    if (_safety == 0) {
        _lifted_off = false;   // no countdown in passthrough
        _timer_done = false;
        if (_state == State::LOCKED) {
            _state = State::SAFE;   // no lockout phase in passthrough
        }
        run_sequence(_arm_request, _fire_request);
        return;
    }

    const bool armed = hal.util->get_soft_armed();
    const uint32_t now = AP_HAL::millis();

    // Disarmed: nothing to monitor, fully safe and reset. One countdown per
    // flight - everything re-arms only after the next arm.
    if (!armed) {
        _armed_prev = false;
        _lifted_off = false;
        _timer_done = false;
        _state = State::SAFE;
        set_arm_output(false);
        set_fire_output(false);
        return;
    }

    // Armed: start watching the baro. Capture the ground reference at the moment
    // of arming so liftoff is measured relative to the launch altitude.
    const float alt = AP::baro().get_altitude();   // m, relative to baro ground reference
    if (!_armed_prev) {
        _armed_prev = true;
        _ref_alt = alt;
        _lifted_off = false;
        _timer_done = false;
        _state = State::LOCKED;
    }

    // detect liftoff once (baro climbed past the threshold above the arm altitude)
    if (!_lifted_off && (alt - _ref_alt) >= _lift_alt) {
        _lifted_off = true;
        _liftoff_ms = now;
    }

    // post-liftoff countdown completion
    if (_lifted_off && !_timer_done &&
        (now - _liftoff_ms) >= (uint32_t)_timer_s.get() * 1000UL) {
        _timer_done = true;
    }

    const bool arm_req = _arm_request;
    const bool fire_req = _fire_request;

    // Locked until the post-liftoff countdown completes: both relays off, and
    // any arm/fire request is a premature-action error.
    if (!_timer_done) {
        set_arm_output(false);
        set_fire_output(false);
        if (_state == State::ERROR) {
            if (!arm_req && !fire_req) {
                _state = State::LOCKED;  // cleared, still locked
            }
        } else if (arm_req || fire_req) {
            _state = State::ERROR;
        } else {
            _state = State::LOCKED;
        }
        return;
    }

    // Unlocked: enforce arm-then-fire.
    run_sequence(arm_req, fire_req);
}

// Arm-then-fire sequencer shared by both safety modes. Relay 2 (fire) is only
// energized once relay 1 (arm) is high; a fire-before-arm request latches
// ERROR (both relays held off) until both requests return low, so an
// out-of-sequence fire can never be recovered by then raising arm.
void AP_ESAD::run_sequence(bool arm_req, bool fire_req)
{
    switch (_state) {
    case State::LOCKED:
        // countdown just completed - drop to safe, awaiting an arm request
        set_arm_output(false);
        set_fire_output(false);
        _state = State::SAFE;
        break;

    case State::SAFE:
        set_arm_output(false);
        set_fire_output(false);
        if (fire_req) {
            _state = State::ERROR;   // fire before arm
        } else if (arm_req) {
            _state = State::ARMED;
        }
        break;

    case State::ARMED:
        set_arm_output(true);
        set_fire_output(false);
        if (!arm_req) {
            _state = State::SAFE;
        } else if (fire_req) {
            _state = State::FIRED;
        }
        break;

    case State::FIRED:
        set_arm_output(true);
        set_fire_output(true);
        if (!arm_req && !fire_req) {
            _state = State::SAFE;
        }
        break;

    case State::ERROR:
        set_arm_output(false);
        set_fire_output(false);
        if (!arm_req && !fire_req) {
            _state = State::SAFE;
        }
        break;
    }
}

namespace AP {
    AP_ESAD &esad()
    {
        return *AP_ESAD::get_singleton();
    }
};

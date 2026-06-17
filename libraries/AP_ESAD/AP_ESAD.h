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
#pragma once

// AP_ESAD - Electronic Safe-Arm Device sequencing + interlock.
//
// Both the arm (R1) and fire (R2) relays are driven solely by the firmware.
// The operator provides an arm request (ESAD_ARM aux) and a fire request
// (ESAD_FIRE aux); the firmware energizes the relays only when every rule is met:
//
//   1. The vehicle must be armed. Arming starts the watch; while armed the baro
//      is monitored for liftoff (climb of ESAD_LIFT_ALT metres above the arm
//      altitude). A one-shot ESAD_TIMER countdown then starts at liftoff.
//      Disarmed, nothing is monitored and the device is fully reset.
//   2. Until that countdown completes BOTH relays are locked off. Any arm or
//      fire request during the lockout latches an ERROR.
//   3. After the countdown the normal sequence applies: arm request -> R1,
//      then fire request -> R2. Fire requested before arm latches an ERROR.
//
// ERROR is latched and clears only when both requests return low. Disarming
// fully resets the device (one countdown per flight).

#include <AP_Param/AP_Param.h>
#include <stdint.h>

class AP_ESAD {
public:
    AP_ESAD();
    CLASS_NO_COPY(AP_ESAD);

    static AP_ESAD *get_singleton() { return _singleton; }

    // periodic update - runs the sequencing state machine and drives the relays
    void update();

    // operator request inputs (from the ESAD_ARM / ESAD_FIRE RC aux functions)
    void set_arm_request(bool req) { _arm_request = req; }
    void set_fire_request(bool req) { _fire_request = req; }

    bool enabled() const { return _enable != 0; }
    // when false, the firmware skips the post-liftoff countdown only; the
    // arm-then-fire ordering interlock and out-of-sequence ERROR latch still
    // apply, and the OSD element is still shown
    bool safety_enabled() const { return _safety != 0; }

    enum class State : uint8_t {
        LOCKED = 0, // armed, pre-liftoff or counting down - relays inhibited
        SAFE,       // unlocked, awaiting arm request
        ARMED,      // arm relay energized
        FIRED,      // fire relay energized
        ERROR,      // out-of-sequence / premature request latched
    };
    State get_state() const { return _state; }

    // countdown is meaningful (and shown) from liftoff until it completes
    bool countdown_active() const { return _lifted_off && !_timer_done; }
    uint16_t countdown_remaining_s() const;

    // OSD overlay configuration (single combined element, e.g. "E:A 60s")
    bool osd_enabled() const { return _osd_en != 0; }
    uint8_t osd_x() const { return (uint8_t)_osd_x; }
    uint8_t osd_y() const { return (uint8_t)_osd_y; }

    static const struct AP_Param::GroupInfo var_info[];

private:
    static AP_ESAD *_singleton;

    // parameters
    AP_Int8  _enable;
    AP_Int8  _safety;       // enforce order/countdown + show OSD (1) or passthrough (0)
    AP_Int8  _arm_relay;    // relay instance driven as the ARM output
    AP_Int8  _fire_relay;   // relay instance driven as the FIRE output
    AP_Int16 _timer_s;      // post-liftoff arming countdown, seconds
    AP_Float _lift_alt;     // liftoff detection height above arming altitude, metres
    AP_Int8  _osd_en;
    AP_Int8  _osd_x;
    AP_Int8  _osd_y;

    // runtime state (initialized so requests can never read garbage-true at boot)
    State    _state = State::LOCKED;
    bool     _arm_request = false;
    bool     _fire_request = false;
    bool     _armed_prev = false;     // edge-detect the arm transition
    bool     _lifted_off = false;     // one-shot per flight; only watched while armed
    bool     _timer_done = false;
    uint32_t _liftoff_ms = 0;
    float    _ref_alt = 0.0f;         // baro altitude captured at arming (ground ref)

    void set_arm_output(bool on);
    void set_fire_output(bool on);
    void set_relay(uint8_t instance, bool on);

    // arm-then-fire sequencer + out-of-sequence ERROR latch. Drives the relays
    // from _state and the two requests. Enforced in BOTH safety modes: fire is
    // never energized before arm, and a fire-before-arm request latches ERROR
    // (so neither relay activates) until both requests return low.
    void run_sequence(bool arm_req, bool fire_req);
};

namespace AP {
    AP_ESAD &esad();
};

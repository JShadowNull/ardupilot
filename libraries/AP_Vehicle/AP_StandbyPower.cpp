#include "AP_StandbyPower.h"

#if AP_STANDBY_POWER_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Math/crc.h>
#include <stdio.h>

#ifndef HAL_BATT_VOLT_PIN
#error "AP_STANDBY_POWER_ENABLED requires HAL_BATT_VOLT_PIN"
#endif

#ifndef HAL_BATT_VOLT_SCALE
#define HAL_BATT_VOLT_SCALE 11.0
#endif

// CRSF RC frame constants (see AP_RCProtocol_CRSF)
#define STBY_CRSF_BAUDRATE          416666U
#define STBY_CRSF_FRAMETYPE_RC      0x16
#define STBY_CRSF_RC_FRAME_LEN      24      // type + 22 payload + crc

// how long the wake channel must be held high before switching on
#define STBY_WAKE_HOLD_MS           500
// RC frame freshness requirement
#define STBY_FRAME_TIMEOUT_MS       500
// how long to wait for battery voltage after switching on before
// releasing the relay again
#define STBY_BATT_TIMEOUT_MS        10000

extern const AP_HAL::HAL &hal;

const AP_Param::GroupInfo AP_StandbyPower::var_info[] = {

    // @Param: _EN
    // @DisplayName: Standby power mode enable
    // @Description: If enabled and no main battery is detected at boot (USB power only), the autopilot halts initialisation in a low-power loop listening to the RC receiver. Holding the wake channel high drives the wake relay to connect the main battery, after which boot continues normally.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO_FLAGS("_EN", 1, AP_StandbyPower, enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: _CHAN
    // @DisplayName: Standby wake RC channel
    // @Description: RC input channel which wakes the vehicle from standby
    // @Range: 1 16
    // @User: Advanced
    AP_GROUPINFO("_CHAN", 2, AP_StandbyPower, channel, 7),

    // @Param: _TRIG
    // @DisplayName: Standby wake trigger PWM
    // @Description: Wake channel value above which the wake relay is driven
    // @Units: PWM
    // @Range: 1000 2000
    // @User: Advanced
    AP_GROUPINFO("_TRIG", 3, AP_StandbyPower, trigger_us, 1800),

    // @Param: _RELAY
    // @DisplayName: Standby wake relay instance
    // @Description: Relay instance (first relay is 1) whose pin drives the main battery switch
    // @Range: 1 6
    // @User: Advanced
    AP_GROUPINFO("_RELAY", 4, AP_StandbyPower, relay_instance, 1),

    // @Param: _UART
    // @DisplayName: Standby RC serial port
    // @Description: Serial port number carrying CRSF RC input, listened to while in standby
    // @Range: 0 7
    // @User: Advanced
    AP_GROUPINFO("_UART", 5, AP_StandbyPower, uart_num, 2),

    // @Param: _BATT_V
    // @DisplayName: Standby battery present voltage
    // @Description: Battery voltage above which the main battery is considered present
    // @Units: V
    // @User: Advanced
    AP_GROUPINFO("_BATT_V", 6, AP_StandbyPower, batt_volt, 7),

    AP_GROUPEND
};

AP_StandbyPower::AP_StandbyPower()
{
    AP_Param::setup_object_defaults(this, var_info);
}

bool AP_StandbyPower::battery_present(void)
{
    if (batt_source == nullptr) {
        return false;
    }
    return (batt_source->voltage_average() * HAL_BATT_VOLT_SCALE) >= batt_volt.get();
}

// look up the configured relay instance's pin and inversion from the
// already-loaded parameters, without initialising the relay library
bool AP_StandbyPower::resolve_relay_pin(int16_t &pin, bool &inverted) const
{
    char name[17];
    enum ap_var_type ptype;

    hal.util->snprintf(name, sizeof(name), "RELAY%u_PIN", (unsigned)relay_instance.get());
    AP_Int16 *pin_p = (AP_Int16 *)AP_Param::find(name, &ptype);
    if (pin_p == nullptr || ptype != AP_PARAM_INT16) {
        return false;
    }
    pin = pin_p->get();
    if (pin < 0) {
        return false;
    }

    inverted = false;
    hal.util->snprintf(name, sizeof(name), "RELAY%u_INVERTED", (unsigned)relay_instance.get());
    AP_Int8 *inv_p = (AP_Int8 *)AP_Param::find(name, &ptype);
    if (inv_p != nullptr && ptype == AP_PARAM_INT8) {
        inverted = inv_p->get() > 0;
    }
    return true;
}

void AP_StandbyPower::set_relay(int16_t pin, bool inverted, bool on) const
{
    hal.gpio->pinMode(pin, HAL_GPIO_OUTPUT);
    hal.gpio->write(pin, on != inverted);
}

// put the MAX7456/AT7456E OSD chip in its lowest-power state while in
// standby: software reset (leaves OSD disabled) then disable the input
// video buffer. AP_OSD_MAX7456::init() fully resets and reconfigures
// the chip when boot resumes, so this needs no undo.
void AP_StandbyPower::disable_osd(void) const
{
    auto dev = hal.spi->get_device("osd");
    if (!dev) {
        return;
    }
    dev->get_semaphore()->take_blocking();
    dev->write_register(0x00, 0x02);   // VM0: software reset
    hal.scheduler->delay(1);
    dev->write_register(0x00, 0x01);   // VM0: OSD off, video buffer disabled
    dev->get_semaphore()->give();
}

// decode a 22 byte CRSF RC payload: 16 channels of 11 bits.
// scale factors defined by TBS - us = (x - 992) * 5 / 8 + 1500
void AP_StandbyPower::decode_channels(const uint8_t *payload)
{
    uint32_t bits = 0;
    uint8_t bitcount = 0;
    uint8_t ch = 0;
    for (uint8_t i = 0; i < 22 && ch < 16; i++) {
        bits |= (uint32_t)payload[i] << bitcount;
        bitcount += 8;
        while (bitcount >= 11 && ch < 16) {
            const uint16_t raw = bits & 0x7FF;
            channels_us[ch++] = (uint16_t)((raw * 5U) / 8U + 880U);
            bits >>= 11;
            bitcount -= 11;
        }
    }
    last_frame_ms = AP_HAL::millis();
}

// feed one byte into a minimal CRSF frame parser. Returns true when a
// valid RC channels frame has been decoded.
bool AP_StandbyPower::parse_byte(uint8_t b)
{
    if (buf_len >= sizeof(buf)) {
        buf_len = 0;
    }
    buf[buf_len++] = b;

    bool got_rc_frame = false;
    while (buf_len >= 2) {
        // frame: [address][length][type][payload...][crc]
        // length counts type+payload+crc
        const uint8_t flen = buf[1];
        if (flen < 3 || flen > 62) {
            // not a plausible frame start, resync by one byte
            memmove(&buf[0], &buf[1], --buf_len);
            continue;
        }
        const uint16_t total = (uint16_t)flen + 2;
        if (buf_len < total) {
            // frame incomplete
            break;
        }
        if (crc8_dvb_s2_update(0, &buf[2], flen - 1) == buf[total - 1]) {
            if (buf[2] == STBY_CRSF_FRAMETYPE_RC && flen == STBY_CRSF_RC_FRAME_LEN) {
                decode_channels(&buf[3]);
                got_rc_frame = true;
            }
            buf_len -= total;
            memmove(&buf[0], &buf[total], buf_len);
        } else {
            memmove(&buf[0], &buf[1], --buf_len);
        }
    }
    return got_rc_frame;
}

void AP_StandbyPower::check(void)
{
    if (enable.get() != 1) {
        return;
    }

    batt_source = hal.analogin->channel(HAL_BATT_VOLT_PIN);
    if (batt_source == nullptr) {
        return;
    }
    // let the ADC accumulate some samples
    hal.scheduler->delay(50);

    if (battery_present()) {
        // main battery attached, boot normally
        return;
    }
    if (hal.gpio->usb_connected()) {
        // attached to a USB host for configuration, boot normally
        return;
    }

    int16_t relay_pin;
    bool relay_inverted;
    if (!resolve_relay_pin(relay_pin, relay_inverted)) {
        return;
    }

    const uint8_t chan_idx = constrain_int16(channel.get(), 1, 16) - 1;

    AP_HAL::UARTDriver *uart = hal.serial(uart_num.get());
    if (uart == nullptr) {
        return;
    }
    uart->begin(STBY_CRSF_BAUDRATE);

    ::printf("StandbyPower: entering standby\n");

    disable_osd();

    uint32_t high_start_ms = 0;   // when the wake channel first went high
    uint32_t relay_on_ms = 0;     // when we switched the relay on
    bool holding = false;         // relay currently driven on
    bool wait_for_release = false; // require channel low before re-arming
    uint32_t last_blink_ms = 0;

    while (true) {
        hal.scheduler->delay(2);

        // drain the uart through the CRSF parser
        uint32_t avail = uart->available();
        while (avail--) {
            const int16_t b = uart->read();
            if (b < 0) {
                break;
            }
            parse_byte((uint8_t)b);
        }

        const uint32_t now_ms = AP_HAL::millis();
        const bool frame_fresh = last_frame_ms != 0 && (now_ms - last_frame_ms) < STBY_FRAME_TIMEOUT_MS;
        const bool chan_high = frame_fresh && channels_us[chan_idx] >= trigger_us.get();

        if (!holding) {
            if (battery_present()) {
                // battery plugged in manually, boot normally
                break;
            }
            if (wait_for_release) {
                if (frame_fresh && !chan_high) {
                    wait_for_release = false;
                }
            } else if (chan_high) {
                if (high_start_ms == 0) {
                    high_start_ms = now_ms;
                } else if (now_ms - high_start_ms >= STBY_WAKE_HOLD_MS) {
                    // wake: connect the main battery
                    set_relay(relay_pin, relay_inverted, true);
                    holding = true;
                    relay_on_ms = now_ms;
                }
            } else {
                high_start_ms = 0;
            }
        } else {
            if (battery_present()) {
                // battery confirmed - continue the normal boot with the
                // relay pin still held high. AP_Relay must be configured
                // with RELAYx_DEFAULT=2 (no change) so init does not
                // glitch the pin.
                ::printf("StandbyPower: battery on, resuming boot\n");
                break;
            }
            if ((frame_fresh && !chan_high) ||
                (now_ms - relay_on_ms >= STBY_BATT_TIMEOUT_MS)) {
                // user released the switch, or the battery never
                // appeared - release and re-arm
                set_relay(relay_pin, relay_inverted, false);
                holding = false;
                high_start_ms = 0;
                wait_for_release = true;
            }
        }

        if (!holding && hal.gpio->usb_connected()) {
            // USB host attached mid-standby, boot normally for configuration
            break;
        }

#ifdef HAL_GPIO_A_LED_PIN
        // short blink every 2s to indicate standby
        if (now_ms - last_blink_ms >= 2000) {
            last_blink_ms = now_ms;
            hal.gpio->pinMode(HAL_GPIO_A_LED_PIN, HAL_GPIO_OUTPUT);
            hal.gpio->write(HAL_GPIO_A_LED_PIN, 1);
            hal.scheduler->delay(30);
            hal.gpio->write(HAL_GPIO_A_LED_PIN, 0);
        }
#endif
    }
}

#endif // AP_STANDBY_POWER_ENABLED

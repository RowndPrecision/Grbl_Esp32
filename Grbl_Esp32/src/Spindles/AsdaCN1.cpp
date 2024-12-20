
#include "AsdaCN1.h"

// ===================================== AsdaCN1 ==============================================

namespace Spindles {

    // Get the GPIO from the machine definition
    void AsdaCN1::get_pins_and_settings() {
        // setup all the pins

#ifdef ASDA_CN1_OUTPUT_PIN
        _output_pin = ASDA_CN1_OUTPUT_PIN;
#else
        _output_pin = UNDEFINED_PIN;
#endif

        _invert_pwm = spindle_output_invert->get();

#ifdef ASDA_CN1_S_P_PIN
        digitalWrite(ASDA_CN1_S_P_PIN, true);
        grbl_msg_sendf(CLIENT_ALL, MsgLevel::Info, "cn1 SP on");
#else
        grbl_msg_sendf(CLIENT_ALL, MsgLevel::Info, "Warning: ASDA_CN1_S_P_PIN not defined");
        return;  // We cannot continue without the output pin
#endif

#ifdef ASDA_CN1_ENABLE_PIN
        _enable_pin = ASDA_CN1_ENABLE_PIN;
#else
        _enable_pin = UNDEFINED_PIN;
#endif

#ifdef ASDA_CN1_DIR_PIN
        _direction_pin = ASDA_CN1_DIR_PIN;
#else
        _direction_pin = UNDEFINED_PIN;
#endif

        if (_output_pin == UNDEFINED_PIN) {
            grbl_msg_sendf(CLIENT_ALL, MsgLevel::Info, "Warning: ASDA_CN1_OUTPUT_PIN not defined");
            return;  // We cannot continue without the output pin
        }

        _off_with_zero_speed = spindle_enbl_off_with_zero_speed->get();

        is_reversable = (_direction_pin != UNDEFINED_PIN);

        _pwm_freq      = spindle_pwm_freq->get();
        _pwm_precision = calc_pwm_precision(_pwm_freq);  // detewrmine the best precision
        _pwm_period    = (1 << _pwm_precision);

        if (spindle_pwm_min_value->get() > spindle_pwm_min_value->get()) {
            grbl_msg_sendf(CLIENT_ALL, MsgLevel::Info, "Warning: Spindle min pwm is greater than max. Check $35 and $36");
        }

        // pre-caculate some PWM count values
        _pwm_off_value = (_pwm_period * spindle_pwm_off_value->get() / 100.0);
        _pwm_min_value = (_pwm_period * spindle_pwm_min_value->get() / 100.0);
        _pwm_max_value = (_pwm_period * spindle_pwm_max_value->get() / 100.0);

#ifdef ENABLE_PIECEWISE_LINEAR_SPINDLE
        _min_rpm          = RPM_MIN;
        _max_rpm          = RPM_MAX;
        _piecewide_linear = true;
#else
        _min_rpm          = rpm_min->get();
        _max_rpm          = rpm_max_chuck->get();
        _piecewide_linear = false;
#endif

        _pwm_chan_num = 0;  // Channel 0 is reserved for spindle use

        _spinup_delay   = spindle_delay_spinup->get() * 1000.0;
        _spindown_delay = spindle_delay_spindown->get() * 1000.0;
    }
    /*
    uint32_t AsdaCN1::set_rpm(uint32_t rpm_target) {
        uint32_t pwm_value;
        uint32_t rpm_current = 0;  // Initialize to ensure it's defined
        uint32_t rpm_init;

        if (_output_pin == UNDEFINED_PIN) {
            return rpm_target;
        }

        // Apply override
        rpm_target = rpm_target * sys.spindle_speed_ovr / 100;  // Scale by spindle speed override value (uint8_t percent)

        // Apply limits
        if ((_min_rpm >= _max_rpm) || (rpm_target >= _max_rpm)) {
            rpm_target = _max_rpm;
        } else if (rpm_target != 0 && rpm_target <= _min_rpm) {
            rpm_target = _min_rpm;
        }

        rpm_init    = sys.spindle_speed;
        rpm_current = rpm_init;

        int32_t cnt_spin;
        int32_t rpm_diff = rpm_target - rpm_init;

        // Determine the number of steps based on the dynamic ramp time
        if (rpm_diff > 0) {
            cnt_spin = map_uint32_t(rpm_diff, 0, _max_rpm - _min_rpm, 0, _spinup_delay);
        } else {
            cnt_spin = map_uint32_t(-rpm_diff, 0, _max_rpm - _min_rpm, 0, _spindown_delay);
        }

        float rpm_step = (cnt_spin > 0) ? ((float)rpm_diff / (float)cnt_spin) : 0.0;

        for (uint32_t cnt = 0; cnt < cnt_spin; cnt++) {
            if ((rpm_step > 0 && rpm_current >= rpm_target) || (rpm_step < 0 && rpm_current <= rpm_target) || (rpm_step == 0)) {
                break;  // Stop if target RPM is reached
            }

            rpm_current = rpm_init + (uint32_t)((float)cnt * rpm_step);
            if (rpm_current < 0)
                rpm_current = 0;  // Prevent negative RPM

            // Set PWM value based on current RPM
            sys.spindle_speed = rpm_current;

            pwm_value = (rpm_current == 0) ? _pwm_off_value : map_uint32_t(rpm_current, _min_rpm, _max_rpm, _pwm_min_value, _pwm_max_value);

            set_output(pwm_value);

            delay(1);  // Short delay for smooth ramping
        }

        // Final update to ensure target RPM is set
        sys.spindle_speed = rpm_target;

        pwm_value = (rpm_target == 0) ? _pwm_off_value : map_uint32_t(rpm_target, _min_rpm, _max_rpm, _pwm_min_value, _pwm_max_value);

        set_output(pwm_value);

        return 0;
    }
*/
    void AsdaCN1::set_state(SpindleState state, uint32_t rpm) {
        if (sys.abort) {
            return;  // Block during abort.
        }

        if (state == SpindleState::Disable) {  // Halt or set spindle direction and rpm.
            stop();
        } else {
            if (get_state() == SpindleState::Disable) {
                set_enable_pin(true);
                set_dir_pin(state == SpindleState::Cw);
            } else if (state != get_state()) {
                set_enable_pin(true);
                set_rpm(0);
            }

            set_dir_pin(state == SpindleState::Cw);
            set_rpm(rpm);
            set_enable_pin(state != SpindleState::Disable);  // must be done after setting rpm for enable features to work
        }

        _current_state = state;

        sys.report_ovr_counter = 0;  // Set to report change immediately
    }

    void AsdaCN1::stop() {
        // inverts are delt with in methods
        set_rpm(0);
        set_output(_pwm_off_value);
        set_enable_pin(false);
    }

    void AsdaCN1::set_dir_pin(bool Clockwise) {
        if (chuck_direction_invert->get())
            Clockwise = !Clockwise;
        digitalWrite(_direction_pin, Clockwise);
    }

    void AsdaCN1::deinit() {
        stop();
#ifdef ASDA_CN1_OUTPUT_PIN
        gpio_reset_pin(ASDA_CN1_OUTPUT_PIN);
        pinMode(ASDA_CN1_OUTPUT_PIN, OUTPUT);
        digitalWrite(ASDA_CN1_OUTPUT_PIN, 0);
#endif

#ifdef ASDA_CN1_ENABLE_PIN
        _enable_pin = ASDA_CN1_ENABLE_PIN;
#endif

#ifdef ASDA_CN1_S_P_PIN
        digitalWrite(ASDA_CN1_S_P_PIN, false);
        grbl_msg_sendf(CLIENT_ALL, MsgLevel::Info, "cn1 SP off");
#else
        grbl_msg_sendf(CLIENT_ALL, MsgLevel::Info, "Warning: ASDA_CN1_S_P_PIN not defined");
        return;  // We cannot continue without the output pin
#endif
    }

}

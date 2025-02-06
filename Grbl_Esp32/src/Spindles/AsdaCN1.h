#pragma once

#include "PWMSpindle.h"

namespace Spindles {
    // this is the same as a PWM spindle but the M4 compensation is supported.
    class AsdaCN1 : public PWM {
    public:
        AsdaCN1() = default;

        AsdaCN1(const AsdaCN1&)            = delete;
        AsdaCN1(AsdaCN1&&)                 = delete;
        AsdaCN1& operator=(const AsdaCN1&) = delete;
        AsdaCN1& operator=(AsdaCN1&&)      = delete;

        // virtual uint32_t set_rpm(uint32_t rpm) override;
        void set_state(SpindleState state, uint32_t rpm) override;
        void stop() override;

        void deinit() override;
        void get_pins_and_settings() override;

        virtual ~AsdaCN1() {}

    protected:
        virtual void set_dir_pin(bool Clockwise);
        virtual void set_enable_pin(bool enable_pin);
    };
}

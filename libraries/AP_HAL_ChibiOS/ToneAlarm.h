#pragma once

#include "AP_HAL_ChibiOS.h"

#include "ch.h"
#include "hal.h"
#include "MMLPlayer.h"

#ifdef HAL_PWM_ALARM

namespace ChibiOS {

class ToneAlarm : public MMLPlayer {
public:
    bool init();

protected:
    // Play a tone on the buzzer. If frequency or volume is zero, stop playing a tone
    // on the buzzer. Otherwise, volume may be ignored if unsupported.
    virtual void setBuzzerTone(float frequency, float volume) override;
    virtual uint32_t micros() override { return AP_HAL::micros(); }

private:
    struct pwmGroup {
        pwmchannel_t chan;
        PWMConfig pwm_cfg;
        PWMDriver* pwm_drv;
    };
    static pwmGroup pwm_group;
};

}
#endif // HAL_PWM_ALARM

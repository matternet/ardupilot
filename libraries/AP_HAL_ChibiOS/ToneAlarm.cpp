#include <AP_HAL/AP_HAL.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_Math/AP_Math.h>

#ifdef HAL_PWM_ALARM

#include "ToneAlarm.h"

using namespace ChibiOS;

struct ToneAlarm::pwmGroup ToneAlarm::pwm_group = HAL_PWM_ALARM;

extern const AP_HAL::HAL& hal;

bool ToneAlarm::init()
{
    // start PWM driver
    pwm_group.pwm_cfg.period = 1000;
    pwmStart(pwm_group.pwm_drv, &pwm_group.pwm_cfg);

    return true;
}

void ToneAlarm::setBuzzerTone(float frequency, float volume) {
    if (is_zero(frequency) || is_zero(volume)) {
        pwmDisableChannel(pwm_group.pwm_drv, pwm_group.chan);
    } else {
        pwmChangePeriod(pwm_group.pwm_drv,
                        roundf(pwm_group.pwm_cfg.frequency/frequency));

        pwmEnableChannel(pwm_group.pwm_drv, pwm_group.chan, roundf(volume*pwm_group.pwm_cfg.frequency/frequency)/2);
    }
}
#endif

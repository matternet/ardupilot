#pragma once

#include "AP_BattMonitor_Analog.h"

class AP_BattMonitor_Analog_Table : public AP_BattMonitor_Analog
{
public:
    /// Constructor
    AP_BattMonitor_Analog_Table(AP_BattMonitor &mon, AP_BattMonitor::BattMonitor_State &mon_state, AP_BattMonitor_Params &params) :
        AP_BattMonitor_Analog(mon, mon_state, params) {}


    /// Read the battery voltage and current.  Should be called at 10hz
    void read() override;

    /// returns true if battery monitor provides current info
    bool has_current() const override { return true; }

    // lookup SoC in table, returning SoC as a percentage
    static float lookup_SoC_table(float voltage);
    
private:
    uint32_t last_armed_ms;
    bool using_table = true;
};

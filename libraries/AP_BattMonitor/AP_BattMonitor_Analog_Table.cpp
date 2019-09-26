#include <AP_HAL/AP_HAL.h>
#include "AP_BattMonitor_Analog_Table.h"

extern const AP_HAL::HAL& hal;

// assume 12 cells
#define TABLE_NUM_CELLS 12

/*
  state of charge table for a battery. This should be chosen based on
  a battery that is at maximum age of batteries that will be used in
  the vehicle
 */
static const struct {
    float volt_per_cell;
    float soc_pct;
} soc_table[] = {
    { 4.173, 100 },
    { 4.112, 96.15 },
    { 4.085, 92.31 },
    { 4.071, 88.46 },
    { 4.039, 84.62 },
    { 3.987, 80.77 },
    { 3.943, 76.92 },
    { 3.908, 73.08 },
    { 3.887, 69.23 },
    { 3.854, 65.38 },
    { 3.833, 61.54 },
    { 3.801, 57.69 },
    { 3.783, 53.85 },
    { 3.742, 50 },
    { 3.715, 46.15 },
    { 3.679, 42.31 },
    { 3.636, 38.46 },
    { 3.588, 34.62 },
    { 3.543, 30.77 },
    { 3.503, 26.92 },
    { 3.462, 23.08 },
    { 3.379, 19.23 },
    { 3.296, 15.38 },
    { 3.218, 11.54 },
    { 3.165, 7.69 },
    { 3.091, 3.85 },
    { 2.977, 0 }};

/*
  lookup SoC in table, returning SoC as a percentage
 */
float AP_BattMonitor_Analog_Table::lookup_SoC_table(float voltage)
{
    float per_cell = voltage / TABLE_NUM_CELLS;

    if (per_cell >= soc_table[0].volt_per_cell) {
        return 100.0;
    }

    for (uint8_t i=1; i<ARRAY_SIZE(soc_table); i++) {
        if (per_cell >= soc_table[i].volt_per_cell) {
            // linear interpolation between table rows
            float dv1 = per_cell - soc_table[i].volt_per_cell;
            float dv2 = soc_table[i-1].volt_per_cell - soc_table[i].volt_per_cell;
            float soc1 = soc_table[i].soc_pct;
            float soc2 = soc_table[i-1].soc_pct;
            return soc1 + (dv1 / dv2) * (soc2 - soc1);
        }
    }
    // off the bottom of the table
    return 0;
}

/*
  read - read the voltage and current
  Use SoC table when disarmed
*/
void AP_BattMonitor_Analog_Table::read()
{
    // first call base class read() method
    AP_BattMonitor_Analog::read();

    uint32_t now = AP_HAL::millis();
    bool armed = hal.util->get_soft_armed();
    if (armed) {
        last_armed_ms = now;
        using_table = false;
    }

    // if we have been disarmed for more than 2 minutes then we start using the table
    if (now - last_armed_ms > 120000) {
        using_table = true;
    }

    if (!armed && _state.voltage < 2.0*TABLE_NUM_CELLS) {
        // below 2.0v/cell we assume battery is removed, force use of
        // table when new battery is inserted. The threshold is chosen
        // to make it more likely that a floating voltage pin will
        // detect a low enough voltage to trigger table use
        using_table = true;
    }

    if (using_table) {
        // reset remaining capacity based on the table
        float soc_pct = lookup_SoC_table(_state.voltage);
        reset_remaining(soc_pct);
    }
}

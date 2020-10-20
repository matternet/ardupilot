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
/*
  multicopter simulator class
*/

#pragma once

#include "SIM_Aircraft.h"
#include "SIM_Motor.h"

namespace SITL {

/*
  class to describe a multicopter frame type
 */
class Frame {
public:
    const char *name;
    uint8_t num_motors;
    Motor *motors;

    Frame(const char *_name,
          uint8_t _num_motors,
          Motor *_motors) :
        name(_name),
        num_motors(_num_motors),
        motors(_motors) {}


    // find a frame by name
    static Frame *find_frame(const char *name);
    
    // initialise frame
    void init(const char *frame_str);
    
    // calculate rotational and linear accelerations
    void calculate_forces(const Aircraft &aircraft,
                          const struct sitl_input &input,
                          Vector3f &rot_accel, Vector3f &body_accel, float* rpm,
                          bool use_drag=true);

    float terminal_velocity;
    float terminal_rotation_rate;
    uint8_t motor_offset;

    // calculate current and voltage
    void current_and_voltage(const struct sitl_input &input, float &voltage, float &current);

    // get mass in kg
    float get_mass(void) const {
        return mass;
    }

private:
    /*
      parameters that define the multicopter model. Can be loaded from
      a json file to give a custom model
     */
    const struct Model {
        float mass = 1.5;
        float diagonal_size = 0.35;
        float refSpd = 7.4;
        float refAngle = 20;
        float refVoltage = 12.14;
        float refCurrent = 25.37;
        float refAlt = 618;
        float refTempC = 25;
        float maxVoltage = 4.1*3;
        float hovCurr = 22.7;
        float hoverThrOut = 0.4;
        float refBatRes = 0.006;
        float propExpo = 0;
        float refRotRate = 120;
        float pwmMin = 1100;
        float pwmMax = 1900;
        float spin_min = 0.0;
        float spin_max = 1.0;
        float slew_max = 0;
    } default_model;
    struct Model model;

    // exposed area times coefficient of drag
    float areaCd;
    float mass;
    float velocity_max;
    float effective_prop_area;

    // get air density in kg/m^3
    float get_air_density(float alt_amsl) const;

    // load frame parameters from a json model file
    void load_frame_params(const char *model_json);

};
}

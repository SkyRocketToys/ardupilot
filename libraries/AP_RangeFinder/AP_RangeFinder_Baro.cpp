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
#include "AP_RangeFinder_Baro.h"

#include <utility>
#include <stdio.h>

#include <AP_Param/AP_Param.h>

extern const AP_HAL::HAL& hal;

/*
   constructor 
*/
AP_RangeFinder_Baro::AP_RangeFinder_Baro(RangeFinder &_ranger, uint8_t instance,
                                         RangeFinder::RangeFinder_State &_state)
    : AP_RangeFinder_Backend(_ranger, instance, _state, MAV_DISTANCE_SENSOR_UNKNOWN)
{
}

/*
   detect if a baro backend is available
*/
AP_RangeFinder_Backend *AP_RangeFinder_Baro::detect(RangeFinder &_ranger, uint8_t instance,
                                                    RangeFinder::RangeFinder_State &_state)
{
    AP_RangeFinder_Baro *sensor = new AP_RangeFinder_Baro(_ranger, instance, _state);
    if (!sensor ||
        !sensor->init()) {
        delete sensor;
        return nullptr;
    }
    return sensor;
}

/*
  initialise the sensor to required settings
 */
bool AP_RangeFinder_Baro::init(void)
{
    ahrs = (AP_AHRS *)AP_Param::find_object("AHRS_");
    if (ahrs == nullptr) {
        return false;
    }
    return true;
}

/*
  update range
 */
void AP_RangeFinder_Baro::update(void)
{
    // adjust zero when disarmed, assuming flat surface
    float baro_alt = ahrs->get_baro().get_altitude();
    if (!hal.util->get_soft_armed()) {
        base_alt = 0.97 * base_alt + 0.03 * baro_alt;
    }
    baro_alt -= base_alt;
    float scale = ahrs->get_rotation_body_to_ned().c.z;
    if (scale < 0.2) {
        set_status(RangeFinder::RangeFinder_NoData);
    } else {
        state.distance_cm = baro_alt * 100 / scale;
        update_status();
    }
}

#pragma once

#include "RangeFinder.h"
#include "RangeFinder_Backend.h"
#include <AP_AHRS/AP_AHRS.h>

/*
  simulate a rangefinder with a barometer
 */

class AP_RangeFinder_Baro : public AP_RangeFinder_Backend
{

public:
    // static detection function
    static AP_RangeFinder_Backend *detect(RangeFinder::RangeFinder_State &_state);

    // update state
    void update(void) override;


private:
    // constructor
    AP_RangeFinder_Baro(RangeFinder::RangeFinder_State &_state);

    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_UNKNOWN;
    }
    
    bool init(void);

    AP_AHRS *ahrs;
    float base_alt;
};

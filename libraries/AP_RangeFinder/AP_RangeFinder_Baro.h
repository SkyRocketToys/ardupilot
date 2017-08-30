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
    static AP_RangeFinder_Backend *detect(RangeFinder &ranger, uint8_t instance,
                                          RangeFinder::RangeFinder_State &_state);

    // update state
    void update(void) override;


private:
    // constructor
    AP_RangeFinder_Baro(RangeFinder &ranger, uint8_t instance,
                        RangeFinder::RangeFinder_State &_state);

    bool init(void);

    AP_AHRS *ahrs;
    float base_alt;
};

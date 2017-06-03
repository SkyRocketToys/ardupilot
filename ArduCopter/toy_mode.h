#pragma once

/*
  class to support "toy" mode for simplified user interaction for
  large volume consumer vehicles
 */

class ToyMode
{
public:
    friend class Copter;

    void update(void);

    static const struct AP_Param::GroupInfo var_info[];
    
private:
    AP_Int8 enable;
    
    int32_t ch6_counter;
    uint32_t power_counter;
    uint32_t throttle_low_counter;
    bool last_gps_enable;
};

#pragma once

/*
  class to support "toy" mode for simplified user interaction for
  large volume consumer vehicles
 */

class ToyMode
{
public:
    friend class Copter;

    ToyMode();
    void update(void);

    static const struct AP_Param::GroupInfo var_info[];
    
private:
    bool first_update;
    AP_Int8 enable;
    AP_Int8 primary_mode1;
    AP_Int8 primary_mode2;
    
    int32_t ch6_counter;
    uint32_t power_counter;
    uint32_t throttle_low_counter;
    bool last_secondary_mode;
};

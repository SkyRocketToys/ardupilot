#pragma once

#include <AP_HAL/AP_HAL.h>

#include <AP_HAL_Empty/AP_HAL_Empty_Namespace.h>

class HAL_ChibiOS : public AP_HAL::HAL {
public:
    HAL_ChibiOS();
    void run(int argc, char* const* argv, Callbacks* callbacks) const override;
};

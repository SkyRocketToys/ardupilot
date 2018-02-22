/*
 * These are stubs for functions found on AP_NavEKF2_MagFusion.cpp
 */

#include <AP_NavEKF2/AP_NavEKF2_core.h>

#ifdef STUB_FILE_OUT_AP_NavEKF2_MagFusion

void NavEKF2_core::alignMagStateDeclination()
{
}

void NavEKF2_core::realignYawGPS()
{
}

void NavEKF2_core::SelectMagFusion()
{
}

#endif


#ifdef STUB_FILE_OUT_AP_NavEKF2_OptFlowFusion
/*
 * These are stubs for functions found on AP_NavEKF2_OptFlowFusion
 */

void NavEKF2_core::FuseOptFlow()
{
}

void NavEKF2_core::SelectFlowFusion()
{
}

#endif


#ifdef STUB_FILE_OUT_AP_NavEKF2_RngBcnFusion

/*
 * These are stubs for functions found on AP_NavEKF2_RngBcnFusion
 */
void NavEKF2_core::SelectRngBcnFusion()
{
}

#endif


#ifdef STUB_FILE_OUT_AP_NavEKF2_GPSStatus

/*
 * These are stubs for functions found on AP_NavEKF2_GPSStatus
 */
bool NavEKF2_core::calcGpsGoodToAlign(void)
{
    return false;
}

void NavEKF2_core::calcGpsGoodForFlight(void)
{
    gpsAccuracyGood = false;
}

#endif

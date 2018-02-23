#include "AP_NavEKF2_core.h"

#if HAL_CPU_CLASS >= HAL_CPU_CLASS_150

#include <AP_GPS/AP_GPS.h>
#include <GCS_MAVLink/GCS.h>

extern AP_HAL::HAL& hal;

/* Monitor GPS data to see if quality is good enough to initialise the EKF
   Monitor magnetometer innovations to to see if the heading is good enough to use GPS
   Return true if all criteria pass for 10 seconds

   We also record the failure reason so that prearm_failure_reason()
   can give a good report to the user on why arming is failing
*/
bool NavEKF2_core::calcGpsGoodToAlign(void)
{
    const AP_GPS &gps = AP::gps();

    if (inFlight && assume_zero_sideslip() && !use_compass()) {
        // this is a special case where a plane has launched without magnetometer
        // is now in the air and needs to align yaw to the GPS and start navigating as soon as possible
        return true;
    }

    // User defined multiplier to be applied to check thresholds
    float checkScaler = 0.01f*(float)frontend->_gpsCheckScaler;

    // If we have good magnetometer consistency and bad innovations for longer than 5 seconds then we reset heading and field states
    // This enables us to handle large changes to the external magnetic field environment that occur before arming
    if ((magTestRatio.x <= 1.0f && magTestRatio.y <= 1.0f && yawTestRatio <= 1.0f) || !consistentMagData) {
        magYawResetTimer_ms = imuSampleTime_ms;
    }
    if ((imuSampleTime_ms - magYawResetTimer_ms > 5000) && !motorsArmed) {
        // request reset of heading and magnetic field states
        magYawResetRequest = true;
        // reset timer to ensure that bad magnetometer data cannot cause a heading reset more often than every 5 seconds
        magYawResetTimer_ms = imuSampleTime_ms;
    }

    // Check for significant change in GPS position if disarmed which indicates bad GPS
    // This check can only be used when the vehicle is stationary
    const struct Location &gpsloc = gps.location(); // Current location
    const float posFiltTimeConst = 10.0f; // time constant used to decay position drift
    // calculate time lapsesd since last update and limit to prevent numerical errors
    float deltaTime = constrain_float(float(imuDataDelayed.time_ms - lastPreAlignGpsCheckTime_ms)*0.001f,0.01f,posFiltTimeConst);
    lastPreAlignGpsCheckTime_ms = imuDataDelayed.time_ms;
    // Sum distance moved
    gpsDriftNE += location_diff(gpsloc_prev, gpsloc).length();
    gpsloc_prev = gpsloc;
    // Decay distance moved exponentially to zero
    gpsDriftNE *= (1.0f - deltaTime/posFiltTimeConst);
    // Clamp the fiter state to prevent excessive persistence of large transients
    gpsDriftNE = MIN(gpsDriftNE,10.0f);
    // Fail if more than 3 metres drift after filtering whilst on-ground
    // This corresponds to a maximum acceptable average drift rate of 0.3 m/s or single glitch event of 3m
    bool gpsDriftFail = (gpsDriftNE > 3.0f*checkScaler) && onGround && (frontend->_gpsCheck & MASK_GPS_POS_DRIFT);

    // Report check result as a text string and bitmask
    if (gpsDriftFail) {
        hal.util->snprintf(prearm_fail_string,
                           sizeof(prearm_fail_string),
                           "GPS drift %.1fm (needs %.1f)", (double)gpsDriftNE, (double)(3.0f*checkScaler));
        gpsCheckStatus.bad_horiz_drift = true;
    } else {
        gpsCheckStatus.bad_horiz_drift = false;
    }

    // Check that the vertical GPS vertical velocity is reasonable after noise filtering
    bool gpsVertVelFail;
    if (gps.have_vertical_velocity() && onGround) {
        // check that the average vertical GPS velocity is close to zero
        gpsVertVelFilt = 0.1f * gpsDataNew.vel.z + 0.9f * gpsVertVelFilt;
        gpsVertVelFilt = constrain_float(gpsVertVelFilt,-10.0f,10.0f);
        gpsVertVelFail = (fabsf(gpsVertVelFilt) > 0.3f*checkScaler) && (frontend->_gpsCheck & MASK_GPS_VERT_SPD);
    } else if ((frontend->_fusionModeGPS == 0) && !gps.have_vertical_velocity()) {
        // If the EKF settings require vertical GPS velocity and the receiver is not outputting it, then fail
        gpsVertVelFail = true;
        // if we have a 3D fix with no vertical velocity and
        // EK2_GPS_TYPE=0 then change it to 1. It means the GPS is not
        // capable of giving a vertical velocity
        if (gps.status() >= AP_GPS::GPS_OK_FIX_3D) {
            frontend->_fusionModeGPS.set(1);
            gcs().send_text(MAV_SEVERITY_WARNING, "EK2: Changed EK2_GPS_TYPE to 1");
        }
    } else {
        gpsVertVelFail = false;
    }

    // Report check result as a text string and bitmask
    if (gpsVertVelFail) {
        hal.util->snprintf(prearm_fail_string,
                           sizeof(prearm_fail_string),
                           "GPS vertical speed %.2fm/s (needs %.2f)", (double)fabsf(gpsVertVelFilt), (double)(0.3f*checkScaler));
        gpsCheckStatus.bad_vert_vel = true;
    } else {
        gpsCheckStatus.bad_vert_vel = false;
    }

    // Check that the horizontal GPS vertical velocity is reasonable after noise filtering
    // This check can only be used if the vehicle is stationary
    bool gpsHorizVelFail;
    if (onGround) {
        gpsHorizVelFilt = 0.1f * norm(gpsDataDelayed.vel.x,gpsDataDelayed.vel.y) + 0.9f * gpsHorizVelFilt;
        gpsHorizVelFilt = constrain_float(gpsHorizVelFilt,-10.0f,10.0f);
        gpsHorizVelFail = (fabsf(gpsHorizVelFilt) > 0.3f*checkScaler) && (frontend->_gpsCheck & MASK_GPS_HORIZ_SPD);
    } else {
        gpsHorizVelFail = false;
    }

    // Report check result as a text string and bitmask
    if (gpsHorizVelFail) {
        hal.util->snprintf(prearm_fail_string,
                           sizeof(prearm_fail_string),
                           "GPS horizontal speed %.2fm/s (needs %.2f)", (double)gpsDriftNE, (double)(0.3f*checkScaler));
        gpsCheckStatus.bad_horiz_vel = true;
    } else {
        gpsCheckStatus.bad_horiz_vel = false;
    }

    // fail if horiziontal position accuracy not sufficient
    float hAcc = 0.0f;
    bool hAccFail;
    if (gps.horizontal_accuracy(hAcc)) {
        hAccFail = (hAcc > 5.0f*checkScaler)  && (frontend->_gpsCheck & MASK_GPS_POS_ERR);
    } else {
        hAccFail =  false;
    }

    // Report check result as a text string and bitmask
    if (hAccFail) {
        hal.util->snprintf(prearm_fail_string,
                           sizeof(prearm_fail_string),
                           "GPS horiz error %.1fm (needs %.1f)", (double)hAcc, (double)(5.0f*checkScaler));
        gpsCheckStatus.bad_hAcc = true;
    } else {
        gpsCheckStatus.bad_hAcc = false;
    }

    // Check for vertical GPS accuracy
    float vAcc = 0.0f;
    bool vAccFail = false;
    if (gps.vertical_accuracy(vAcc)) {
        vAccFail = (vAcc > 7.5f * checkScaler) && (frontend->_gpsCheck & MASK_GPS_POS_ERR);
    }
    // Report check result as a text string and bitmask
    if (vAccFail) {
        hal.util->snprintf(prearm_fail_string,
                           sizeof(prearm_fail_string),
                           "GPS vert error %.1fm (needs < %.1f)", (double)vAcc, (double)(7.5f * checkScaler));
        gpsCheckStatus.bad_vAcc = true;
    } else {
        gpsCheckStatus.bad_vAcc = false;
    }

    // fail if reported speed accuracy greater than threshold
    bool gpsSpdAccFail = (gpsSpdAccuracy > 1.0f*checkScaler) && (frontend->_gpsCheck & MASK_GPS_SPD_ERR);

    // Report check result as a text string and bitmask
    if (gpsSpdAccFail) {
        hal.util->snprintf(prearm_fail_string,
                           sizeof(prearm_fail_string),
                           "GPS speed error %.1f (needs %.1f)", (double)gpsSpdAccuracy, (double)(1.0f*checkScaler));
        gpsCheckStatus.bad_sAcc = true;
    } else {
        gpsCheckStatus.bad_sAcc = false;
    }

    // fail if satellite geometry is poor
    bool hdopFail = (gps.get_hdop() > 250)  && (frontend->_gpsCheck & MASK_GPS_HDOP);

    // Report check result as a text string and bitmask
    if (hdopFail) {
        hal.util->snprintf(prearm_fail_string, sizeof(prearm_fail_string),
                           "GPS HDOP %.1f (needs 2.5)", (double)(0.01f * gps.get_hdop()));
        gpsCheckStatus.bad_hdop = true;
    } else {
        gpsCheckStatus.bad_hdop = false;
    }

    // fail if not enough sats
    bool numSatsFail = (gps.num_sats() < 6) && (frontend->_gpsCheck & MASK_GPS_NSATS);

    // Report check result as a text string and bitmask
    if (numSatsFail) {
        hal.util->snprintf(prearm_fail_string, sizeof(prearm_fail_string),
                           "GPS numsats %u (needs 6)", gps.num_sats());
        gpsCheckStatus.bad_sats = true;
    } else {
        gpsCheckStatus.bad_sats = false;
    }

    // fail if magnetometer innovations are outside limits indicating bad yaw
    // with bad yaw we are unable to use GPS
    bool yawFail;
    if ((magTestRatio.x > 1.0f || magTestRatio.y > 1.0f || yawTestRatio > 1.0f) && (frontend->_gpsCheck & MASK_GPS_YAW_ERR)) {
        yawFail = true;
    } else {
        yawFail = false;
    }

    // Report check result as a text string and bitmask
    if (yawFail) {
        hal.util->snprintf(prearm_fail_string,
                           sizeof(prearm_fail_string),
                           "Mag yaw error x=%.1f y=%.1f",
                           (double)magTestRatio.x,
                           (double)magTestRatio.y);
        gpsCheckStatus.bad_yaw = true;
    } else {
        gpsCheckStatus.bad_yaw = false;
    }

    // assume failed first time through and notify user checks have started
    if (lastGpsVelFail_ms == 0) {
        hal.util->snprintf(prearm_fail_string, sizeof(prearm_fail_string), "EKF starting GPS checks");
        lastGpsVelFail_ms = imuSampleTime_ms;
    }

    // record time of fail
    if (gpsSpdAccFail || numSatsFail || hdopFail || hAccFail || vAccFail ||  yawFail || gpsDriftFail || gpsVertVelFail || gpsHorizVelFail) {
        lastGpsVelFail_ms = imuSampleTime_ms;
    }

    // continuous period without fail required to return a healthy status
    if (imuSampleTime_ms - lastGpsVelFail_ms > 10000) {
        return true;
    }
    return false;
}

// update inflight calculaton that determines if GPS data is good enough for reliable navigation
void NavEKF2_core::calcGpsGoodForFlight(void)
{
    // use a simple criteria based on the GPS receivers claimed speed accuracy and the EKF innovation consistency checks

    // set up varaibles and constants used by filter that is applied to GPS speed accuracy
    const float alpha1 = 0.2f; // coefficient for first stage LPF applied to raw speed accuracy data
    const float tau = 10.0f; // time constant (sec) of peak hold decay
    if (lastGpsCheckTime_ms == 0) {
        lastGpsCheckTime_ms =  imuSampleTime_ms;
    }
    float dtLPF = (imuSampleTime_ms - lastGpsCheckTime_ms) * 1e-3f;
    lastGpsCheckTime_ms = imuSampleTime_ms;
    float alpha2 = constrain_float(dtLPF/tau,0.0f,1.0f);

    // get the receivers reported speed accuracy
    float gpsSpdAccRaw;
    if (!AP::gps().speed_accuracy(gpsSpdAccRaw)) {
        gpsSpdAccRaw = 0.0f;
    }

    // filter the raw speed accuracy using a LPF
    sAccFilterState1 = constrain_float((alpha1 * gpsSpdAccRaw + (1.0f - alpha1) * sAccFilterState1),0.0f,10.0f);

    // apply a peak hold filter to the LPF output
    sAccFilterState2 = MAX(sAccFilterState1,((1.0f - alpha2) * sAccFilterState2));

    // Apply a threshold test with hysteresis to the filtered GPS speed accuracy data
    if (sAccFilterState2 > 1.5f ) {
        gpsSpdAccPass = false;
    } else if(sAccFilterState2 < 1.0f) {
        gpsSpdAccPass = true;
    }

    // Apply a threshold test with hysteresis to the normalised position and velocity innovations
    // Require a fail for one second and a pass for 10 seconds to transition
    if (lastInnovFailTime_ms == 0) {
        lastInnovFailTime_ms = imuSampleTime_ms;
        lastInnovPassTime_ms = imuSampleTime_ms;
    }
    if (velTestRatio < 1.0f && posTestRatio < 1.0f) {
        lastInnovPassTime_ms = imuSampleTime_ms;
    } else if (velTestRatio > 0.7f || posTestRatio > 0.7f) {
        lastInnovFailTime_ms = imuSampleTime_ms;
    }
    if ((imuSampleTime_ms - lastInnovPassTime_ms) > 1000) {
        ekfInnovationsPass = false;
    } else if ((imuSampleTime_ms - lastInnovFailTime_ms) > 10000) {
        ekfInnovationsPass = true;
    }

    // both GPS speed accuracy and EKF innovations must pass
    gpsAccuracyGood = gpsSpdAccPass && ekfInnovationsPass;
}


#endif

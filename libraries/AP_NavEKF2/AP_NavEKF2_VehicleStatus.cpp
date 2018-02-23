#include <AP_HAL/AP_HAL.h>

#if HAL_CPU_CLASS >= HAL_CPU_CLASS_150

#include "AP_NavEKF2.h"
#include "AP_NavEKF2_core.h"
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <GCS_MAVLink/GCS.h>

#include <stdio.h>

extern const AP_HAL::HAL& hal;

// Detect if we are in flight or on ground
void NavEKF2_core::detectFlight()
{
    /*
        If we are a fly forward type vehicle (eg plane), then in-air status can be determined through a combination of speed and height criteria.
        Because of the differing certainty requirements of algorithms that need the in-flight / on-ground status we use two booleans where
        onGround indicates a high certainty we are not flying and inFlight indicates a high certainty we are flying. It is possible for
        both onGround and inFlight to be false if the status is uncertain, but they cannot both be true.

        If we are a plane as indicated by the assume_zero_sideslip() status, then different logic is used

        TODO - this logic should be moved out of the EKF and into the flight vehicle code.
    */

    if (assume_zero_sideslip()) {
        // To be confident we are in the air we use a criteria which combines arm status, ground speed, airspeed and height change
        float gndSpdSq = sq(gpsDataDelayed.vel.x) + sq(gpsDataDelayed.vel.y);
        bool highGndSpd = false;
        bool highAirSpd = false;
        bool largeHgtChange = false;

        // trigger at 8 m/s airspeed
        if (_ahrs->airspeed_sensor_enabled()) {
            const AP_Airspeed *airspeed = _ahrs->get_airspeed();
            if (airspeed->get_airspeed() * airspeed->get_EAS2TAS() > 10.0f) {
                highAirSpd = true;
            }
        }

        // trigger at 10 m/s GPS velocity, but not if GPS is reporting bad velocity errors
        if (gndSpdSq > 100.0f && gpsSpdAccuracy < 1.0f) {
            highGndSpd = true;
        }

        // trigger if more than 10m away from initial height
        if (fabsf(hgtMea) > 10.0f) {
            largeHgtChange = true;
        }

        // Determine to a high certainty we are flying
        if (motorsArmed && highGndSpd && (highAirSpd || largeHgtChange)) {
            onGround = false;
            inFlight = true;
        }

        // if is possible we are in flight, set the time this condition was last detected
        if (motorsArmed && (highGndSpd || highAirSpd || largeHgtChange)) {
            airborneDetectTime_ms = imuSampleTime_ms;
            onGround = false;
        }

        // Determine to a high certainty we are not flying
        // after 5 seconds of not detecting a possible flight condition or we are disarmed, we transition to on-ground mode
        if(!motorsArmed || ((imuSampleTime_ms - airborneDetectTime_ms) > 5000)) {
            onGround = true;
            inFlight = false;
        }
    } else {
        // Non fly forward vehicle, so can only use height and motor arm status

        // If the motors are armed then we could be flying and if they are not armed then we are definitely not flying
        if (motorsArmed) {
            onGround = false;
        } else {
            inFlight = false;
            onGround = true;
        }

        if (!onGround) {
            // If height has increased since exiting on-ground, then we definitely are flying
            if ((stateStruct.position.z - posDownAtTakeoff) < -1.5f) {
                inFlight = true;
            }

            // If rangefinder has increased since exiting on-ground, then we definitely are flying
            if ((rangeDataNew.rng - rngAtStartOfFlight) > 0.5f) {
                inFlight = true;
            }

            // If more than 5 seconds since likely_flying was set
            // true, then set inFlight true
            if (_ahrs->get_time_flying_ms() > 5000) {
                inFlight = true;
            }
        }

    }

    // store current on-ground  and in-air status for next time
    prevOnGround = onGround;
    prevInFlight = inFlight;

    // Store vehicle height and range prior to takeoff for use in post takeoff checks
    if (onGround) {
        // store vertical position at start of flight to use as a reference for ground relative checks
        posDownAtTakeoff = stateStruct.position.z;
        // store the range finder measurement which will be used as a reference to detect when we have taken off
        rngAtStartOfFlight = rangeDataNew.rng;
        // if the magnetic field states have been set, then continue to update the vertical position
        // quaternion and yaw innovation snapshots to use as a reference when we start to fly.
        if (magStateInitComplete) {
            posDownAtLastMagReset = stateStruct.position.z;
            quatAtLastMagReset = stateStruct.quat;
            yawInnovAtLastMagReset = innovYaw;
        }
    }

}


// determine if a takeoff is expected so that we can compensate for expected barometer errors due to ground effect
bool NavEKF2_core::getTakeoffExpected()
{
    if (expectGndEffectTakeoff && imuSampleTime_ms - takeoffExpectedSet_ms > frontend->gndEffectTimeout_ms) {
        expectGndEffectTakeoff = false;
    }

    return expectGndEffectTakeoff;
}

// called by vehicle code to specify that a takeoff is happening
// causes the EKF to compensate for expected barometer errors due to ground effect
void NavEKF2_core::setTakeoffExpected(bool val)
{
    takeoffExpectedSet_ms = imuSampleTime_ms;
    expectGndEffectTakeoff = val;
}


// determine if a touchdown is expected so that we can compensate for expected barometer errors due to ground effect
bool NavEKF2_core::getTouchdownExpected()
{
    if (expectGndEffectTouchdown && imuSampleTime_ms - touchdownExpectedSet_ms > frontend->gndEffectTimeout_ms) {
        expectGndEffectTouchdown = false;
    }

    return expectGndEffectTouchdown;
}

// called by vehicle code to specify that a touchdown is expected to happen
// causes the EKF to compensate for expected barometer errors due to ground effect
void NavEKF2_core::setTouchdownExpected(bool val)
{
    touchdownExpectedSet_ms = imuSampleTime_ms;
    expectGndEffectTouchdown = val;
}

// Set to true if the terrain underneath is stable enough to be used as a height reference
// in combination with a range finder. Set to false if the terrain underneath the vehicle
// cannot be used as a height reference
void NavEKF2_core::setTerrainHgtStable(bool val)
{
    terrainHgtStableSet_ms = imuSampleTime_ms;
    terrainHgtStable = val;
}

// Detect takeoff for optical flow navigation
void NavEKF2_core::detectOptFlowTakeoff(void)
{
    if (!onGround && !takeOffDetected && (imuSampleTime_ms - timeAtArming_ms) > 1000) {
        // we are no longer confidently on the ground so check the range finder and gyro for signs of takeoff
        const AP_InertialSensor &ins = _ahrs->get_ins();
        Vector3f angRateVec;
        Vector3f gyroBias;
        getGyroBias(gyroBias);
        bool dual_ins = ins.get_gyro_health(0) && ins.get_gyro_health(1);
        if (dual_ins) {
            angRateVec = (ins.get_gyro(0) + ins.get_gyro(1)) * 0.5f - gyroBias;
        } else {
            angRateVec = ins.get_gyro() - gyroBias;
        }

        takeOffDetected = (takeOffDetected || (angRateVec.length() > 0.1f) || (rangeDataNew.rng > (rngAtStartOfFlight + 0.1f)));
    } else if (onGround) {
        // we are confidently on the ground so set the takeoff detected status to false
        takeOffDetected = false;
    }
}


#endif // HAL_CPU_CLASS

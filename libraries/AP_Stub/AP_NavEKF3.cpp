#include <AP_NavEKF3/AP_NavEKF3.h>
#include <AP_AHRS/AP_AHRS_NavEKF.h>

#include <GCS_MAVLink/GCS.h>

#ifdef STUB_OUT_AP_NavEKF3

const AP_Param::GroupInfo NavEKF3::var_info[] = {
    AP_GROUPEND
};

NavEKF3::NavEKF3(AP_AHRS const*ahrs, AP_Baro&baro, RangeFinder const&rng) :
    _ahrs(ahrs),
    _baro(baro),
    _rng(rng),
    gpsNEVelVarAccScale(0.05f),     // Scale factor applied to horizontal velocity measurement variance due to manoeuvre acceleration - used when GPS doesn't report speed error
    gpsDVelVarAccScale(0.07f),      // Scale factor applied to vertical velocity measurement variance due to manoeuvre acceleration - used when GPS doesn't report speed error
    gpsPosVarAccScale(0.05f),       // Scale factor applied to horizontal position measurement variance due to manoeuvre acceleration
    magDelay_ms(60),                // Magnetometer measurement delay (msec)
    tasDelay_ms(100),               // Airspeed measurement delay (msec)
    tiltDriftTimeMax_ms(15000),      // Maximum number of ms allowed without any form of tilt aiding (GPS, flow, TAS, etc)
    posRetryTimeUseVel_ms(10000),   // Position aiding retry time with velocity measurements (msec)
    posRetryTimeNoVel_ms(7000),     // Position aiding retry time without velocity measurements (msec)
    hgtRetryTimeMode0_ms(10000),    // Height retry time with vertical velocity measurement (msec)
    hgtRetryTimeMode12_ms(5000),    // Height retry time without vertical velocity measurement (msec)
    tasRetryTime_ms(5000),          // True airspeed timeout and retry interval (msec)
    magFailTimeLimit_ms(10000),     // number of msec before a magnetometer failing innovation consistency checks is declared failed (msec)
    magVarRateScale(0.005f),        // scale factor applied to magnetometer variance due to angular rate and measurement timing jitter. Assume timing jitter of 10msec
    gyroBiasNoiseScaler(2.0f),      // scale factor applied to imu gyro bias learning before the vehicle is armed
    hgtAvg_ms(100),                 // average number of msec between height measurements
    betaAvg_ms(100),                // average number of msec between synthetic sideslip measurements
    covTimeStepMax(0.1f),           // maximum time (sec) between covariance prediction updates
    covDelAngMax(0.05f),            // maximum delta angle between covariance prediction updates
    DCM33FlowMin(0.71f),            // If Tbn(3,3) is less than this number, optical flow measurements will not be fused as tilt is too high.
    fScaleFactorPnoise(1e-10f),     // Process noise added to focal length scale factor state variance at each time step
    flowTimeDeltaAvg_ms(100),       // average interval between optical flow measurements (msec)
    flowIntervalMax_ms(100),        // maximum allowable time between flow fusion events
    gndEffectTimeout_ms(1000),      // time in msec that baro ground effect compensation will timeout after initiation
    gndEffectBaroScaler(4.0f),      // scaler applied to the barometer observation variance when operating in ground effect
    gndGradientSigma(50),           // RMS terrain gradient percentage assumed by the terrain height estimation
    fusionTimeStep_ms(10),          // The minimum number of msec between covariance prediction and fusion operations
    sensorIntervalMin_ms(50),       // The minimum allowed time between measurements from any non-IMU sensor (msec)
    runCoreSelection(false),        // true when the default primary core has stabilised after startup and core selection can run
    inhibitGpsVertVelUse(false)     // true when GPS vertical velocity use is prohibited
{}

void NavEKF3::resetGyroBias(void)
{
}

static const char *stubbed_out_message = "NavEKF3 has been stubbed out";
bool NavEKF3::InitialiseFilter(void)
{
    return false;
}

const char *NavEKF3::prearm_failure_reason(void) const
{
    return stubbed_out_message;
}

uint32_t NavEKF3::getLastYawResetAngle(float &yawAngDelta)
{
    return 0;
}

uint32_t NavEKF3::getLastPosNorthEastReset(Vector2f &posDelta)
{
    return 0;
}

uint32_t NavEKF3::getLastPosDownReset(float &posDelta)
{
    return 0;
}

uint32_t NavEKF3::getLastVelNorthEastReset(Vector2f &vel) const
{
    return 0;
}

bool NavEKF3::resetHeightDatum(void)
{
    return false;
}

bool NavEKF3::getOriginLLH(int8_t instance, struct Location &loc) const
{
    return false;
}

bool NavEKF3::getHeightControlLimit(float &height) const
{
    return false;
}

void NavEKF3::getVariances(int8_t instance, float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar, Vector2f &offset)
{
    // turns out we always agree with the sensors:
    velVar = 0;
    posVar = 0;
    hgtVar = 0;
    magVar = {0,0,0};
    tasVar = 0;
    offset = {0, 0};
}

void NavEKF3::getFilterFaults(int8_t instance, uint16_t &faults)
{
    // everything is all good, all the time:
    faults = 0;
}

bool NavEKF3::healthy(void) const
{
    // everything is all bad, all the time:
    return false;
}

void NavEKF3::getFilterStatus(int8_t instance, nav_filter_status &status)
{
    // everything is all statussy, all the time
}

bool NavEKF3::getLLH(struct Location &loc) const
{
    return false;
}

bool NavEKF3::getPosD(int8_t instance, float &posD)
{
    posD = 0;
    return false;
}

void NavEKF3::getWind(int8_t instance, Vector3f &wind)
{
    wind = {0,0,0};
}

bool NavEKF3::use_compass(void) const
{
    return false;
}

void NavEKF3::getVelNED(int8_t instance, Vector3f &vel)
{
    vel = {0,0,0};
}

bool NavEKF3::setOriginLLH(const Location &loc)
{
    return false;
}

void NavEKF3::getMagXYZ(int8_t instance, Vector3f &magXYZ)
{
}

float NavEKF3::getPosDownDerivative(int8_t instance)
{
    return 0;
}

bool NavEKF3::getPosNE(int8_t instance, Vector2f &posNE)
{
    return false;
}

void NavEKF3::UpdateFilter(void)
{
}

void NavEKF3::getRotationBodyToNED(Matrix3f &mat) const
{
    mat = {{0,0,0},{0,0,0},{0,0,0}};
}

void NavEKF3::getEulerAngles(int8_t instance, Vector3f &eulers)
{
    eulers = {0,0,0};
}

int8_t NavEKF3::getPrimaryCoreIndex(void) const
{
    return -1;
}

void NavEKF3::getGyroBias(int8_t instance, Vector3f &gyroBias)
{
}

void NavEKF3::getAccelBias(int8_t instance, Vector3f &accelBias)
{
}

void NavEKF3::getEkfControlLimits(float &ekfGndSpdLimit, float &ekfNavVelGainScaler) const
{
    ekfGndSpdLimit = 0.0f;
    ekfNavVelGainScaler = 0.0f;
}

int8_t NavEKF3::getPrimaryCoreIMUIndex(void) const
{
    return -1;
}

bool NavEKF3::getMagOffsets(uint8_t mag_idx, Vector3f &magOffsets) const
{
    return false;
}

void NavEKF3::send_status_report(mavlink_channel_t chan)
{
}

void NavEKF3::setTakeoffExpected(bool val)
{
}

void NavEKF3::setTouchdownExpected(bool val)
{
}

void NavEKF3::writeBodyFrameOdom(float quality, const Vector3f &delPos, const Vector3f &delAng, float delTime, uint32_t timeStamp_ms, const Vector3f &posOffset)
{
}

void NavEKF3::writeOptFlowMeas(uint8_t &rawFlowQuality, Vector2f &rawFlowRates, Vector2f &rawGyroRates, uint32_t &msecFlowMeas, const Vector3f &posOffset)
{
}

bool NavEKF3::getHAGL(float &HAGL) const
{
    return false;
}

void NavEKF3::getMagNED(int8_t instance, Vector3f &magNED)
{
}

void NavEKF3::Log_Write_EKF3(AP_AHRS_NavEKF &ahrs)
{
}

#endif

/**
 * @file FusionAhrs.h
 * @author Seb Madgwick
 * @brief The AHRS sensor fusion algorithm to combines gyroscope, accelerometer,
 * and magnetometer measurements into a single measurement of orientation
 * relative to the Earth (NWU convention).
 *
 * The algorithm behaviour is governed by a gain.  A low gain will decrease the
 * influence of the accelerometer and magnetometer so that the algorithm will
 * better reject disturbances causes by translational motion and temporary
 * magnetic distortions.  However, a low gain will also increase the risk of
 * drift due to gyroscope calibration errors.  A typical gain value suitable for
 * most applications is 0.5.
 *
 * The algorithm allows the application to define a minimum and maximum valid
 * magnetic field magnitude.  The algorithm will ignore magnetic measurements
 * that fall outside of this range.  This allows the algorithm to reject
 * magnetic measurements that do not represent the direction of magnetic North.
 * The typical magnitude of the Earth's magnetic field is between 20 uT and
 * 70 uT.
 *
 * The algorithm can be used without a magnetometer.  Measurements of
 * orientation obtained using only gyroscope and accelerometer measurements
 * can be expected to drift in the yaw component of orientation only.  The
 * application can reset the drift in yaw by setting the yaw to a specified
 * angle at any time.
 *
 * The algorithm provides the measurement of orientation as a quaternion.  The
 * library includes functions for converting this quaternion to a rotation
 * matrix and Euler angles.
 *
 * The algorithm also provides a measurement of linear acceleration and Earth
 * acceleration.  Linear acceleration is equal to the accelerometer  measurement
 * with the 1 g of gravity removed.  Earth acceleration is a measurement of
 * linear acceleration in the Earth coordinate frame.
 */

#ifndef FUSION_AHRS_H
#define FUSION_AHRS_H

 //------------------------------------------------------------------------------
 // Includes

#include "FusionDefines.h"
#include "FusionTypes.h"
#include <stdbool.h>

//------------------------------------------------------------------------------
// Definitions

/**
 * @brief AHRS algorithm structure.  Structure members are used internally and
 * should not be used by the user application.
 */
typedef struct {
    float gain;
    float minimumMagneticFieldSquared;
    float maximumMagneticFieldSquared;
    FusionQuaternion quaternion; // describes the Earth relative to the sensor
    FusionVector3 linearAcceleration;
    float rampedGain;
    bool zeroYawPending;
} FusionAhrs;

//------------------------------------------------------------------------------
// Function prototypes

MADGWICK_API void FusionAhrsInitialise(FusionAhrs* const fusionAhrs, const float gain);
MADGWICK_API void FusionAhrsSetGain(FusionAhrs* const fusionAhrs, const float gain);
MADGWICK_API void FusionAhrsSetMagneticField(FusionAhrs* const fusionAhrs, const float minimumMagneticField, const float maximumMagneticField);
MADGWICK_API void FusionAhrsUpdate(FusionAhrs* const fusionAhrs, const FusionVector3 gyroscope, const FusionVector3 accelerometer, const FusionVector3 magnetometer, const float samplePeriod);
MADGWICK_API void FusionAhrsUpdateWithoutMagnetometer(FusionAhrs* const fusionAhrs, const FusionVector3 gyroscope, const FusionVector3 accelerometer, const float samplePeriod);
MADGWICK_API FusionQuaternion FusionAhrsGetQuaternion(const FusionAhrs* const fusionAhrs);
MADGWICK_API FusionVector3 FusionAhrsGetLinearAcceleration(const FusionAhrs* const fusionAhrs);
MADGWICK_API FusionVector3 FusionAhrsGetEarthAcceleration(const FusionAhrs* const fusionAhrs);
MADGWICK_API void FusionAhrsReinitialise(FusionAhrs* const fusionAhrs);
MADGWICK_API bool FusionAhrsIsInitialising(const FusionAhrs* const fusionAhrs);
MADGWICK_API void FusionAhrsSetYaw(FusionAhrs* const fusionAhrs, const float yaw);

#endif


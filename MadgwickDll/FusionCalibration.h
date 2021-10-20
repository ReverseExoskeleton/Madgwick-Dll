/**
 * @file FusionCalibration.h
 * @author Seb Madgwick
 * @brief Gyroscope, accelerometer, and magnetometer calibration model.
 *
 * Static inline implementations are used to optimise for increased execution
 * speed.
 */

#ifndef FUSION_CALIBRATION_H
#define FUSION_CALIBRATION_H

 //------------------------------------------------------------------------------
 // Includes

#include "FusionDefines.h"
#include "FusionTypes.h"

//------------------------------------------------------------------------------
// Function prototypes


MADGWICK_API FusionVector3 FusionCalibrationInertial(const FusionVector3 uncalibrated, const FusionRotationMatrix misalignment, const FusionVector3 sensitivity, const FusionVector3 bias);
MADGWICK_API FusionVector3 FusionCalibrationMagnetic(const FusionVector3 uncalibrated, const FusionRotationMatrix softIronMatrix, const FusionVector3 hardIronBias);

#endif

//------------------------------------------------------------------------------
// End of file

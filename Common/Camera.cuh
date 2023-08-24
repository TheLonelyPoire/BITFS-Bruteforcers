#pragma once

#include "cuda.h"
#include "cuda_runtime.h"

#include "Trig.cuh"

namespace BITFS {

    extern bool validCameraAngles[65536];

    // Only CPU for now
    void init_camera_angles();

    // accurate at PU distances!
    __host__ __device__ int crude_camera_yaw(float* currentPosition, float* lakituPosition);

    __host__ __device__ int fine_camera_yaw(float* currentPosition, float* lakituPosition, short faceAngle, bool onPole = false);

    __host__ __device__ int tenk_camera_yaw(float* currentPosition, float* lakituPosition, short faceAngle, float* trueCameraFocus, float* truePan, bool onPole = false);

}
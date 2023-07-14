#pragma once

#include "cuda.h"
#include "cuda_runtime.h"

#include "Trig.cuh"

namespace BITFS {

    // accurate at PU distances!
    __device__ int crude_camera_yawG(float* currentPosition, float* lakituPosition);

    // CPU version of the above function.
    int crude_camera_yaw(float* currentPosition, float* lakituPosition);

}
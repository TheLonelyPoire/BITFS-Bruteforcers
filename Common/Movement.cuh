#pragma once

#include "cuda.h"
#include "cuda_runtime.h"

#include "CommonBruteforcerStructs.hpp"


namespace BITFS {

	__host__ __device__ bool sim_slide(StickTableData stick, float* startPos, float forward_speed, float vX, float vZ, int faceAngle, int slideYaw, int camera, bool usePolePlatform, FancySlideInfo& output);

	__host__ __device__ bool sim_airstep(float* initialPos, float initialSpeed, int initialAngle, bool first, AirInfo& output);

}
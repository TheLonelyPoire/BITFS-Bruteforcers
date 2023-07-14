#pragma once

#include "cuda.h"
#include "cuda_runtime.h"

#include "BruteforcerStructs.hpp"


namespace BITFS {

	__host__ __device__ FancySlideInfo sim_slide(StickTableData stick, float* startPos, float forward_speed, float vX, float vZ, int faceAngle, int slideYaw, int camera);

}
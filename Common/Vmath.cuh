#pragma once

#include "cuda.h"
#include "cuda_runtime.h"

namespace BITFS {

	__host__ __device__ float find_dis(float* firstPos, float* secondPos);

	__host__ __device__ float intervalClip(float low, float hi, float value);

}
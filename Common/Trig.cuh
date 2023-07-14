#pragma once

#include "cuda.h"
#include "cuda_runtime.h"
#include <stdint.h>

namespace BITFS {

	extern __device__ float gSineTableG[4096];
	extern __device__ float gCosineTableG[4096];
	extern __device__ int gArctanTableG[8192];

	// yes we need separate trig tables for the CPU and GPU.

	extern float gSineTable[4096];
	extern float gCosineTable[4096];
	extern int gArctanTable[8192];

	__host__ __device__ int16_t atan2_lookup(float z, float x);

	__host__ __device__ int16_t atan2s(float z, float x);

	
	__host__ __device__ int fix(int angle);

	__host__ __device__ float lawofCosines(float c, float a, float b);
}
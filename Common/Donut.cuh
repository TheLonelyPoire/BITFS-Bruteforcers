#pragma once
#include "cuda.h"
#include "cuda_runtime.h"

#include "CommonBruteforcerStructs.hpp"

namespace BITFS {

	//this takes two circles, their radii and distance, and computes the area of their overlap.
	__host__ __device__ float circle_compute(float sRad, float tRad, float dis);

	//this is a black magic one. Given two donuts, find the angles where they overlap and the area of their overlap.
	//input is the four coordinates of the donut centers and the four donut radii.
	__host__ __device__ DonutData donut_compute(float* marioPos, float sRadInner, float sRadOuter, float* targetPos, float tRadInner, float tRadOuter);

}
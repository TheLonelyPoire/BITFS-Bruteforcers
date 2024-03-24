#pragma once

#include "cuda.h"
#include "cuda_runtime.h"

namespace BITFS {

	__host__ __device__ void vec3_copy(float* to, float* from);

	__host__ __device__ void vec3_normalize(float* dest);

	__host__ __device__ void vec3_cross(float* dest, float* a, float* b);

	__host__ __device__ float find_dis(float* firstPos, float* secondPos);

	//  Computes a dot product for 2D vectors. There's probably a c++ function for this.
	__host__ __device__ float dot_prod_2(float* v, float* w);

	// Computes a dot product for 3D vectors. There's probably a c++ function for this.
	__host__ __device__ float dot_prod_3(float* v, float* w);

	// Computes a dot product for vectors of dimensionality `n_dim`. There's probably a c++ function for this.
	__host__ __device__ float dot_prod_n(float* v, float* w, int n_dim);

	__host__ __device__ float intervalClip(float low, float hi, float value);

	__host__ __device__ void vec3f_set_dist_and_angle(float* from, float* to, float dist, unsigned short pitch, unsigned short yaw);

}
#include "Vmath.cuh"
#include "math.h"

namespace BITFS {

	__host__ __device__ float find_dis(float* firstPos, float* secondPos) {
		return sqrtf((firstPos[0] - secondPos[0]) * (firstPos[0] - secondPos[0]) + (firstPos[2] - secondPos[2]) * (firstPos[2] - secondPos[2]));
	}

	//  Computes a dot product for 2D vectors. There's probably a c++ function for this.
	__host__ __device__ float dot_prod_2(float* v, float* w) {
		return v[0] * w[0] + v[1] * w[1];
	}

	// Computes a dot product for 3D vectors. There's probably a c++ function for this.
	__host__ __device__ float dot_prod_3(float* v, float* w) {
		return v[0] * w[0] + v[1] * w[1] + v[2] * w[2];
	}

	// Computes a dot product for vectors of dimensionality `n_dim`. There's probably a c++ function for this.
	__host__ __device__ float dot_prod_n(float* v, float* w, int n_dim) {
		float sum = 0;
		for (int i = 0; i < n_dim; ++i)
		{
			sum += v[i] * w[i];
		}
		return sum;
	}

	// this takes a float and clips it to the closest value within an interval.
	__host__ __device__ float intervalClip(float low, float hi, float value) {
		return fminf(hi, fmaxf(low, value));
	}
}

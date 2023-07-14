#include "Vmath.cuh"
#include "math.h"

namespace BITFS {

	__host__ __device__ float find_dis(float* firstPos, float* secondPos) {
		return sqrtf((firstPos[0] - secondPos[0]) * (firstPos[0] - secondPos[0]) + (firstPos[2] - secondPos[2]) * (firstPos[2] - secondPos[2]));
	}


	// this takes a float and clips it to the closest value within an interval.
	__host__ __device__ float intervalClip(float low, float hi, float value) {
		return fminf(hi, fmaxf(low, value));
	}
}

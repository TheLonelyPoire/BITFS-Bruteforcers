#pragma once

#include "cuda.h"
#include "cuda_runtime.h"
#include <stdint.h>

namespace BITFS {

    __host__ __device__ uint32_t roll(uint32_t n, int m);

    __host__ __device__ uint32_t murmur3(float d1, float d2, float d3, uint32_t seed);

    __host__ __device__ void hash(float d1, float d2, float d3, int* locations, int k);

    __host__ __device__ void bloom_filter(int n, float* data1, float* data2, float* data3, bool* unique);

}
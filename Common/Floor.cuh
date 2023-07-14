#pragma once

#include "cuda.h"
#include "cuda_runtime.h"

#include "Surface.cuh"

namespace BITFS {

    __device__ const int total_floorsG = 38;
    extern __device__ Surface floorsG[total_floorsG];

    __global__ void initialise_floors();

    __device__ int find_floor(float* position, Surface** floor, float& floor_y, Surface floor_set[], int n_floor_set);

    __device__ int assess_floor(float* position);

    __device__ bool stability_check(float* position, float speed, int angle);
}
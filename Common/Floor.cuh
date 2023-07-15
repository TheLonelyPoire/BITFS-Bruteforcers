#pragma once

#include "cuda.h"
#include "cuda_runtime.h"

#include "Surface.cuh"

namespace BITFS {

    __device__ const int total_floors = 38;
    extern __device__ Surface floorsG[total_floors];
    extern Surface floors[total_floors];

    __global__ void initialise_floorsG();

    void initialise_floors();

    __host__ __device__ int find_floor(float* position, Surface** floor, float& floor_y, Surface floor_set[], int n_floor_set);

    __host__ __device__ int assess_floor(float* position);

    __host__ __device__ bool stability_check(float* position, float speed, int angle);

    __host__ __device__ bool on_one_up(float* position);

}
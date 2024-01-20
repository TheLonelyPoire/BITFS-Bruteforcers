#pragma once

#include "cuda.h"
#include "cuda_runtime.h"

#include "CommonBruteforcerStructs.hpp"

namespace BITFS {

	extern __device__ StickTableData stickTabG[20129];
    extern StickTableData stickTab[20129];

    // novel function here. Basically, some stick positions are redundant, so what this does is iterates through the stick positions,
    //computes their magnitudes and angles, iterates through stick positions again to look for any exact copies, and if there's no exact
    //copy, adds its data to the table of unique stick positions. This cuts down on the number of stick positions to test.
    __global__ void init_stick_tablesG(bool backwardsOnly = false);

    // novel function here. Basically, some stick positions are redundant, so what this does is iterates through the stick positions,
    //computes their magnitudes and angles, iterates through stick positions again to look for any exact copies, and if there's no exact
    //copy, adds its data to the table of unique stick positions. This cuts down on the number of stick positions to test.
    void init_stick_tables(bool backwardsOnly = false);

    // converts internal stick positions back into input stick positions.
    __host__ __device__ int correct_stick(int stick);

    // Works out the stick position needed to hit a given target with a 1 frame crouchslide. Return value is whether or not a valid stick position was computed.
    __host__ __device__ bool infer_stick(float* startPos, float* endPos, float startSpeed, int angle, int startCameraYaw, StickTableData& output);
}
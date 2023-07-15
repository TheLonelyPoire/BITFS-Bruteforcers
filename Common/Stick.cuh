#pragma once

#include "cuda.h"
#include "cuda_runtime.h"

#include "BruteforcerStructs.hpp"

namespace BITFS {

	extern __device__ StickTableData stickTabG[20129];
    extern StickTableData stickTab[20129];

    // novel function here. Basically, some stick positions are redundant, so what this does is iterates through the stick positions,
    //computes their magnitudes and angles, iterates through stick positions again to look for any exact copies, and if there's no exact
    //copy, adds its data to the table of unique stick positions. This cuts down on the number of stick positions to test.
    __global__ void init_stick_tablesG();

    // novel function here. Basically, some stick positions are redundant, so what this does is iterates through the stick positions,
    //computes their magnitudes and angles, iterates through stick positions again to look for any exact copies, and if there's no exact
    //copy, adds its data to the table of unique stick positions. This cuts down on the number of stick positions to test.
    void init_stick_tables();

    // converts internal stick positions back into input stick positions.
    __device__ int correct_stick(int stick);

    // works out the stick position needed to hit a given target with a 1 frame crouchslide.
    __device__ StickTableData infer_stick(float* startPos, float* endPos, float startSpeed, int angle, int startCameraYaw);
}
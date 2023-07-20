#include <fstream>
#include <cstring>
#include <string>
#include <unordered_map>
#include <iostream>
#include <cmath>
#include "cuda.h"
#include "cuda_runtime.h"
#include "device_launch_parameters.h"
#include "device_atomic_functions.h"

#include "../Common/CommonBruteforcerStructs.hpp"
#include "../Common/Camera.cuh"
#include "../Common/Donut.cuh"
#include "../Common/Floor.cuh"
#include "../Common/Movement.cuh"
#include "../Common/Stick.cuh"
#include "../Common/Surface.cuh"
#include "../Common/Trig.cuh"
#include "../Common/VMath.cuh"

#include "HighSpeedRouteStructs.hpp"


# define MAX_FIRST_SLIDES 10000
# define MAX_SECOND_SLIDES 10000
# define MAX_THIRD_SLIDES 10000
# define MAX_FOURTH_SLIDES 100000

using namespace BITFS;

__device__ MotionData13* firstSlides;
__device__ int nFirstSlides;
__device__ MotionData2* secondSlides;
__device__ int nSecondSlides;
__device__ MotionData13* thirdSlides;
__device__ int nThirdSlides;
__device__ MotionData4* fourthSlides;
__device__ int nFourthSlides;
__device__ int lookupTable[8192];
__device__ int lookupSize;
__device__ AllData* finalSolutionsLog;
__device__ int nSolutions = 0;



// copied from Spud's thing.
__global__ void copy_pointers_to_gpu(MotionData13* p1, MotionData2* p2, MotionData13* p3, MotionData4* p4, AllData* p5) {
    firstSlides = p1;
    secondSlides = p2;
    thirdSlides = p3;
    fourthSlides = p4;
    finalSolutionsLog = p5;
}


// initializes a table with a bunch of 0's and 1's, and returns the number of 1's.
__global__ void table_builder(int index2, TargetLog target) {
    lookupSize = 0;
    int camYaw = fix(crude_camera_yaw(secondSlides[index2].nextPos, target.posCam));

    // now, there's a bit of black magic here. We have two intervals of HAU's. So we convert them to AU's.
    // the way of testing whether or not a point is "between them" is that we define the distance between two angles
    // to be the minimum of the two distances between those angles.
    // then, point 2 (our facing angle) is between points 1 and 3 (the HAU bounds) precisely when the 1 to 3 distance
    // equals the 1 to 2 distance plus the 2 to 3 distance.
    // but we have to do everything twice because there are two bands of HAU's.

    auto hauBand = secondSlides[index2].donut.hauBand;
    int dist130 = min(fix(hauBand[0][0] * 16 - hauBand[0][1] * 16), fix(hauBand[0][1] * 16 - hauBand[0][0] * 16));
    int dist131 = min(fix(hauBand[1][0] * 16 - hauBand[1][1] * 16), fix(hauBand[1][1] * 16 - hauBand[1][0] * 16));

    for (int t = 0; t < 8192; t++) {

        int fangle = fix(gArctanTableG[t] + camYaw);

        int dist230 = min(fix(fangle - hauBand[0][1] * 16), fix(hauBand[0][1] * 16 - fangle));
        int dist231 = min(fix(fangle - hauBand[1][1] * 16), fix(hauBand[1][1] * 16 - fangle));

        int dist120 = min(fix(fangle - hauBand[0][0] * 16), fix(hauBand[0][0] * 16 - fangle));
        int dist121 = min(fix(fangle - hauBand[1][0] * 16), fix(hauBand[1][0] * 16 - fangle));

        // if it's in neither band,mark it as false and continue.
        if (dist120 + dist230 > dist130 && dist121 + dist231 > dist131) {
            continue;
        }
        else {
            lookupTable[lookupSize] = t;
            lookupSize++;
        }
    }
}


__device__ BullyData sim_bully_collision(float* marioPos, float* bullyPos, int facingAngle, float marioVel) {

    float offsetX = marioPos[0] - bullyPos[0];
    float offsetZ = marioPos[2] - bullyPos[2];

    // Removed unecessary distance calculation with sqrtf

    int pushAngle;

    if (offsetX * offsetX + offsetZ * offsetZ == 0.0f) {
        pushAngle = fix(facingAngle);
    }
    else {
        pushAngle = fix(atan2s(offsetZ, offsetX));
    }

    float newMarioX = bullyPos[0] + 115.0f * gSineTableG[pushAngle >> 4];
    float newMarioZ = bullyPos[2] + 115.0f * gCosineTableG[pushAngle >> 4];

    float marioSpeed = -1.0f * marioVel;
    int marioYaw = fix(facingAngle + 0x8000);

    float marioVelX = marioSpeed * gSineTableG[marioYaw >> 4];
    float marioVelZ = marioSpeed * gCosineTableG[marioYaw >> 4];

    float rx = bullyPos[0] - newMarioX;
    float rz = bullyPos[2] - newMarioZ;

    float projectedV1 = (rx * marioVelX + rz * marioVelZ) / (rx * rx + rz * rz);

    float velX = (53.0f / 73.0f) * projectedV1 * rx;
    float velZ = (53.0f / 73.0f) * projectedV1 * rz;

    int bullyYaw = fix(atan2s(velZ, velX));
    float bullyVel = sqrtf(velX * velX + velZ * velZ);

    struct BullyData solution;
    solution.posBully[0] = bullyPos[0];
    solution.posBully[1] = bullyPos[1];
    solution.posBully[2] = bullyPos[2];
    solution.angle = bullyYaw;
    solution.velBully = bullyVel;

    return solution;

}


__global__ void rewrite_structs(int index1, int index2, TargetLog target, float posX, float posY, float posZ, float firstSpeed) {

    int counter = 0;

    for (int i = 0; i < nFourthSlides; i++) {

        if (nSolutions + i >= MAX_FOURTH_SLIDES) {
            break;
        }

        MotionData4* fourthData = &(fourthSlides[i]);
        MotionData13* thirdData = &(thirdSlides[fourthData->identifier]);
        MotionData2* secondData = &(secondSlides[index2]);
        MotionData13* firstData = &(firstSlides[index1]);

        finalSolutionsLog[nSolutions + i].positions.posCam[0] = target.posCam[0];
        finalSolutionsLog[nSolutions + i].positions.posCam[1] = target.posCam[1];
        finalSolutionsLog[nSolutions + i].positions.posCam[2] = target.posCam[2];
        finalSolutionsLog[nSolutions + i].positions.pos21[0] = posX;
        finalSolutionsLog[nSolutions + i].positions.pos21[1] = posY;
        finalSolutionsLog[nSolutions + i].positions.pos21[2] = posZ;
        finalSolutionsLog[nSolutions + i].positions.pos22[0] = firstData->nextPos[0];
        finalSolutionsLog[nSolutions + i].positions.pos22[1] = firstData->nextPos[1];
        finalSolutionsLog[nSolutions + i].positions.pos22[2] = firstData->nextPos[2];
        finalSolutionsLog[nSolutions + i].positions.pos23[0] = secondData->nextPos[0];
        finalSolutionsLog[nSolutions + i].positions.pos23[1] = secondData->nextPos[1];
        finalSolutionsLog[nSolutions + i].positions.pos23[2] = secondData->nextPos[2];
        finalSolutionsLog[nSolutions + i].positions.pos24[0] = thirdData->nextPos[0];
        finalSolutionsLog[nSolutions + i].positions.pos24[1] = thirdData->nextPos[1];
        finalSolutionsLog[nSolutions + i].positions.pos24[2] = thirdData->nextPos[2];

        finalSolutionsLog[nSolutions + i].velocities.vel21 = firstSpeed;
        finalSolutionsLog[nSolutions + i].velocities.vel22 = firstData->nextVel;
        finalSolutionsLog[nSolutions + i].velocities.vel23 = secondData->nextVel;
        finalSolutionsLog[nSolutions + i].velocities.vel24 = thirdData->nextVel;

        finalSolutionsLog[nSolutions + i].sticks.stick21X = firstData->stickX;
        finalSolutionsLog[nSolutions + i].sticks.stick21Y = firstData->stickY;
        finalSolutionsLog[nSolutions + i].sticks.stick22X = secondData->stickX;
        finalSolutionsLog[nSolutions + i].sticks.stick22Y = secondData->stickY;
        finalSolutionsLog[nSolutions + i].sticks.stick23X = thirdData->stickX;
        finalSolutionsLog[nSolutions + i].sticks.stick23Y = thirdData->stickY;
        finalSolutionsLog[nSolutions + i].sticks.stick24X = fourthData->stickX;
        finalSolutionsLog[nSolutions + i].sticks.stick24Y = fourthData->stickY;

        finalSolutionsLog[nSolutions + i].angles.cam21 = firstData->camAngle;
        finalSolutionsLog[nSolutions + i].angles.facing21 = firstData->facingAngle;
        finalSolutionsLog[nSolutions + i].angles.cam22 = secondData->camAngle;
        finalSolutionsLog[nSolutions + i].angles.facing22 = secondData->facingAngle;
        finalSolutionsLog[nSolutions + i].angles.cam23 = thirdData->camAngle;
        finalSolutionsLog[nSolutions + i].angles.facing23 = thirdData->facingAngle;
        finalSolutionsLog[nSolutions + i].angles.cam24 = fourthData->camAngle;
        finalSolutionsLog[nSolutions + i].angles.facing24 = fourthData->facingAngle;

        finalSolutionsLog[nSolutions + i].waits.waiting21 = firstData->waitingFrames;
        finalSolutionsLog[nSolutions + i].waits.waiting22 = secondData->waitingFrames;
        finalSolutionsLog[nSolutions + i].waits.waiting23 = thirdData->waitingFrames;

        // at this point, we must resimulate the final crouchslide to find Mario's angle and other parameters
        // of relevance for simulating the bully collision.

        StickTableData sticksol = infer_stick(thirdData->nextPos, target.posBully, thirdData->nextVel, fourthData->facingAngle, fourthData->camAngle);

        FancySlideInfo lastslide;
        bool successfulSlide = sim_slide(sticksol, thirdData->nextPos, thirdData->nextVel, (thirdData->nextVel) * gSineTableG[(fourthData->facingAngle) >> 4], (thirdData->nextVel) * gCosineTableG[(fourthData->facingAngle) >> 4], fourthData->facingAngle, fourthData->facingAngle, fourthData->camAngle, false, lastslide);

        if (!successfulSlide){
            printf("FAILED SLIDE IN REWRITE STRUCTS! THIS SHOULD NOT HAPPEN!");
            return;
        }

        // then, with this info, we simulate the bully impact.

        BullyData collide = sim_bully_collision(lastslide.endPos, target.posBully, lastslide.endFacingAngle, lastslide.endSpeed);

        finalSolutionsLog[nSolutions + i].bully.posBully[0] = collide.posBully[0];
        finalSolutionsLog[nSolutions + i].bully.posBully[1] = collide.posBully[1];
        finalSolutionsLog[nSolutions + i].bully.posBully[2] = collide.posBully[2];
        finalSolutionsLog[nSolutions + i].bully.angle = collide.angle;
        finalSolutionsLog[nSolutions + i].bully.velBully = collide.velBully;

        counter++;
    }

    nSolutions += counter;
}


__global__ void fourth_move(TargetLog target) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= 64 * min(MAX_THIRD_SLIDES, nThirdSlides)) {
        return;
    }
    int delta = idx % 64;
    int index = (idx - delta) / 64;

    // since we're not doing explicit iteration, we fetch the linked data directly from the GPU.

    MotionData13* data = &(thirdSlides[index]);

    // which lets us compute the camera yaw, and the direction from the bully to us, which approximates our facing angle.
    int camYaw = fix(crude_camera_yaw(data->nextPos, target.posCam));
    int direction = atan2s(data->nextPos[2] - target.posBully[2], data->nextPos[0] - target.posBully[0]);
    direction = fix(direction);

    // and to get our facing angle, we take the direction and add a displacement depending on our "delta" number
    // (roughly, the number of half-AU's over we are, from -32 to 32, or -16 to +16 HAU's)
    // and get it back to the standard 0 to 65536 range.

    int fangle = fix(direction + 8 * (delta - 32));

    // check stability

    if (!stability_check(data->nextPos, data->nextVel, fangle)) {
        return;
    }

    // infer stick position

    StickTableData sticksol = infer_stick(data->nextPos, target.posBully, data->nextVel, fangle, camYaw);

    // check to see if the stick position is reasonable
    if (sticksol.magnitude > 64.0f || sticksol.magnitude < 0) {
        return;
    }

    // simulate the slide.

    FancySlideInfo fourthslide;
    if (!sim_slide(sticksol, data->nextPos, data->nextVel, (data->nextVel) * gSineTableG[fangle >> 4], (data->nextVel) * gCosineTableG[fangle >> 4], fangle, fangle, camYaw, false, fourthslide)) {
        return;
    }

    // see how much you miss the bully by. If it's below 63, it's a hit!

    float miss = find_dis(fourthslide.endPos, target.posBully);
    if (miss > 63.0f) {
        return;
    }
    int solIdx = atomicAdd(&nFourthSlides, 1);
    if (solIdx > MAX_FOURTH_SLIDES) {
        return;
    }
    struct MotionData4* data2 = &(fourthSlides[solIdx]);
    data2->identifier = index;
    data2->stickX = correct_stick(sticksol.stickX);
    data2->stickY = correct_stick(sticksol.stickY);
    data2->camAngle = camYaw;
    data2->facingAngle = fangle;
}


__global__ void third_move(int index, TargetLog target) {

    MotionData2* seconddata = &(secondSlides[index]);
    float pos[3];
    pos[0] = seconddata->nextPos[0];
    pos[1] = seconddata->nextPos[1];
    pos[2] = seconddata->nextPos[2];
    float vel = seconddata->nextVel;


    float lb = 0.9f;
    float ub = 0.94f;

    int camYaw = fix(crude_camera_yaw(pos, target.posCam));

    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    
    if (idx >= 20129 * lookupSize) {
        return;
    }
    int i = idx % 20129;
    int t = lookupTable[((idx - i) / 20129) % lookupSize];

    int fangle = fix(gArctanTableG[t] + camYaw);

    if (!stability_check(pos, vel, fangle)) {
        return;
    }

    FancySlideInfo thirdslide;
    if (!sim_slide(stickTabG[i], pos, vel, vel * gSineTableG[fangle >> 4], vel * gCosineTableG[fangle >> 4], fangle, fangle, camYaw, false, thirdslide)){
        return;
    }

    // in the air?

    if (assess_floor(thirdslide.endPos) != 1) {
        return;
    }

    //simulate air step.

    AirInfo thirdair;
    if(!sim_airstep(thirdslide.endPos, thirdslide.endSpeed, thirdslide.endFacingAngle, false, thirdair) || assess_floor(thirdair.endPos) != 2) {
        return;
    }

    float nextspeed = thirdair.endSpeed;
    // iterate over waiting frames. See if it's stable.
    for (int wf = 0; wf <= 3; wf++) {
        if (wf > 0) {
            nextspeed *= 0.98f;
        }
        if (!stability_check(thirdair.endPos, nextspeed, thirdslide.endFacingAngle)) {
            break;
        }
        /*if (fmaxf(lb * fabs(nextspeed), target.minSpeed) >= ub * fabs(nextspeed)) {
            break;
        }*/
        if (target.minSpeed >= ub * fabs(nextspeed)) {
            break;
        }

        float dis = find_dis(thirdair.endPos, target.posBully);
        // a better check for whether our speed is compatible with both making it to our target
        // AND making it to the target with the speed we need.
        if (dis < 0.25f * fmaxf(lb * fabs(nextspeed), target.minSpeed) || dis > 0.25 * ub * fabs(nextspeed)) {
            continue;
        }

        int solIdx = atomicAdd(&nThirdSlides, 1);
        if (solIdx > MAX_THIRD_SLIDES) {
            break;
        }
        struct MotionData13* data = &(thirdSlides[solIdx]);
        data->nextPos[0] = thirdair.endPos[0];
        data->nextPos[1] = thirdair.endPos[1];
        data->nextPos[2] = thirdair.endPos[2];
        data->nextVel = nextspeed;
        data->stickX = correct_stick(stickTabG[i].stickX);
        data->stickY = correct_stick(stickTabG[i].stickY);
        data->camAngle = camYaw;
        data->facingAngle = fangle;
        data->waitingFrames = wf;
    }
}


__global__ void second_move(int index, TargetLog target) {

    float lb = 0.9f * 0.9f * 0.98f * 0.98f * 0.98f;
    float ub = 0.94f * 0.94f;

    MotionData13* firstdata = &(firstSlides[index]);
    float pos[3];
    pos[0] = firstdata->nextPos[0];
    pos[1] = firstdata->nextPos[1];
    pos[2] = firstdata->nextPos[2];
    float vel = firstdata->nextVel;

    int camYaw = fix(crude_camera_yaw(pos, target.posCam));

    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= 20129 * 8192) {
        return;
    }
    int i = idx % 20129;
    int t = ((idx - i) / 20129) % 8192;

    int fangle = fix(gArctanTableG[t] + camYaw);

    if (!stability_check(pos, vel, fangle)) {
        return;
    }

    FancySlideInfo secondslide;
    if (!sim_slide(stickTabG[i], pos, vel, vel * gSineTableG[fangle >> 4], vel * gCosineTableG[fangle >> 4], fangle, fangle, camYaw, false, secondslide)){
        return;
    }

    // in the air?

    if (assess_floor(secondslide.endPos) != 1) {
        return;
    }

    //simulate air step.

    AirInfo secondair;
    if(!sim_airstep(secondslide.endPos, secondslide.endSpeed, secondslide.endFacingAngle, false, secondair) || assess_floor(secondair.endPos) != 2) {
        return;
    }

    float nextspeed = secondair.endSpeed;
    // iterate over waiting frames. See if it's stable.
    for (int wf = 0; wf <= 3; wf++) {
        if (wf > 0) {
            nextspeed *= 0.98f;
        }
        if (!stability_check(secondair.endPos, nextspeed, secondslide.endFacingAngle)) {
            break;
        }
        /*if (fmaxf(lb * fabs(nextspeed), target.minSpeed) >= ub * fabs(nextspeed)) {
            break;
        }*/
        if (target.minSpeed >= ub * fabs(nextspeed)) {
            break;
        }

        DonutData jelly = donut_compute(secondair.endPos, 0.9f * 0.5f * fabs(nextspeed), 0.94f * 0.5f * fabs(nextspeed), target.posBully, 0.25f * fmaxf(lb * fabs(nextspeed), target.minSpeed), 0.25f * ub * fabs(nextspeed));
        if (jelly.overlapArea < 10000.0f) {
            continue;
        }

        int solIdx = atomicAdd(&nSecondSlides, 1);
        if (solIdx > MAX_SECOND_SLIDES) {
            break;
        }
        struct MotionData2* data = &(secondSlides[solIdx]);
        data->nextPos[0] = secondair.endPos[0];
        data->nextPos[1] = secondair.endPos[1];
        data->nextPos[2] = secondair.endPos[2];
        data->nextVel = nextspeed;
        data->stickX = correct_stick(stickTabG[i].stickX);
        data->stickY = correct_stick(stickTabG[i].stickY);
        data->camAngle = camYaw;
        data->facingAngle = fangle;
        data->waitingFrames = wf;
        data->donut = jelly;
    }

}


__global__ void first_move(float posX, float posY, float posZ, float vel, TargetLog target) {

    float pos[3];
    pos[0] = posX;
    pos[1] = posY;
    pos[2] = posZ;

    float lb = 0.9f * 0.9f * 0.9f * 0.98f * 0.98f * 0.98f * 0.98f * 0.98f * 0.98f;
    float ub = 0.94f * 0.94f * 0.94f;

    int camYaw = fix(crude_camera_yaw(pos, target.posCam));

    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= 20129 * 8192) {
        return;
    }
    int i = idx % 20129;
    int t = ((idx - i) / 20129) % 8192;

    int fangle = fix(gArctanTableG[t] + camYaw);

    if (!stability_check(pos, vel, fangle)) {
        return;
    }
    
    FancySlideInfo firstslide;
    if (!sim_slide(stickTabG[i], pos, vel, vel * gSineTableG[fangle >> 4], vel * gCosineTableG[fangle >> 4], fangle, fangle, camYaw, true, firstslide)) {
        return;
    }

    // in the air?

    if (assess_floor(firstslide.endPos) != 1) {
        return;
    }

    //simulate air step.

    AirInfo firstair;
    if(!sim_airstep(firstslide.endPos, firstslide.endSpeed, firstslide.endFacingAngle, false, firstair) || assess_floor(firstair.endPos) != 2) {
        return;
    }

    float nextspeed = firstair.endSpeed;
    // iterate over waiting frames. See if it's stable.
    for (int wf = 0; wf <= 3; wf++) {
        if (wf > 0) {
            nextspeed *= 0.98f;
        }
        if (!stability_check(firstair.endPos, nextspeed, firstslide.endFacingAngle)) {
            break;
        }
        /*if (fmaxf(lb * fabs(nextspeed), target.minSpeed) >= ub * fabs(nextspeed)) {
            break;
        }*/
        if (target.minSpeed >= ub * fabs(nextspeed)) {
            break;
        }

        int solIdx = atomicAdd(&nFirstSlides, 1);
        if (solIdx > MAX_FIRST_SLIDES) {
            break;
        }
        struct MotionData13* data = &(firstSlides[solIdx]);
        data->nextPos[0] = firstair.endPos[0];
        data->nextPos[1] = firstair.endPos[1];
        data->nextPos[2] = firstair.endPos[2];
        data->nextVel = nextspeed;
        data->stickX = correct_stick(stickTabG[i].stickX);
        data->stickY = correct_stick(stickTabG[i].stickY);
        data->camAngle = camYaw;
        data->facingAngle = fangle;
        data->waitingFrames = wf;
    }
}


int main(int argc, char* argv[]) {

    // changeable
    float cameraPosition[3];
    cameraPosition[0] = -24910.0f;
    cameraPosition[1] = -2300.0f;
    cameraPosition[2] = 4440.0f;
    // changeable
    float firstPosition[3];
    firstPosition[0] = 5700.0f - (65536.0f * 7.0f);
    firstPosition[1] = -2917.0f;
    firstPosition[2] = 266.0f + (65536.0f * 1.0f);
    // changeable
    float firstSpeed = -1196779776.0f;
    // changeable
    float targetSpeed = 6.0e+08 * (73.0f / 53.0f);

    int nThreads = 256;

    std::string outFile = "lastData.csv";

    bool verbose = false;

    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "-h") || !strcmp(argv[i], "--help")) {
            printf("BitFS Second Phase PU Route Brute Forcer.\n");
            printf("This program accepts the following options:\n\n");
            printf("-ms <speed>: Speed that Mario starts out with.\n");
            printf("             Default: %f\n", firstSpeed);
            printf("-ts <speed>: Speed that Mario must end near.\n");
            printf("             Default: %d\n", targetSpeed);
            printf("-cp <pos_x> <pos_y> <pos_z>: Position of the camera.\n");
            printf("             Default: %f %f %f\n", cameraPosition[0], cameraPosition[1], cameraPosition[2]);
            printf("-fp <pos_x> <pos_y> <pos_z>: Mario's starting position.\n");
            printf("             Default: %f %f %f\n", firstPosition[0], firstPosition[1], firstPosition[2]);
            printf("-o: Path to the output file.\n");
            printf("    Default: %s\n", outFile.c_str());
            printf("-v: Verbose mode. Prints all parameters used in brute force.\n");
            printf("    Default: off\n");
            printf("-h --help: Prints this text.\n");
            exit(0);
        }
        else if (!strcmp(argv[i], "-ms")) {
            firstSpeed = std::stoi(argv[i + 1]);

            i += 1;
        }
        else if (!strcmp(argv[i], "-ts")) {
            targetSpeed = std::stoi(argv[i + 1]);

            i += 1;
        }
        else if (!strcmp(argv[i], "-cp")) {
            cameraPosition[0] = std::stoi(argv[i + 1]);
            cameraPosition[1] = std::stoi(argv[i + 2]);
            cameraPosition[2] = std::stoi(argv[i + 3]);

            i += 3;
        }
        else if (!strcmp(argv[i], "-fp")) {
            firstPosition[0] = std::stoi(argv[i + 1]);
            firstPosition[1] = std::stoi(argv[i + 2]);
            firstPosition[2] = std::stoi(argv[i + 3]);

            i += 3;
        }
        else if (!strcmp(argv[i], "-o")) {
            outFile = argv[i + 1];
            i += 1;
        }
        else if (!strcmp(argv[i], "-v")) {
            verbose = true;
        }
        if (verbose) {
            printf("Mario Starting Speed: %f\n", firstSpeed);
            printf("Target Speed: %d\n", targetSpeed);
            printf("Camera Position: (%f, %f, %f)\n", cameraPosition[0], cameraPosition[1], cameraPosition[2]);
            printf("First Position: (%f, %f, %f)\n", firstPosition[0], firstPosition[1], firstPosition[2]);
        }
    }

    // initialize the data we're starting off with

    TargetLog target;
    target.minSpeed = targetSpeed;
    target.posBully[0] = -1700.0f;
    target.posBully[1] = -2800.0f;
    target.posBully[2] = -350.0f;
    target.posCam[0] = cameraPosition[0];
    target.posCam[1] = cameraPosition[1];
    target.posCam[2] = cameraPosition[2];

    // initialize the stuff on the GPU

    struct MotionData13* firstSlidesGPU;
    cudaMalloc((void**)&firstSlidesGPU, MAX_FIRST_SLIDES * sizeof(struct MotionData13));
    struct MotionData2* secondSlidesGPU;
    cudaMalloc((void**)&secondSlidesGPU, MAX_SECOND_SLIDES * sizeof(struct MotionData2));
    struct MotionData13* thirdSlidesGPU;
    cudaMalloc((void**)&thirdSlidesGPU, MAX_THIRD_SLIDES * sizeof(struct MotionData13));
    struct MotionData4* fourthSlidesGPU;
    cudaMalloc((void**)&fourthSlidesGPU, MAX_FOURTH_SLIDES * sizeof(struct MotionData4));



    struct AllData* finalDataLogGPU;
    cudaMalloc((void**)&finalDataLogGPU, MAX_FOURTH_SLIDES * sizeof(struct AllData));


    copy_pointers_to_gpu << <1, 1 >> > (firstSlidesGPU, secondSlidesGPU, thirdSlidesGPU, fourthSlidesGPU, finalDataLogGPU);

    printf("Initializing Floors/Stick Tables...\n\n");

    initialise_floorsG << <1, 1 >> > ();
    init_stick_tablesG << <1, 1 >> > (false);


    int nFirstBlocks = (20129 * 8192 + nThreads - 1) / nThreads;
    int nFirstSlidesCPU = 0;

    printf("Starting First Move Computations...\n");

    cudaMemcpyToSymbol(nFirstSlides, &nFirstSlidesCPU, sizeof(int), 0, cudaMemcpyHostToDevice);
    first_move << <nFirstBlocks, nThreads >> > (firstPosition[0], firstPosition[1], firstPosition[2], firstSpeed, target);
    cudaMemcpyFromSymbol(&nFirstSlidesCPU, nFirstSlides, sizeof(int), 0, cudaMemcpyDeviceToHost);

    if (nFirstSlidesCPU == 0) {
        printf("No solutions found! Ending early!...");
        return 0;
    }
    if (nFirstSlidesCPU > MAX_FIRST_SLIDES) {
        fprintf(stderr, "Warning: The number of first slide paths has been exceeded. No more will be recorded. Increase the internal maximum to prevent this from happening.\n");
        nFirstSlidesCPU = MAX_FIRST_SLIDES;
    }

    printf("Found %d First Slides!\n\n", nFirstSlidesCPU);

    struct MotionData13* firstSlidesCPU = (struct MotionData13*)std::malloc(nFirstSlidesCPU * sizeof(struct MotionData13));
    cudaMemcpy(firstSlidesCPU, firstSlidesGPU, nFirstSlidesCPU * sizeof(struct MotionData13), cudaMemcpyDeviceToHost);

   /* std::cout << "Motion Data for the first id: \n";
    std::cout << "\tCam Angle: " << firstSlidesCPU[0].camAngle << "\n";
    std::cout << "\tFacing Angle: " << firstSlidesCPU[0].facingAngle << "\n";
    std::cout << "\tNext Pos: " << firstSlidesCPU[0].nextPos << "\n";
    std::cout << "\tNext Vel: " << firstSlidesCPU[0].nextVel << "\n";
    std::cout << "\tStick X, Y: " << firstSlidesCPU[0].stickX << ", " << firstSlidesCPU[0].stickY << "\n";
    std::cout << "\tWaiting Frames: " << firstSlidesCPU[0].waitingFrames << std::endl;*/

    for (int i = 0; i < 100/* nFirstSlidesCPU */; i++) {
        int hitCounter = 0;

        printf("id is %d\n", i);

        int nSecondBlocks = (20129 * 8192 + nThreads - 1) / nThreads;
        int nSecondSlidesCPU = 0;
        cudaMemcpyToSymbol(nSecondSlides, &nSecondSlidesCPU, sizeof(int), 0, cudaMemcpyHostToDevice);
        second_move << <nSecondBlocks, nThreads >> > (i, target);
        cudaMemcpyFromSymbol(&nSecondSlidesCPU, nSecondSlides, sizeof(int), 0, cudaMemcpyDeviceToHost);


        if (nSecondSlidesCPU == 0) {
            continue;
        }
        if (nSecondSlidesCPU > MAX_SECOND_SLIDES) {
            fprintf(stderr, "Warning: The number of second slide paths has been exceeded. No more will be recorded. Increase the internal maximum to prevent this from happening.\n");
            nFirstSlidesCPU = MAX_SECOND_SLIDES;
        }


        struct MotionData2* secondSlidesCPU = (struct MotionData2*)std::malloc(nSecondSlidesCPU * sizeof(struct MotionData2));
        cudaMemcpy(secondSlidesCPU, secondSlidesGPU, nSecondSlidesCPU * sizeof(struct MotionData2), cudaMemcpyDeviceToHost);

        for (int j = 0; j < nSecondSlidesCPU; j++) {
            // this is where things get a bit tricky. Our further progress depends on the details of the donut data
            // encoded in the second slide.
            // there's no clear mapping from the block and thread id we're on into the facing angle.
            // so, roughly, we'll have to take the second slide data, work out the camera yaw, build a boolean table
            // of which arctan table entries correspond to an angle that is acceptable according to the donut data, count
            // the number of entries in that boolean table, craft a lookup table to map the remainder of the id
            // to the arctan table entry, and then we'll be done.
            // step 1 is to initialize the table.
            int lookupSizeCPU = 0;
            table_builder << <1, 1 >> > (j, target);
            cudaMemcpyFromSymbol(&lookupSizeCPU, lookupSize, sizeof(int), 0, cudaMemcpyDeviceToHost);

            // now, the lookupTable maps an initial segment of the natural numbers to the entries in the arctan table
            // that correspond to a good facing angle that's compatible with the donut.


            int nThirdBlocks = (20129 * lookupSizeCPU + nThreads - 1) / nThreads;
            int nThirdSlidesCPU = 0;
            cudaMemcpyToSymbol(nThirdSlides, &nThirdSlidesCPU, sizeof(int), 0, cudaMemcpyHostToDevice);
            third_move << <nThirdBlocks, nThreads >> > (j, target);
            cudaMemcpyFromSymbol(&nThirdSlidesCPU, nThirdSlides, sizeof(int), 0, cudaMemcpyDeviceToHost);


            if (nThirdSlidesCPU == 0) {
                continue;
            }
            if (nThirdSlidesCPU > MAX_THIRD_SLIDES) {
                fprintf(stderr, "Warning: The number of third slide paths has been exceeded. No more will be recorded. Increase the internal maximum to prevent this from happening.\n");
                nThirdSlidesCPU = MAX_THIRD_SLIDES;
            }

            // we do not copy over the stuff to CPU here, because it is an inner loop and must go fast and we'll fetch
            // its information from the GPU later.

            int nFourthBlocks = (64 * nThirdSlidesCPU + nThreads - 1) / nThreads;
            int nFourthSlidesCPU = 0;
            cudaMemcpyToSymbol(nFourthSlides, &nFourthSlidesCPU, sizeof(int), 0, cudaMemcpyHostToDevice);
            fourth_move << <nFourthBlocks, nThreads >> > (target);
            cudaMemcpyFromSymbol(&nFourthSlidesCPU, nFourthSlides, sizeof(int), 0, cudaMemcpyDeviceToHost);


            if (nFourthSlidesCPU == 0) {
                continue;
            }
            if (nFourthSlidesCPU > MAX_FOURTH_SLIDES) {
                fprintf(stderr, "Warning: The number of fourth slide paths has been exceeded. No more will be recorded. Increase the internal maximum to prevent this from happening.\n");
                nFourthSlidesCPU = MAX_FOURTH_SLIDES;
            }

            hitCounter += nFourthSlidesCPU;
            //printf("hit!\n");
            // we now get everything into the proper struct format.
            rewrite_structs << <1, 1 >> > (i, j, target, firstPosition[0], firstPosition[1], firstPosition[2], firstSpeed);
            //printf("%i\n", nFourthSlidesCPU);
        }
        // free up memory
        std::free(secondSlidesCPU);
        printf("Hits: %i\n\n", hitCounter);
    }
    // free up memory
    std::free(firstSlidesCPU);

    // figure out how many solutions we have
    printf("writing to file...\n");
    int nSolutionsCPU = 0;
    cudaMemcpyFromSymbol(&nSolutionsCPU, nSolutions, sizeof(int), 0, cudaMemcpyDeviceToHost);
    printf("%d solutions found!\n", nSolutionsCPU);

    // get the solutions from the GPU to the CPU
    struct AllData* finalDataLog = (struct AllData*)std::malloc(nSolutionsCPU * sizeof(struct AllData));
    cudaMemcpy(finalDataLog, finalDataLogGPU, nSolutionsCPU * sizeof(struct AllData), cudaMemcpyDeviceToHost);

    // ok, at this point all our GPU shit is over and we've got 2000 or fewer solutions in a table. It's time
    // to start writing this shit into a file.
    std::ofstream wf(outFile);
    wf << std::fixed;
    wf << "Camera Position X, Camera Position Y, Camera Position Z, ";
    wf << "First Position X, First Position Y, First Position Z, ";
    wf << "First Facing Angle, First Cam Angle, First Velocity, First Stick X, First Stick Y, First Landing Frames, ";
    wf << "Second Position X, Second Position Y, Second Position Z, ";
    wf << "Second Facing Angle, Second Cam Angle, Second Velocity, Second Stick X, Second Stick Y, Second Landing Frames, ";
    wf << "Third Position X, Third Position Y, Third Position Z, ";
    wf << "Third Facing Angle, Third Cam Angle, Third Velocity, Third Stick X, Third Stick Y, Third Landing Frames, ";
    wf << "Fourth Position X, Fourth Position Y, Fourth Position Z, ";
    wf << "Fourth Facing Angle, Fourth Cam Angle, Fourth Velocity, Fourth Stick X, Fourth Stick Y, ";
    wf << "Bully Position X, Bully Position Y, BullyPosition Z, ";
    wf << "Bully Angle, Bully Velocity" << std::endl;


    for (int k = 0; k < nSolutionsCPU; k++) {
        wf << finalDataLog[k].positions.posCam[0] << ", " << finalDataLog[k].positions.posCam[1] << ", " << finalDataLog[k].positions.posCam[2] << ", ";
        wf << finalDataLog[k].positions.pos21[0] << ", " << finalDataLog[k].positions.pos21[1] << ", " << finalDataLog[k].positions.pos21[2] << ", ";
        wf << finalDataLog[k].angles.facing21 << ", " << finalDataLog[k].angles.cam21 << ", " << finalDataLog[k].velocities.vel21 << ", " << finalDataLog[k].sticks.stick21X << ", " << finalDataLog[k].sticks.stick21Y << ", " << finalDataLog[k].waits.waiting21 << ", ";
        wf << finalDataLog[k].positions.pos22[0] << ", " << finalDataLog[k].positions.pos22[1] << ", " << finalDataLog[k].positions.pos22[2] << ", ";
        wf << finalDataLog[k].angles.facing22 << ", " << finalDataLog[k].angles.cam22 << ", " << finalDataLog[k].velocities.vel22 << ", " << finalDataLog[k].sticks.stick22X << ", " << finalDataLog[k].sticks.stick22Y << ", " << finalDataLog[k].waits.waiting22 << ", ";
        wf << finalDataLog[k].positions.pos23[0] << ", " << finalDataLog[k].positions.pos23[1] << ", " << finalDataLog[k].positions.pos23[2] << ", ";
        wf << finalDataLog[k].angles.facing23 << ", " << finalDataLog[k].angles.cam23 << ", " << finalDataLog[k].velocities.vel23 << ", " << finalDataLog[k].sticks.stick23X << ", " << finalDataLog[k].sticks.stick23Y << ", " << finalDataLog[k].waits.waiting23 << ", ";
        wf << finalDataLog[k].positions.pos24[0] << ", " << finalDataLog[k].positions.pos24[1] << ", " << finalDataLog[k].positions.pos24[2] << ", ";
        wf << finalDataLog[k].angles.facing24 << ", " << finalDataLog[k].angles.cam24 << ", " << finalDataLog[k].velocities.vel24 << ", " << finalDataLog[k].sticks.stick24X << ", " << finalDataLog[k].sticks.stick24Y << ", ";
        wf << finalDataLog[k].bully.posBully[0] << ", " << finalDataLog[k].bully.posBully[1] << ", " << finalDataLog[k].bully.posBully[2] << ", ";
        wf << finalDataLog[k].bully.angle << ", " << finalDataLog[k].bully.velBully << std::endl;
    }
    wf.close();

    std::free(finalDataLog);
    // free up some memory now!

    cudaFree(firstSlidesGPU);
    cudaFree(secondSlidesGPU);
    cudaFree(thirdSlidesGPU);
    cudaFree(fourthSlidesGPU);
    cudaFree(finalDataLogGPU);

    printf("complete!");

    return 0;
}
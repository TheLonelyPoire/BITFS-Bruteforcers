#include <fstream>
#include <cstring>
#include <string>
#include <unordered_map>
#include <iostream>
#include <cmath>
  
#include "../Common/CommonBruteforcerStructs.hpp"
#include "../Common/Camera.cuh"
#include "../Common/Donut.cuh"
#include "../Common/Floor.cuh"
#include "../Common/Movement.cuh"
#include "../Common/Stick.cuh"
#include "../Common/Surface.cuh"
#include "../Common/Trig.cuh"
#include "../Common/VMath.cuh"

#include "LowSpeedRouteStructs.hpp"

# define M_PI            3.14159265358979323846  /* pi */
   
using namespace BITFS;

bool attainableArctans[8192];

int keyFloors[7][4][2];
int keyCenter[7][2];
float thatNearOneConstant;


void initialise_keyFloors() {
    keyFloors[0][0][0] = (int)round(floors[21].min_x);
    keyFloors[0][0][1] = (int)round(floors[21].min_z);
    keyFloors[0][1][0] = (int)round(floors[21].min_x);
    keyFloors[0][1][1] = (int)round(floors[21].max_z);
    keyFloors[0][2][0] = (int)round(floors[21].max_x);
    keyFloors[0][2][1] = (int)round(floors[21].min_z);
    keyFloors[0][3][0] = (int)round(floors[21].max_x);
    keyFloors[0][3][1] = (int)round(floors[21].max_z);
    keyFloors[1][0][0] = (int)round(floors[26].min_x);
    keyFloors[1][0][1] = (int)round(floors[26].min_z);
    keyFloors[1][1][0] = (int)round(floors[26].min_x);
    keyFloors[1][1][1] = (int)round(floors[26].max_z);
    keyFloors[1][2][0] = (int)round(floors[26].max_x);
    keyFloors[1][2][1] = (int)round(floors[26].min_z);
    keyFloors[1][3][0] = (int)round(floors[26].max_x);
    keyFloors[1][3][1] = (int)round(floors[26].max_z);
    keyFloors[2][0][0] = (int)round(floors[23].min_x);
    keyFloors[2][0][1] = (int)round(floors[23].min_z);
    keyFloors[2][1][0] = (int)round(floors[23].min_x);
    keyFloors[2][1][1] = (int)round(floors[23].max_z);
    keyFloors[2][2][0] = (int)round(floors[23].max_x);
    keyFloors[2][2][1] = (int)round(floors[23].min_z);
    keyFloors[2][3][0] = (int)round(floors[23].max_x);
    keyFloors[2][3][1] = (int)round(floors[23].max_z);
    keyFloors[3][0][0] = (int)round(floors[29].min_x);
    keyFloors[3][0][1] = (int)round(floors[29].min_z);
    keyFloors[3][1][0] = (int)round(floors[29].min_x);
    keyFloors[3][1][1] = (int)round(floors[29].max_z);
    keyFloors[3][2][0] = (int)round(floors[29].max_x);
    keyFloors[3][2][1] = (int)round(floors[29].min_z);
    keyFloors[3][3][0] = (int)round(floors[29].max_x);
    keyFloors[3][3][1] = (int)round(floors[29].max_z);
    keyFloors[4][0][0] = (int)round(floors[19].min_x);
    keyFloors[4][0][1] = (int)round(floors[19].min_z);
    keyFloors[4][1][0] = (int)round(floors[19].min_x);
    keyFloors[4][1][1] = (int)round(floors[19].max_z);
    keyFloors[4][2][0] = (int)round(floors[19].max_x);
    keyFloors[4][2][1] = (int)round(floors[19].min_z);
    keyFloors[4][3][0] = (int)round(floors[19].max_x);
    keyFloors[4][3][1] = (int)round(floors[19].max_z);
    keyFloors[5][0][0] = (int)round(floors[15].min_x);
    keyFloors[5][0][1] = (int)round(floors[15].min_z);
    keyFloors[5][1][0] = (int)round(floors[15].min_x);
    keyFloors[5][1][1] = (int)round(floors[15].max_z);
    keyFloors[5][2][0] = (int)round(floors[15].max_x);
    keyFloors[5][2][1] = (int)round(floors[15].min_z);
    keyFloors[5][3][0] = (int)round(floors[15].max_x);
    keyFloors[5][3][1] = (int)round(floors[15].max_z);
    keyFloors[6][0][0] = (int)round(floors[10].min_x);
    keyFloors[6][0][1] = (int)round(floors[8].min_z);
    keyFloors[6][1][0] = (int)round(floors[10].min_x);
    keyFloors[6][1][1] = (int)round(floors[13].max_z);
    keyFloors[6][2][0] = (int)round(floors[8].max_x);
    keyFloors[6][2][1] = (int)round(floors[8].min_z);
    keyFloors[6][3][0] = (int)round(floors[8].max_x);
    keyFloors[6][3][1] = (int)round(floors[13].max_z);
    
    keyCenter[0][0] = (int)round(0.5f * floors[21].min_x + 0.5f * floors[21].max_x);
    keyCenter[0][1] = (int)round(0.5f * floors[21].min_z + 0.5f * floors[21].max_z);
    keyCenter[1][0] = (int)round(0.5f * floors[26].min_x + 0.5f * floors[26].max_x);
    keyCenter[1][1] = (int)round(0.5f * floors[26].min_z + 0.5f * floors[26].max_z);
    keyCenter[2][0] = (int)round(0.5f * floors[23].min_x + 0.5f * floors[23].max_x);
    keyCenter[2][1] = (int)round(0.5f * floors[23].min_z + 0.5f * floors[23].max_z);
    keyCenter[3][0] = (int)round(0.5f * floors[29].min_x + 0.5f * floors[29].max_x);
    keyCenter[3][1] = (int)round(0.5f * floors[29].min_z + 0.5f * floors[29].max_z);
    keyCenter[4][0] = (int)round(0.5f * floors[19].min_x + 0.5f * floors[19].max_x);
    keyCenter[4][1] = (int)round(0.5f * floors[19].min_z + 0.5f * floors[19].max_z);
    keyCenter[5][0] = (int)round(0.5f * floors[15].min_x + 0.5f * floors[15].max_x);
    keyCenter[5][1] = (int)round(0.5f * floors[15].min_z + 0.5f * floors[15].max_z);
    keyCenter[6][0] = (int)round(0.5f * floors[10].min_x + 0.5f * floors[8].max_x);
    keyCenter[6][1] = (int)round(0.5f * floors[8].min_z  + 0.5f * floors[13].max_z);
    
    thatNearOneConstant = floors[30].normal[1];
}


void init_attainable_arctans() {
    for (int t = 0; t < 8192; t++) {
        attainableArctans[t] = false;
    }
    for (int i = 0; i < 20129; i++) {
        attainableArctans[gReverseArctanTable[fix(stickTab[i].angle)]] = true;
    }
}


//Thank you Spud!! Our precious baby function, needed for getting the range of camera yaws on the pole.
// it relies on the current floor Y, and the base camera yaw,
// but for the current floor Y... it isn't really needed.
// and for the base camera yaw, we just do the shitty approximation where the camera is pointed directly at the pole
// because that gets things right *enough*.
int fine_camera_yaw(float* currentPosition, float* lakituPosition, short faceAngle) {
    float baseCameraDist = 1400.0;
    short baseCameraPitch = 0x05B0;
    short baseCameraYaw = -8192;

    //SurfaceG* floor;
    //float floorY;

    //float xOff = currentPosition[0] + gSineTableG[fix((int)baseCameraYaw) >> 4] * 40.f;
    //float zOff = currentPosition[2] + gCosineTableG[fix((int)baseCameraYaw) >> 4] * 40.f;
    //float offPos[3] = { xOff, currentPosition[1], zOff };

    //int floorIdx = find_floorG(offPos, &floor, floorY, floorsG, total_floorsG);
    //floorY = floorY - currentPosition[1];

    //if (floorIdx != -1) {
        //if (floorY > 0) {
            //if (!(floor->normal[2] == 0.f && floorY < 100.f)) {
                //baseCameraPitch += atan2sG(40.f, floorY);
            //}
        //}
    //}
    
    // so, why was all this commented out? Well, offPos is just going to basically be on the pole platform, because
    // 40 units off isn't that much. And the height is the current height.
    // finding the floor would find the floor of the pole platform.
    // floorY would yield a negative number since we're on the pole, which is higher than the pole platform.
    // and so, the if statement is skipped.

    //float posMul = 1.f;
    //float posBound = 1200.f; //pole
    //float focMul = 0.9f;
    //float focBound = 200.f;

    //float posY = (currentFloorY - currentPosition[1]) * posMul;

    //if (posY > posBound) {
        //posY = posBound;
    //}

    //if (posY < -posBound) {
        //posY = -posBound;
    //}

    //float focusY = (currentFloorY - currentPosition[1]) * focMul;

    //if (focusY > focBound) {
        //focusY = focBound;
    //}

    //if (focusY < -focBound) {
        //focusY = -focBound;
    //}
    
    // see, all the above segment was skipped because we're probably pretty close to the ground, but
    // have an unknown level of closeness. Focus updates pretty fast, faster than pan
    // so we can safely assume we're pretty damn close to the ground. So especially posY can be assumed to be near 0
    // while focusY is decently close to 0, enough to be neglected, I think. They will be negative.

    baseCameraPitch = baseCameraPitch + 2304;

    float cameraPos[3] = { currentPosition[0] + baseCameraDist * gCosineTableG[fix((int)baseCameraPitch) >> 4] * gSineTableG[fix((int)baseCameraYaw) >> 4],
                       currentPosition[1] + 125.0f + baseCameraDist * gSineTableG[fix((int)baseCameraPitch) >> 4],
                       currentPosition[2] + baseCameraDist * gCosineTableG[fix((int)baseCameraPitch) >> 4] * gCosineTableG[fix((int)baseCameraYaw) >> 4]
    };
    // posY was removed from the camera position height because it's close to 0.

    float pan[3] = { 0, 0, 0 };
    float temp[3] = { 0, 0, 0 };

    // Get distance and angle from camera to Mario.
    float dx = currentPosition[0] - cameraPos[0];
    float dy = currentPosition[1] - cameraPos[1];
    float dz = currentPosition[2] - cameraPos[2];

    float cameraDist = sqrtf(dx * dx + dy * dy + dz * dz);
    float cameraPitch = atan2s(sqrtf(dx * dx + dz * dz), dy);
    float cameraYaw = atan2s(dz, dx);

    // The camera will pan ahead up to about 30% of the camera's distance to Mario.
    pan[2] = gSineTableG[0xC0] * cameraDist;

    temp[0] = pan[0];
    temp[1] = pan[1];
    temp[2] = pan[2];

    pan[0] = temp[2] * gSineTableG[fix((int)faceAngle) >> 4] + temp[0] * gCosineTableG[fix((int)faceAngle) >> 4];
    pan[2] = temp[2] * gCosineTableG[fix((int)faceAngle) >> 4] - temp[0] * gSineTableG[fix((int)faceAngle) >> 4];

    // rotate in the opposite direction
    cameraYaw = -cameraYaw;

    temp[0] = pan[0];
    temp[1] = pan[1];
    temp[2] = pan[2];

    pan[0] = temp[2] * gSineTableG[fix((int)cameraYaw) >> 4] + temp[0] * gCosineTableG[fix((int)cameraYaw) >> 4];
    pan[2] = temp[2] * gCosineTableG[fix((int)cameraYaw) >> 4] - temp[0] * gSineTableG[fix((int)cameraYaw) >> 4];

    // Only pan left or right
    pan[2] = 0.f;

    cameraYaw = -cameraYaw;

    temp[0] = -pan[0]; //pole
    temp[1] = pan[1];
    temp[2] = pan[2];

    pan[0] = temp[2] * gSineTableG[fix((int)cameraYaw) >> 4] + temp[0] * gCosineTableG[fix((int)cameraYaw) >> 4];
    pan[2] = temp[2] * gCosineTableG[fix((int)cameraYaw) >> 4] - temp[0] * gSineTableG[fix((int)cameraYaw) >> 4];

    float cameraFocus[3] = { currentPosition[0] + pan[0], currentPosition[1] + 125.0f + pan[1], currentPosition[2] + pan[2] };
    // focusY removed because it's probably 0-ish.
    // and makes negligible difference on the camera yaw.

    dx = cameraFocus[0] - lakituPosition[0];
    dy = cameraFocus[1] - lakituPosition[1];
    dz = cameraFocus[2] - lakituPosition[2];

    cameraDist = sqrtf(dx * dx + dy * dy + dz * dz);
    cameraPitch = atan2s(sqrtf(dx * dx + dz * dz), dy);
    cameraYaw = atan2s(dz, dx);

    if (cameraPitch > 15872) {
        cameraPitch = 15872;
    }
    if (cameraPitch < -15872) {
        cameraPitch = -15872;
    }

    cameraFocus[0] = lakituPosition[0] + cameraDist * gCosineTableG[fix((int)cameraPitch) >> 4] * gSineTableG[fix((int)cameraYaw) >> 4];
    cameraFocus[1] = lakituPosition[1] + cameraDist * gSineTableG[fix((int)cameraPitch) >> 4];
    cameraFocus[2] = lakituPosition[2] + cameraDist * gCosineTableG[fix((int)cameraPitch) >> 4] * gCosineTableG[fix((int)cameraYaw) >> 4];

    return atan2s(lakituPosition[2] - cameraFocus[2], lakituPosition[0] - cameraFocus[0]);
}


// computes 1QF of crouchslide, given stick position, starting position, starting speed, starting facing angle, camera angle.
// might be inaccurate, takes some shortcuts.
// note that it takes the floor normal as a parameter, this is so we don't have to keep recomputing that shit.
SlideInfo sim_slide(StickTableData stick, float* startPos, float startSpeed, int startAngle, int camera, float* slope) {
    
    // initialize some variables we'll be reusing because they get updated
    float nextPos[3];
    nextPos[0] = startPos[0];
    nextPos[1] = startPos[1];
    nextPos[2] = startPos[2];
    float speed = startSpeed;
    int facingAngle = fix(startAngle);
    int cameraYaw = camera;
    
    int slopeAngle = atan2s(slope[2], slope[0]);
    slopeAngle = fix(slopeAngle);
    float steepness = sqrtf(slope[0] * slope[0] + slope[2] * slope[2]);
    
    float intendedMag = ((stick.magnitude / 64.0f) * (stick.magnitude / 64.0f)) * 32.0f;
    
    // and the angle stuff
    int intendedDYaw = stick.angle + cameraYaw - facingAngle;
    intendedDYaw = fix(intendedDYaw);
    float forward = gCosineTableG[intendedDYaw >> 4];
    float sideward = gSineTableG[intendedDYaw >> 4];
    
    float lossFactor = intendedMag / 32.0f * forward * 0.02f + 0.92f;
    
    // and the speed updating from sliding stuff
    float velX = speed * gSineTableG[facingAngle >> 4];
    float velZ = speed * gCosineTableG[facingAngle >> 4];
    float oldSpeed = sqrtf(velX * velX + velZ * velZ);
    velX += velZ * (intendedMag / 32.0f) * sideward * 0.05f;
    velZ -= velX * (intendedMag / 32.0f) * sideward * 0.05f;
    float newSpeed = sqrtf(velX * velX + velZ * velZ);
    velX = velX * oldSpeed / newSpeed;
    velZ = velZ * oldSpeed / newSpeed;
    velX += 7.0f * steepness * gSineTableG[slopeAngle >> 4];
    velZ += 7.0f * steepness * gCosineTableG[slopeAngle >> 4];
    velX *= lossFactor;
    velZ *= lossFactor;
    
    nextPos[0] = nextPos[0] + slope[1] * (velX / 4.0f);
    nextPos[2] = nextPos[2] + slope[1] * (velZ / 4.0f);
    
    // and update the sliding yaw
    int slidingYaw = atan2s(velZ, velX);
    speed = -sqrtf(velX * velX + velZ * velZ);
    
    // and your new angle
    facingAngle = slidingYaw + 0x8000;
    facingAngle = fix(facingAngle);
    
    // and return the result
    struct SlideInfo solution;
    solution.endAngle = facingAngle;
    solution.endSpeed = speed;
    solution.endPos[0] = nextPos[0];
    solution.endPos[1] = nextPos[1];
    solution.endPos[2] = nextPos[2];

    return solution;
}

// computes 1QF of crouchslide, given stick position, starting position, starting speed, starting facing angle, camera angle.
// will be totally accurate.
FancySlideInfo precise_sim_slide(StickTableData stick, float* startPos, float startSpeed, float vX, float vZ, int faceAngle, int slideYaw, int camera, bool polePlat) {
    
    // initialize some variables we'll be reusing because they get updated
    float nextPos[3];
    nextPos[0] = startPos[0];
    nextPos[1] = startPos[1];
    nextPos[2] = startPos[2];
    float speed = startSpeed;
    int facingAngle = fix(faceAngle);
    int slidingYaw = fix(slideYaw);
    int cameraYaw = camera;
    
    float intendedMag = ((stick.magnitude / 64.0f) * (stick.magnitude / 64.0f)) * 32.0f;
    
    // and the angle stuff
    int intendedDYaw = stick.angle + cameraYaw - slidingYaw;
    intendedDYaw = fix(intendedDYaw);
    float forward = gCosineTableG[intendedDYaw >> 4];
    float sideward = gSineTableG[intendedDYaw >> 4];
    
    // and the 10k loss factor stuff.
    if (forward < 0.0f && speed >= 0.0f) {
        forward *= 0.5f + 0.5f * speed / 100.0f;
    }
    float lossFactor = intendedMag / 32.0f * forward * 0.02f + 0.92f;
    
    // and the speed updating from sliding stuff
    float velX = vX;
    float velZ = vZ;
    float oldSpeed = sqrtf(velX * velX + velZ * velZ);
    velX += velZ * (intendedMag / 32.0f) * sideward * 0.05f;
    velZ -= velX * (intendedMag / 32.0f) * sideward * 0.05f;
    float newSpeed = sqrtf(velX * velX + velZ * velZ);
    velX = velX * oldSpeed / newSpeed;
    velZ = velZ * oldSpeed / newSpeed;
    
    // we need this because the pole platform has that 0.999999 thing for its slope.
    if(polePlat) {
        velX *= lossFactor;
        velZ *= lossFactor;
        nextPos[0] = nextPos[0] + thatNearOneConstant * (velX / 4.0f);
        nextPos[2] = nextPos[2] + thatNearOneConstant * (velZ / 4.0f);
    }
    else {
        SurfaceG* floor;
        float floorHeight;
        int floorIdx = find_floor(nextPos, &floor, floorHeight, floorsG, total_floorsG);
        int slopeAngle = atan2sG(floor->normal[2], floor->normal[0]);
        slopeAngle = fix(slopeAngle);
        float steepness = sqrtf(floor->normal[0] * floor->normal[0] + floor->normal[2] * floor->normal[2]);
        velX += 7.0f * steepness * gSineTableG[slopeAngle >> 4];
        velZ += 7.0f * steepness * gCosineTableG[slopeAngle >> 4];
        velX *= lossFactor;
        velZ *= lossFactor;
        nextPos[0] = nextPos[0] + floor->normal[1] * (velX / 4.0f);
        nextPos[2] = nextPos[2] + floor->normal[1] * (velZ / 4.0f);
    }
    
    // and update the sliding yaw
    slidingYaw = atan2sG(velZ, velX);
    int newFacingDYaw = (short)(facingAngle - slidingYaw);
    if (newFacingDYaw > 0 && newFacingDYaw <= 0x4000) {
        if ((newFacingDYaw -= 0x200) < 0) {
            newFacingDYaw = 0;
        }
    }
    else if (newFacingDYaw > -0x4000 && newFacingDYaw < 0) {
        if ((newFacingDYaw += 0x200) > 0) {
            newFacingDYaw = 0;
        }
    }
    else if (newFacingDYaw > 0x4000 && newFacingDYaw < 0x8000) {
        if ((newFacingDYaw += 0x200) > 0x8000) {
            newFacingDYaw = 0x8000;
        }
    }
    else if (newFacingDYaw > -0x8000 && newFacingDYaw < -0x4000) {
        if ((newFacingDYaw -= 0x200) < -0x8000) {
            newFacingDYaw = -0x8000;
        }
    }
    
    // adjust your outgoing speed, now that the X and Z velocities are pinned down
    if (newFacingDYaw > -0x4000 && newFacingDYaw < 0x4000) {
        speed = sqrtf(velX * velX + velZ * velZ);
    }
    else {
        speed = -sqrtf(velX * velX + velZ * velZ);
    }
    
    // and your new angle
    facingAngle = slidingYaw + newFacingDYaw;
    facingAngle = fix(facingAngle);
    slidingYaw = fix(slidingYaw);
    
    // and return the result
    struct FancySlideInfo solution;
    solution.endFacingAngle = facingAngle;
    solution.endSlidingAngle = slidingYaw;
    solution.endSpeed = speed;
    solution.endPos[0] = nextPos[0];
    solution.endPos[1] = nextPos[1];
    solution.endPos[2] = nextPos[2];

    return solution;
}

// burns speed while walking against OOB for a given number of frames.
float speed_burn(float speed, int frames) {
    float finalvel = speed;
    for (int i = 1; i <= frames; i++) {
        finalvel += 1.1f;
    }
    return finalvel;
}


bool second_tenk_compute(AllData data, AllData* dataPoint) {
    
    // alright, so here's the deal. We can compute a rough approximation of the base camera yaw.
    // or feed it in directly, idk
    // and then we iterate through viable facing angles to get a maximum and minimum camera yaw.
    // and then we search to see which of the numbers in that range is a true camera yaw!
    // and then we can check all of those guys.
    int maxCamYaw = 0;
    int minCamYaw = 65536;
    for (int t = -32768; t < 32768; t++) {
        int cyaw = fine_camera_yaw(data.positions.posPole, data.positions.posCam1, (short)t);
        // -8192 is a guess.
        if (fix(cyaw) > maxCamYaw) {
            maxCamYaw = fix(cyaw);
        }
        if (fix(cyaw) < minCamYaw) {
            minCamYaw = fix(cyaw);
        }
    }
    // ok, max and min camera yaws are found. Next is to iterate through them and, very importantly, CHECK
    // whether it's a viable camera yaw that could possibly exist.
    for (int camYaw = minCamYaw; camYaw <= maxCamYaw; camYaw++) {
        if (!validCameraAngles[camYaw]) {
            continue;
        }
        for (int i = 0; i < 20129; i++) {
        
            FancySlideInfo poleslide = precise_sim_slide(stickTab[i], data.positions.posPole, fminf((stickTab[i].magnitude / 64.0f) * (stickTab[i].magnitude / 64.0f) * 32.0f, 8.0f), data.velocities.poleVelX, data.velocities.poleVelZ, stickTab[i].angle + camYaw, data.angles.slidePole, camYaw, true);
        
            if(poleslide.endSpeed < 0.0f) {
                continue;
            }
            if (assess_floor (poleslide.endPos) != 1) {
                continue;
            }
            bool noGround = true;
            AirInfo crudepoleair;
            for (int qf = 1; qf <= 4; qf++) {
                 if(!sim_airstep(((qf == 1) ? poleslide.endPos : crudepoleair.endPos), ((qf == 1) ? poleslide.endSpeed : crudepoleair.endSpeed), poleslide.endFacingAngle, (qf == 1), crudepoleair) || assess_floor(crudepoleair.endPos) == 0) {
                    break;
                }
                else if(assess_floor(crudepoleair.endPos) == 2 || assess_floor(crudepoleair.endPos) == 3) {
                    noGround = false;
                    break;
                }
            }
            if (noGround) {
                continue;
            }
            if (!on_one_up(crudepoleair.endPos)) {
                continue;
            }
            // and at this point, you know that you hit the 1-up.
            float cameraFocus[3];
            cameraFocus[0] = 0.8f * poleslide.endPos[0] + 0.2f * data.positions.posPole[0];
            cameraFocus[1] = 0.3f * poleslide.endPos[1] + 0.7f * data.positions.posPole[1];
            cameraFocus[2] = 0.8f * poleslide.endPos[2] + 0.2f * data.positions.posPole[2];
            int camYawTen = fix(crude_camera_yaw(cameraFocus, data.positions.posCam1));
            // the camera is a bit badly simulated because of panning stuff, but it's accurate enough for our purposes.
            // badly simulate the 10k, mainly because we have a horrifying amount of strainings to get through.
            for (int j = 0; j < 20129; j++) {
                FancySlideInfo crudetenk = precise_sim_slide(stickTab[j], crudepoleair.endPos, crudepoleair.endSpeed, crudepoleair.endSpeed * gSineTable[poleslide.endFacingAngle >> 4], crudepoleair.endSpeed * gCosineTable[poleslide.endFacingAngle >> 4], poleslide.endFacingAngle, poleslide.endSlidingAngle, camYawTen, false);
                if (crudetenk.endSpeed < -2.147e+09 || crudetenk.endSpeed > -1.5e+09){
                    continue;
                }
                // and now we can simulate all the fucking straining.
                for (int k = 0; k < 20129; k++) {
                    int intendedDYaw = fix(stickTab[k].angle + camYaw - poleslide.endFacingAngle);
                    float intendedMag = ((stickTab[k].magnitude / 64.0f) * (stickTab[k].magnitude / 64.0f)) * 32.0f;
                    float speed = poleslide.endSpeed - 0.35f;
                    speed += (intendedMag / 32.0f) * gCosineTable[intendedDYaw >> 4] * 1.5f;
                    float sidewaysSpeed = (intendedMag / 32.0f) * gSineTable[intendedDYaw >> 4] * 10.0f;
                    speed -= 1.0f;
                    float airVelX = speed * gSineTable[poleslide.endFacingAngle >> 4];
                    float airVelZ = speed * gCosineTable[poleslide.endFacingAngle >> 4];
                    airVelX += sidewaysSpeed * gSineTable[((poleslide.endFacingAngle + 16384) % 65536) >> 4];
                    airVelZ += sidewaysSpeed * gCosineTable[((poleslide.endFacingAngle + 16384) % 65536) >> 4];

                    float nextPos[3];
                    nextPos[0] = poleslide.endPos[0];
                    nextPos[1] = poleslide.endPos[1];
                    nextPos[2] = poleslide.endPos[2];
                    bool landMiss = true;
                    for (int quart = 1; quart <= 4; quart++) {
                        nextPos[0] += airVelX / 4.0f;
                        nextPos[2] += airVelZ / 4.0f;
                        if (assess_floor(nextPos) == 0) {
                            break;
                        }
                        else if (assess_floor(nextPos) == 2 || assess_floor(nextPos) == 3) {
                            landMiss = false;
                            Surface* landingFloor;
                            float landingHeight;
                            int floorIdx = find_floor(nextPos, &landingFloor, landingHeight, floors, total_floors);
                            nextPos[1] = landingHeight;
                            break;
                        } 
                    }
                    if (landMiss) {
                        continue;
                    }
                    if (!on_one_up(nextPos)) {
                        continue;
                    }
                    // at this point, we know that the straining hits the 1-up platform.
                    FancySlideInfo finetenk = precise_sim_slide(stickTab[j], nextPos, speed, airVelX, airVelZ, poleslide.endFacingAngle, poleslide.endSlidingAngle, camYawTen, false);
                    if (finetenk.endSpeed < -2.147e+09 || finetenk.endSpeed > -1.5e+09){
                        continue;
                    }
                    if (assess_floor(finetenk.endPos) != 1) {
                        continue;
                    }
                    AirInfo tenkair;
                    if(!sim_airstep(finetenk.endPos, finetenk.endSpeed, finetenk.endFacingAngle, true, tenkair) || (assess_floor(tenkair.endPos) != 2 && assess_floor(tenkair.endPos) != 3)) {
                        continue;
                    }
                    if (!stability_check(tenkair.endPos, tenkair.endSpeed, finetenk.endFacingAngle)) {             
                        continue;         
                    }
                    // uncomment and modify in case you want to ensure you land on a specific piece of floor.
                    // 27 and 30 is the id for the most convenient slice of floor to land on, for later PU routing.
                
                    //SurfaceG* floor;
                    //float floorheight;
                    //int floorIdx = find_floor(tenkair.endPos, &floor, floorheight, floorsG, total_floorsG);
                
                    //if (floorIdx != 27 && floorIdx != 30) {
                        //continue;
                    //}

                    (dataPoint->positions).posTenK2Air[0] = poleslide.endPos[0];
                    (dataPoint->positions).posTenK2Air[1] = poleslide.endPos[1];
                    (dataPoint->positions).posTenK2Air[2] = poleslide.endPos[2];
                    (dataPoint->positions).posTenK2[0] = nextPos[0];
                    (dataPoint->positions).posTenK2[1] = nextPos[1];
                    (dataPoint->positions).posTenK2[2] = nextPos[2];
                    (dataPoint->positions).pos21[0] = tenkair.endPos[0];
                    (dataPoint->positions).pos21[1] = tenkair.endPos[1];
                    (dataPoint->positions).pos21[2] = tenkair.endPos[2];
                    (dataPoint->velocities).velTenK2X = airVelX;
                    (dataPoint->velocities).velTenK2Z = airVelZ;
                    (dataPoint->velocities).velTenK2 = speed;
                    (dataPoint->velocities).vel21 = tenkair.endSpeed;
                    (dataPoint->sticks).stickPoleX = correct_stick(stickTab[i].stickX);
                    (dataPoint->sticks).stickPoleY = correct_stick(stickTab[i].stickY);
                    (dataPoint->sticks).stickStrainX = correct_stick(stickTab[k].stickX);
                    (dataPoint->sticks).stickStrainY = correct_stick(stickTab[k].stickY);
                    (dataPoint->sticks).stickTenK2X = correct_stick(stickTab[j].stickX);
                    (dataPoint->sticks).stickTenK2Y = correct_stick(stickTab[j].stickY);
                    (dataPoint->angles).camPole = camYaw;
                    (dataPoint->angles).camTenK2 = camYawTen;
                    (dataPoint->angles).facingTenK2 = poleslide.endFacingAngle;
                    (dataPoint->angles).slideTenK2 = poleslide.endSlidingAngle;
                    printf("10K'd\n");
                    return true;
                }
            } 
        } 
    }
    return false;
}

bool move14(AllData data, AllData* dataPoint) {
    
    int camYaw = fix(crude_camera_yaw(data.positions.pos14, data.positions.posCam1));
    // we're going to do kind of a weird thing here. Namely, instead of iterating over HAU's and stick positions, we iterate
    // over HAU's and waiting frames!! But first, we work out exactly where we be aiming.
    float aimPoint[3];
    aimPoint[0] = data.positions.posPole[0] - (float)(data.targets.speed) * gSineTable[data.targets.hau];
    aimPoint[2] = data.positions.posPole[2] - (float)(data.targets.speed) * gCosineTable[data.targets.hau];
    // and how far to where we're aiming
    float dis = find_dis(data.positions.pos14, aimPoint);
    // and make damn sure that "we attain our exact needed speed" gives you enough distance to make it close to the aiming point
    Surface* floor;
    float floorheight;
    int floorIdx = find_floor(data.positions.pos14, &floor, floorheight, floors, total_floors);
    if (fabs(dis - (fabs((float)(data.targets.speed)) * floor->normal[1] / 4.0f)) > 150.0f) {
        return false;
    }
    // work out the direction from the pole to where you are now, as that approximates your facing angle
    int direction = atan2s(data.positions.pos14[2] - data.positions.posPole[2], data.positions.pos14[0] - data.positions.posPole[0]);
    // do our black magic computation for which AU's are attainable.     
    int dist13 = fmin(fix((direction - (16 * 16)) - (direction + (16 * 16))), fix((direction + (16 * 16)) - (direction - (16 * 16))));
    for (int t = 0; t < 8192; t++) {
        
        if (!attainableArctans[t]) {
            continue;
        }
        int fangle = fix(gArctanTableG[t] + camYaw);
        
        int dist23 = fmin(fix(fangle - (direction - (16 * 16))), fix((direction - (16 * 16)) - fangle));
        int dist12 = fmin(fix(fangle - (direction + (16 * 16))), fix((direction + (16 * 16)) - fangle));
        if (dist12 + dist23 > dist13) {
            continue;
        }
        // ok, the AU is attainable, is it stable?
        if (!stability_check(data.positions.pos14, data.velocities.vel14Arrive, fangle)) {
            continue;
        }
        // log the speed you arrived with.
        float speed = data.velocities.vel14Arrive;
        // iterate over how long to wait. Long wait times can be sped up with ya wa hoo
        for (int delay = 0; delay <= 600; delay++) {
            if (delay > 0) {
                speed += 1.1f;
            }
            // try to solve for the stick position that hits the aim point
            StickTableData sticksol = infer_stick(data.positions.pos14, aimPoint, speed, fangle, camYaw);
            // check to see that it's feasible
            if(sticksol.magnitude >= 64.0f) {
                continue;
            }
            // simulate the slide
            FancySlideInfo lastslide = precise_sim_slide(sticksol, data.positions.pos14, speed, speed * gSineTable[fangle >> 4], speed * gCosineTable[fangle >> 4], fangle, fangle, camYaw, false);
            // must travel along the needed HAU going in
            if (lastslide.endFacingAngle >> 4 != data.targets.hau) {
                continue;
            }
            // figure out how far you are from your aim point, as that closely approximates how far you are from the pole.
            // you can snag the pole at 100-something units from the pole, so that's why we're checking that dis is low.
            // also make sure you're in the air.
            float dis = find_dis(lastslide.endPos, aimPoint);
            if (dis >= 150.0f || assess_floor(lastslide.endPos) != 1) {
                continue;
            }
            // simulate the air movement and make sure it's within 1.5 speed of your target (number pulled out of ass)
            AirInfo airmove;
            if(!sim_airstep(lastslide.endPos, lastslide.endSpeed, lastslide.endFacingAngle, true, airmove))
                continue;
            if (fabs(airmove.endSpeed - (float)(data.targets.speed)) >= 1.5f) {
                continue;
            }
            float nextPos[3];
            nextPos[0] = airmove.endPos[0];
            nextPos[1] = airmove.endPos[1];
            nextPos[2] = airmove.endPos[2];
            bool survives = true;
            if (assess_floor(nextPos) != 1 && assess_floor(nextPos) != 4) {
                    survives = false;
            }
            // simulate the rest of the air steps.
            for (int j = 2; j <= 4; j++) {
                nextPos[0] += airmove.endSpeed * gSineTable[lastslide.endFacingAngle >> 4] / 4.0f;
                nextPos[2] += airmove.endSpeed * gCosineTable[lastslide.endFacingAngle >> 4] / 4.0f;
                if (assess_floor(nextPos) != 1 && assess_floor(nextPos) != 4) {
                    survives = false;
                    break;
                }
            }
            // go back if you didn't survive (ie, have all air frames)
            if (!survives) {
                continue;
            }
            // check how much you missed the pole by.
            float miss = find_dis(nextPos, data.positions.posPole);
            if (miss > 117.0f) {
                continue;
            }
            // log data
            printf("ROUTE FOUND\n");
            (dataPoint->velocities).vel14 = speed;
            (dataPoint->velocities).poleVelX = airmove.endSpeed * gSineTable[lastslide.endFacingAngle >> 4];
            (dataPoint->velocities).poleVelZ = airmove.endSpeed * gCosineTable[lastslide.endFacingAngle >> 4];
            (dataPoint->sticks).stick14X = correct_stick(sticksol.stickX);
            (dataPoint->sticks).stick14Y = correct_stick(sticksol.stickY);
            (dataPoint->angles).facing14 = fangle;
            (dataPoint->angles).cam14 = camYaw;
            (dataPoint->angles).slidePole = lastslide.endSlidingAngle;
            (dataPoint->waits).delayFrames = delay + 20;
            if(second_tenk_compute(*dataPoint, dataPoint)) {
                    return true;
            }
        }  
    }
    return false;
}

bool move13(AllData data, AllData* dataPoint) {
    
    int camYaw = fix(crude_camera_yaw(data.positions.pos13, data.positions.posCam1));
    // the destination is the center of the platform you're headed for, in the PU you're headed for.
    float destination[3];
    destination[0] = keyCenter[data.targets.platKey][0] + data.targets.inPUX * 65536.0f;
    destination[2] = keyCenter[data.targets.platKey][1] + data.targets.inPUZ * 65536.0f;
    // direction is a good approximation of what your facing angle should be.
    int direction = atan2s(data.positions.pos13[2] - destination[2], data.positions.pos13[0] - destination[0]);
    // fetch the slope data to accelerate crouchslide computation.
    Surface* startFloor;
    float startheight;
    int floorIdx = find_floor(data.positions.pos13, &startFloor, startheight, floors, total_floors);
    // circle black magic for which AU to check.
    int dist13 = fmin(fix((direction - (32 * 16)) - (direction + (32 * 16))), fix((direction + (32 * 16)) - (direction - (32 * 16))));
    for (int t = 0; t < 8192; t++) {
        
        if (!attainableArctans[t]) {
            continue;
        }
        int fangle = fix(gArctanTableG[t] + camYaw);
        
        int dist23 = fmin(fix(fangle - (direction - (32 * 16))), fix((direction - (32 * 16)) - fangle));
        int dist12 = fmin(fix(fangle - (direction + (32 * 16))), fix((direction + (32 * 16)) - fangle));
        if (dist12 + dist23 > dist13) {
            continue;
        }
        // check if it's stable
        if (!stability_check(data.positions.pos13, data.velocities.vel13, fangle)) {
            continue;
        }
        // iterate over stick positions
        for (int i = 0; i < 20129; i++) {
            // simulate the slide
            SlideInfo crudeslide = sim_slide(stickTab[i], data.positions.pos13, data.velocities.vel13, fangle, camYaw, startFloor->normal);
            // did it land in air?
            if (assess_floor(crudeslide.endPos) != 1) {
                continue;
            }
            // simulate air movement
            AirInfo airmove;

            // did it land on ground?
            if(!sim_airstep(crudeslide.endPos, crudeslide.endSpeed, crudeslide.endAngle, true, airmove) || (assess_floor(airmove.endPos) != 2 && assess_floor(airmove.endPos) != 3)) {
                continue;
            }
            // did you hit the right PU?
            if((int)round(airmove.endPos[0] / 65536.0f) != data.targets.inPUX) {
                continue;
            }
            if((int)round(airmove.endPos[2] / 65536.0f) != data.targets.inPUZ) {
                continue;
            }
            // check to see if the floor you landed on is associated with the ID of the platform you're aiming for.
            Surface* floor;
            float floorheight;
            int floorIdx = find_floor(airmove.endPos, &floor, floorheight, floors, total_floors);
            if(data.targets.platKey == 0 && floorIdx != 20 && floorIdx != 21) {
                continue;
            }
            if(data.targets.platKey == 1 && floorIdx != 25 && floorIdx != 26) {
                continue;
            }
            if(data.targets.platKey == 2 && floorIdx != 23 && floorIdx != 24) {
                continue;
            }
            if(data.targets.platKey == 3 && floorIdx != 28 && floorIdx != 29) {
                continue;
            }
            if(data.targets.platKey == 4 && floorIdx != 18 && floorIdx != 19) {
                continue;
            }
            if(data.targets.platKey == 5 && floorIdx != 14 && floorIdx != 15) {
                continue;
            }
            if(data.targets.platKey == 6 && (floorIdx < 8 || floorIdx > 13)) {
                continue;
            }
            // if you made it this far, recompute with precision
            FancySlideInfo fineslide = precise_sim_slide(stickTab[i], data.positions.pos13, data.velocities.vel13, data.velocities.vel13 * gSineTable[fangle >> 4], data.velocities.vel13 * gCosineTable[fangle >> 4], fangle, fangle, camYaw, false);
            // if the angles match up, the precision recomputation validates the crappy one.
            if (fineslide.endFacingAngle != crudeslide.endAngle) {
                continue;
            }
            float nextspeed = airmove.endSpeed;
            // iterate over waiting frames.
            for (int wf = 0; wf <= 3; wf++) {
                if (wf > 0) {
                    nextspeed *= 0.98f;
                }
                // are we stable against OOB?
                bool stable = stability_check(airmove.endPos, nextspeed, crudeslide.endAngle);
                // also, if the speed we're trying to hit is above 0.94x our current speed, then that's bad
                // and waiting longer won't help.
                if (!stable || fabs((float)(data.targets.speed)) > 0.94f * fabs(nextspeed)) {
                    break;
                }
                // if the speed we're trying to hit is below 0.9x our current speed, that's bad, add more waiting frames.
                if(fabs((float)(data.targets.speed)) < 0.9f * fabs(nextspeed)) {
                    continue;
                }
                // log data.
                (dataPoint->positions).pos14[0] = airmove.endPos[0];
                (dataPoint->positions).pos14[1] = airmove.endPos[1];
                (dataPoint->positions).pos14[2] = airmove.endPos[2];
                (dataPoint->velocities).vel14Arrive = speed_burn(nextspeed, 20);
                (dataPoint->sticks).stick13X = correct_stick(stickTab[i].stickX);
                (dataPoint->sticks).stick13Y = correct_stick(stickTab[i].stickY);
                (dataPoint->angles).facing13 = fangle;
                (dataPoint->angles).cam13 = camYaw;
                (dataPoint->waits).waiting13 = wf;
                printf("t");
                if(move14(*dataPoint, dataPoint)) {
                    return true;
                }
            }
        }
    }
    return false;
}

bool move12(AllData data, AllData* dataPoint) {
    float lb = 0.9f;
    float ub = 0.94f;
    int camYaw = fix(crude_camera_yaw(data.positions.pos12, data.positions.posCam1));
    // cache the slope data of where we're starting so we don't have to recompute it in all crouchslides.
    Surface* startFloor;
    float startheight;
    int floorIdx = find_floor(data.positions.pos12, &startFloor, startheight, floors, total_floors);
    // the below for loop is because basically, the PU's that have a shot at the main universe are in the overlap of two donuts
    // the donut of PU's we can reach, and the donut of PU's that can 1fcs to the main universe within our speed band.
    // so we've got two "islands" of PU's we're iterating over.
    for (int a = 0; a <= 1; a++) {
        // Specifying the island we're aiming for means we can use the angle data from our donut computation to figure out
        // the range of angles we're interested in. Warning. These HAU's might be negative. This is intended.
        int dist13 = fmin(fix(data.donuts.hauBand[a][0] * 16 - data.donuts.hauBand[a][1] * 16), fix(data.donuts.hauBand[a][1] * 16 - data.donuts.hauBand[a][0] * 16));
        // the above is some black magic for testing whether our angle will be in the hau band.
        for (int t = 0; t < 8192; t++) {
        
            if (!attainableArctans[t]) {
                continue;
            }
            int fangle = fix(gArctanTableG[t] + camYaw);
            // black magic time.
            int dist23 = fmin(fix(fangle - data.donuts.hauBand[a][1] * 16), fix(data.donuts.hauBand[a][1] * 16 - fangle));
            int dist12 = fmin(fix(fangle - data.donuts.hauBand[a][0] * 16), fix(data.donuts.hauBand[a][0] * 16 - fangle));
            if (dist12 + dist23 > dist13) {
                continue;
            }
            // is the facing angle stable?
            if (!stability_check(data.positions.pos12, data.velocities.vel12, fangle)) {
                continue;
            }
            // iterate over stick positions
            for (int i = 0; i < 20129; i++) {
                // simulate the slide
                SlideInfo crudeslide = sim_slide(stickTab[i], data.positions.pos12, data.velocities.vel12, fangle, camYaw, startFloor->normal);
                // is it in the air?
                if (assess_floor(crudeslide.endPos) != 1) {
                    continue;
                }
                // simulate air movement
                AirInfo airmove;
                
                if (!sim_airstep(crudeslide.endPos, crudeslide.endSpeed, crudeslide.endAngle, true, airmove)) {
                    continue;
                }
                // is it on the ground?
                if(assess_floor(airmove.endPos) != 2 && assess_floor(airmove.endPos) != 3) {
                    continue;
                }
                // did you stably land?
                if (!stability_check(airmove.endPos, airmove.endSpeed, crudeslide.endAngle)) {
                    continue;
                }
                // fancily recompute the angles, see if they match the crappy simulation of the slide.
                FancySlideInfo fineslide = precise_sim_slide(stickTab[i], data.positions.pos12, data.velocities.vel12, data.velocities.vel12 * gSineTable[fangle >> 4], data.velocities.vel12 * gCosineTable[fangle >> 4], fangle, fangle, camYaw, false);
                if (fineslide.endFacingAngle != crudeslide.endAngle) {
                    continue;
                }
                // ok, the angles match up. Time to check if we're on the right ground.
                Surface* floor;
                float floorheight;
                int floorIdx = find_floor(airmove.endPos, &floor, floorheight, floors, total_floors);
                
                
                // If you know exactly what you're doing, you can fuck with this line (and its analogues in other functions)
                // to accelerate the search.
                // roughly, floor ID 28 and 29 corresponds to the ramp up from the lava near the pole
                // which has a bafflingly easy time of hitting targets across the whole level compared to other positions
                // you could be in.
                // and so, many routes will involve 28/29 until you can snag your given target.
                // roughly, this is a "fast compute but might miss stuff" vs "slow compute that won't miss stuff" dial.
            
                //if (floorIdx < 8 || floorIdx > 32 || floorIdx == 22) {
                    //continue;
                //}
                if (floorheight > -2866.0f) {
                    continue;
                }
                if (floorIdx != 28 && floorIdx != 29) {
                    continue;
                }
                
                
                float nextspeed = airmove.endSpeed;
                // simulate waiting frames
                for (int wf = 0; wf <= 3; wf++) {
                    if (wf > 0) {
                        nextspeed *= 0.98f;
                    }
                    // make sure you're stable against OOB
                    if ((!stability_check(airmove.endPos, airmove.endSpeed, crudeslide.endAngle)) && wf > 0) {
                        break;
                    }
                    // basically, if the speeds are incompatible with getting to the target with our needed speed, give up
                    if (fmaxf(lb * fabs(nextspeed), fabs((float)(data.targets.speed)) / (0.94f)) > fminf(ub * fabs(nextspeed), fabs((float)(data.targets.speed)) / (0.9f * 0.98f * 0.98f * 0.98f))) {
                        break;
                    }
                    // work out where you're aiming for specifically
                    float destination[3];
                    destination[0] = keyCenter[data.targets.platKey][0] + data.targets.inPUX * 65536.0f;
                    destination[2] = keyCenter[data.targets.platKey][1] + data.targets.inPUZ * 65536.0f;
                    
                    float dis = find_dis(airmove.endPos, destination);
                    // a better check for whether our speed is compatible with both making it to our target
                    // AND making it to the target with the speed we need.
                    if (dis < ((1.0f + floor->normal[1]) / 4.0f) * fmaxf(lb * fabs(nextspeed), fabs((float)(data.targets.speed)) / (0.94f)) || dis > ((1.0f + floor->normal[1]) / 4.0f) * fminf(ub * fabs(nextspeed), fabs((float)(data.targets.speed)) / (0.9f * 0.98f * 0.98f * 0.98f))) {
                        continue;
                    }
                    // the counter can be uncommented and a print statement added if you want to see.
                    // otherwise, just log the data, proceed to the last step.
                    (dataPoint->positions).pos13[0] = airmove.endPos[0];
                    (dataPoint->positions).pos13[1] = airmove.endPos[1];
                    (dataPoint->positions).pos13[2] = airmove.endPos[2];
                    (dataPoint->velocities).vel13 = speed_burn(nextspeed, 20);
                    (dataPoint->sticks).stick12X = correct_stick(stickTab[i].stickX);
                    (dataPoint->sticks).stick12Y = correct_stick(stickTab[i].stickY);
                    (dataPoint->angles).facing12 = fangle;
                    (dataPoint->angles).cam12 = camYaw;
                    (dataPoint->waits).waiting12 = wf;
                    printf("s\n");
                    if(move13(*dataPoint, dataPoint)) {
                        return true;
                    }
                }
            }
        }
    }
    return false;
}

bool move11(AllData data, AllData* dataPoint) {
    float lb = 0.9f * 0.9f * 0.98f * 0.98f * 0.98f;
    float ub = 0.94f * 0.94f;
    int counter = 0;
    // keeping track of how many hits we have for our first motion is useful for partially running a computation
    // logging where you were, and skipping ahead.
    int camYaw = fix(crude_camera_yaw(data.positions.pos11, data.positions.posCam1));
    // fetch the slope data from the floor so we don't have to endlessly recompute that shit.
    Surface* startFloor;
    float startheight;
    int floorIdx = find_floor(data.positions.pos11, &startFloor, startheight, floors, total_floors);
    // iterate over AU's
    for (int t = 0; t < 8192; t++) {
        // can we do that AU?
        if (!attainableArctans[t]) {
            continue;
        }
        int fangle = fix(gArctanTableG[t] + camYaw);
        // is it stable?
        if (!stability_check(data.positions.pos11, data.velocities.vel11, fangle)) {
            continue;
        }
        // iterate over sticks
        for (int i = 0; i < 20129; i++) {
            // simulate crouchslide.
            SlideInfo crudeslide = sim_slide(stickTab[i], data.positions.pos11, data.velocities.vel11, fangle, camYaw, startFloor->normal);
            if (assess_floor(crudeslide.endPos) != 1) {
                continue;
            }
            //simulate air steps.
            AirInfo airmove;
            bool goodAir = false;
            for (int qf = 1; qf <= 4; qf++) {
                if (!sim_airstep(((qf == 1) ? crudeslide.endPos : airmove.endPos), ((qf == 1) ? crudeslide.endSpeed : airmove.endSpeed), crudeslide.endAngle, (qf == 1), airmove) || assess_floor(airmove.endPos) == 0) {
                    break;
                }
                if (assess_floor(airmove.endPos) == 2 || assess_floor(airmove.endPos) == 3) {
                    goodAir = true;
                    break;
                }
            }
            if (!goodAir) {
                continue;
            }
            // stable landing spot?
            if (!stability_check(airmove.endPos, airmove.endSpeed, crudeslide.endAngle)) {
                continue;
            }
            // we must check that we've landed on an PU of the same parity as our target, it's a severe constraint.
            if (((int)(round(airmove.endPos[0] / 65536.0f)) % 2) != (data.targets.inPUX % 2) || ((int)(round(airmove.endPos[2] / 65536.0f)) % 2) != (data.targets.inPUZ % 2)) {
                continue;
            }
            // ok, since everything appears in fine working order, we resimulate from scratch. The only difference between
            // sim_slide and precise_sim_slide is that the latter handles 10k's (not relevant here), and the latter is
            // pickier about what your end facing angle is.
            FancySlideInfo fineslide = precise_sim_slide(stickTab[i], data.positions.pos11, data.velocities.vel11, data.velocities.vel11 * gSineTable[fangle >> 4], data.velocities.vel11 * gCosineTable[fangle >> 4], fangle, fangle, camYaw, false); // TODO: Check this
            // if the angle matches, it validates.
            if (fineslide.endFacingAngle != crudeslide.endAngle) {
                continue;
            }
            // Now time to check if we're on the right ground.
            Surface* floor;
            float floorheight;
            int floorIdx = find_floor(airmove.endPos, &floor, floorheight, floors, total_floors);
            
            
            // If you know exactly what you're doing, you can fuck with this line (and its analogues in other functions)
            // to accelerate the search.
            // roughly, floor ID 28 and 29 corresponds to the ramp up from the lava near the pole
            // which has a bafflingly easy time of hitting targets across the whole level compared to other positions
            // you could be in.
            // and so, many routes will involve 28/29 until you can snag your given target.
            // roughly, this is a "fast compute but might miss stuff" vs "slow compute that won't miss stuff" dial.
            
            //if (floorIdx < 8 || floorIdx > 32 || floorIdx == 22) {
                //continue;
            //}
            if (floorheight > -2866.0f) {
                continue;
            }
            if (floorIdx != 28 && floorIdx != 29) {
                continue;
            }
            
            
            float nextspeed = airmove.endSpeed;
            // iterate over waiting frames.
            for (int wf = 0; wf <= 3; wf++) {
                if (wf > 0) {
                    nextspeed *= 0.98f;
                }
                // is it stable?
                if ((!stability_check(airmove.endPos, nextspeed, crudeslide.endAngle)) && wf > 0) {
                    break;
                }
                // check to see if our speed is acceptable, though admittedly this is pretty fucking arcane.
                if (fmaxf(lb * fabs(nextspeed), fabs((float)(data.targets.speed)) / 0.94f) >= fminf(ub * fabs(nextspeed), fabs((float)(data.targets.speed)) / (0.98f * 0.98f * 0.98f * 0.9f))) {
                    break;
                }
                // where we're aiming for.
                float destination[3];
                destination[0] = keyCenter[data.targets.platKey][0] + data.targets.inPUX * 65536.0f;
                destination[2] = keyCenter[data.targets.platKey][1] + data.targets.inPUZ * 65536.0f;
                
                // computing the donut of "where we can reach" and "where can reach our target with the needed speed"
                // so we can aim for the intersection.
                DonutData glazed = donut_compute(airmove.endPos, 0.9f * ((1.0f + floor->normal[1]) / 4.0f) * fabs(nextspeed), 0.94f * ((1.0f + floor->normal[1]) / 4.0f) * fabs(nextspeed), destination, ((1.0f + 0.98f) / 4.0f) * fmaxf(lb * fabs(nextspeed), fabs((float)(data.targets.speed)) / 0.94f), ((1.0f + 1.0f) / 4.0f) * fminf(ub * fabs(nextspeed), fabs((float)(data.targets.speed)) / (0.98f * 0.98f * 0.98f * 0.9f)));
                if(glazed.overlapArea < 10000.0f) {
                    continue;
                }
                // log data, continue.
                printf("f(%d,%d)\n", counter, fangle);
                (dataPoint->positions).pos12[0] = airmove.endPos[0];
                (dataPoint->positions).pos12[1] = airmove.endPos[1];
                (dataPoint->positions).pos12[2] = airmove.endPos[2];
                (dataPoint->velocities).vel12 = speed_burn(nextspeed, 20);
                (dataPoint->sticks).stick11X = correct_stick(stickTab[i].stickX);
                (dataPoint->sticks).stick11Y = correct_stick(stickTab[i].stickY);
                (dataPoint->angles).facing11 = fangle;
                (dataPoint->angles).cam11 = camYaw;
                (dataPoint->waits).waiting11 = wf;
                (dataPoint->donuts) = glazed;
                counter++;
                // fuck with the counter to continue with partially terminated computations.
                if(counter > -1) {
                    if(move12(*dataPoint, dataPoint)) {
                        return true;
                    }
                }
            }
        }
    }
    return false;
}


int main(int argc, char* argv[]) {
  
    // changeable
    float cameraPosition[3];
    cameraPosition[0] = -1700.0f;
    cameraPosition[1] = -2300.0f;
    cameraPosition[2] = 1200.0f;
    // immutable
    float polePosition[3];
    polePosition[0] = 6605.0f;
    polePosition[1] = -2970.0f;
    polePosition[2] = 266.0f;
    // changeable
    float firstPosition[3];
    firstPosition[0] = 5700.0f - (65536.0f * 29.0f);
    firstPosition[1] = -2917.0f;
    firstPosition[2] = 266.0f;
    // changeable
    float firstSpeed = -5981800.0f;
    // changeable
    int targetPlat = 1;
    int targetPUX = -80;
    int targetPUZ = 0;
    // for best results, pick 2 speed closer to 0 than the first output you got from the pole thing
    // gives you more robust solutions that way.
    int targetSpeed = -4204757;
    int targetHAU = 3072;
       
    std::string outFile = "outData.csv";
    
    bool verbose = false;
    
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "-h") || !strcmp(argv[i], "--help")) {
            printf("BitFS First Phase PU Route Brute Forcer.\n");
            printf("This program accepts the following options:\n\n");
            printf("-ms <speed>: Speed that Mario starts out with.\n");
            printf("             Default: %f\n", firstSpeed);
            printf("-ts <speed>: Speed that Mario must end near.\n");
            printf("             Default: %d\n", targetSpeed);
            printf("-cp <pos_x> <pos_y> <pos_z>: Position of the camera.\n");
            printf("             Default: %f %f %f\n", cameraPosition[0], cameraPosition[1], cameraPosition[2]);
            printf("-fp <pos_x> <pos_y> <pos_z>: Mario's starting position.\n");
            printf("             Default: %f %f %f\n", firstPosition[0], firstPosition[1], firstPosition[2]);
            printf("-ti <targ_id> <pu_x> <pu_z> <hau>: Optimization target.\n");
            printf("             Default: %d %d %d %d\n", targetPlat, targetPUX, targetPUZ, targetHAU);
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
        else if (!strcmp(argv[i], "-ti")) {
            targetPlat = std::stoi(argv[i + 1]);
            targetPUX = std::stoi(argv[i + 2]);
            targetPUZ = std::stoi(argv[i + 3]);
            targetHAU = std::stoi(argv[i + 4]);

            i += 4;
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
            printf("Targets(platID, PUX, PUZ, HAU): (%d, %d, %d, %d)\n", targetPlat, targetPUX, targetPUZ, targetHAU);
        }
    }
    
    // initialize the tables
    
    AllData data;
    init_reverse_atan();
    initialise_floors();
    initialise_keyFloors();
    init_stick_tables();
    init_attainable_arctans(); 
    init_camera_angles();
    
    // and the data
    
    data.positions.posCam1[0] = cameraPosition[0];
    data.positions.posCam1[1] = cameraPosition[1];
    data.positions.posCam1[2] = cameraPosition[2];
    data.positions.pos11[0] = firstPosition[0];
    data.positions.pos11[1] = firstPosition[1];
    data.positions.pos11[2] = firstPosition[2];
    data.positions.posPole[0] = polePosition[0];
    data.positions.posPole[1] = polePosition[1];
    data.positions.posPole[2] = polePosition[2];
    data.targets.platKey = targetPlat;
    data.targets.inPUX = targetPUX;
    data.targets.inPUZ = targetPUZ;
    data.targets.speed = targetSpeed;
    data.targets.hau = targetHAU;
    data.velocities.vel11 = firstSpeed;
    

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
    wf << "Fourth Facing Angle, Fourth Cam Angle, Fourth Velocity, Fourth Stick X, Fourth Stick Y, Fourth Waiting Frames, ";
    wf << "Pole Velocity X, Pole Velocity Z, Pole Slide Yaw, Pole Cam Angle, Pole Stick X, Pole Stick Y, ";
    wf << "Frame1 Position X, Frame1 Position Y, Frame1 Position Z, ";
    wf << "10k Position X, 10k Position Y, 10k Position Z, ";
    wf << "10k Facing Angle, 10k Cam Angle, 10k Slide Yaw, 10k Velocity, 10k Velocity X, 10k Velocity Z, ";
    
    wf << "Strain Stick X, Strain Stick Y, 10k Stick X, 10k Stick Y, ";
    
    wf << "Final Position X, Final Position Y, Final Position Z, ";
    wf << "Final Speed" << std::endl;
    
    if(move11(data, &data)) {
        wf << data.positions.posCam1[0] <<", " << data.positions.posCam1[1] <<", " << data.positions.posCam1[2] <<", ";
        wf << data.positions.pos11[0] <<", " << data.positions.pos11[1] <<", " << data.positions.pos11[2] <<", ";
        wf << data.angles.facing11 <<", " << data.angles.cam11 <<", " << data.velocities.vel11 <<", " << data.sticks.stick11X <<", " << data.sticks.stick11Y <<", " << data.waits.waiting11 <<", ";
        wf << data.positions.pos12[0] <<", " << data.positions.pos12[1] <<", " << data.positions.pos12[2] <<", ";
        wf << data.angles.facing12 <<", " << data.angles.cam12 <<", " << data.velocities.vel12 <<", " << data.sticks.stick12X <<", " << data.sticks.stick12Y <<", " << data.waits.waiting12 <<", ";
        wf << data.positions.pos13[0] <<", " << data.positions.pos13[1] <<", " << data.positions.pos13[2] <<", ";
        wf << data.angles.facing13 <<", " << data.angles.cam13 <<", " << data.velocities.vel13 <<", " << data.sticks.stick13X <<", " << data.sticks.stick13Y <<", " << data.waits.waiting13 <<", ";
        wf << data.positions.pos14[0] <<", " << data.positions.pos14[1] <<", " << data.positions.pos14[2] <<", ";
        wf << data.angles.facing14 <<", " << data.angles.cam14 <<", " << data.velocities.vel14 <<", " << data.sticks.stick14X <<", " << data.sticks.stick14Y <<", " << data.waits.delayFrames <<", ";
        wf << data.velocities.poleVelX <<", " << data.velocities.poleVelZ <<", " << data.angles.slidePole <<", " << data.angles.camPole <<", " << data.sticks.stickPoleX <<", " << data.sticks.stickPoleY <<", ";
        wf << data.positions.posTenK2Air[0] <<", " << data.positions.posTenK2Air[1] <<", " << data.positions.posTenK2Air[2] <<", ";
        wf << data.positions.posTenK2[0] <<", " << data.positions.posTenK2[1] <<", " << data.positions.posTenK2[2] <<", ";
        wf << data.angles.facingTenK2 <<", " << data.angles.camTenK2 <<", " << data.angles.slideTenK2 <<", " << data.velocities.velTenK2 <<", " << data.velocities.velTenK2X <<", " << data.velocities.velTenK2Z <<", ";
        wf << data.sticks.stickStrainX <<", " << data.sticks.stickStrainY <<", " << data.sticks.stickTenK2X <<", " << data.sticks.stickTenK2Y <<", ";
        wf << data.positions.pos21[0] <<", " << data.positions.pos21[1] <<", " << data.positions.pos21[2] <<", ";
        wf << data.velocities.vel21 << std::endl;
        wf.close();
        return 0;
    }
    else {
        printf("FAILURE");
        return 0;
    }
}
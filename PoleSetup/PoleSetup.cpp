// roughly, what this is supposed to do is, given a camera position, find a viable platform/PU target, and exact approach speed + AU, such that a second 10k setup exists.
#include <fstream>
#include <cstring>
#include <string>
#include <unordered_map>
#include <iostream>
#include <cmath>
// stuff
#include <ostream>
#include <iomanip>
#include <cstdlib>
#include <unordered_set>

#include "../Common/CommonBruteforcerStructs.hpp"
#include "../Common/Camera.cuh"
#include "../Common/Floors.cuh"
#include "../Common/Movement.cuh"
#include "../Common/Stick.cuh"
#include "../Common/Surface.cuh"
#include "../Common/Trig.cuh"

#include "PoleSetupStructs.hpp"

using namespace BITFS;

const int tableOfStrainsSize = 20129;
StrainInfo tableOfStrains[tableOfStrainsSize];

int16_t a = 0;

bool fine_check(AllData* dataPoint, float* trueFocus, float sPanDistance, float* camPos) {
    float fvelocity = (float)dataPoint->targets.speed;
    // step 1: find the vX and vZ from your speed and hau you approached the pole with.
    float vX = fvelocity * gSineTable[dataPoint->targets.hau];
    float vZ = fvelocity * gCosineTable[dataPoint->targets.hau];
    // and the position you're aiming at with your crouchslide.
    float aimPos[3];
    aimPos[0] = dataPoint->positions.posPole[0] - vX;
    aimPos[1] = -2700.0f;
    aimPos[2] = dataPoint->positions.posPole[2] - vZ;
    // and an approximation for where we're starting from, with the given platform and PU.
    float center[3];
    center[0] = (float)(65536 * dataPoint->targets.inPUX + keyCenter[dataPoint->targets.platKey][0]);
    center[1] = -2700.0f;
    center[2] = (float)(65536 * dataPoint->targets.inPUZ + keyCenter[dataPoint->targets.platKey][1]);
    // would the crouchslide end up with us moving along the correct HAU? If not, fail.
    if(!(angle_match(center, aimPos, dataPoint->targets.hau))) {
        return false;
    }
    // try all four corners of the platform to get upper and lower bounds on the speed needed to go from the platform
    // to the target position. Does this interval contain the speed we want? If so, proceed, if not, fail.
    float maxDis = -1.0e+10;
    float minDis = 1.0e+10;
    for (int c = 0; c <= 3; c++) {
        float dis = sqrtf(
            (aimPos[0] - (float)(65536 * dataPoint->targets.inPUX + keyFloors[dataPoint->targets.platKey][c][0]))
            * (aimPos[0] - (float)(65536 * dataPoint->targets.inPUX + keyFloors[dataPoint->targets.platKey][c][0]))
            + (aimPos[2] - (float)(65536 * dataPoint->targets.inPUZ + keyFloors[dataPoint->targets.platKey][c][1]))
            * (aimPos[2] - (float)(65536 * dataPoint->targets.inPUZ + keyFloors[dataPoint->targets.platKey][c][1])));
        if ( dis > maxDis) {
            maxDis = dis;
        }
        if ( dis < minDis) {
            minDis = dis;
        }
    }
    Surface* originFloor;
    float originHeight;
    int originFloorIdx = find_floor(center, &originFloor, originHeight, floors, total_floors);
    if (maxDis <= fabs(fvelocity) * (originFloor->normal[1] / 4.0f)) {
        return false;
    }
    if (minDis >= fabs(fvelocity) * (originFloor->normal[1] / 4.0f)) {
        return false;
    }
    // and now, start at the target position and simulate 4 QF's of air travel to see if it works
    // or if we run into something on our way to the pole.
    float travelPos[3];
    travelPos[0] = aimPos[0];
    travelPos[1] = originHeight;
    travelPos[2] = aimPos[2];
    for (int qf = 0; qf <= 3; qf++) {
        if (qf > 0) {
            travelPos[0] += 0.25 * vX;
            travelPos[2] += 0.25 * vZ;
        }
        Surface* floor;
        float floorHeight;
        int floorIdx = find_floor(travelPos, &floor, floorHeight, floors, total_floors);
        if ((qf == 0 && travelPos[1] <= floorHeight + 100.0f) || (qf > 0 && travelPos[1] <= floorHeight)) {
            return false;
        }
    }
    // figure out which angle is stored on the pole.
    int storedSlide = atan2s((aimPos[2] - center[2]), (aimPos[0] - center[0]));
    if (storedSlide != dataPoint->targets.storedAngle) {
        return false;
    }
    // print some data.
    printf("(%d,%d,%d,%d,%d)\n", dataPoint->targets.platKey, dataPoint->targets.inPUX, dataPoint->targets.inPUZ, dataPoint->targets.hau, dataPoint->targets.speed);
    // once you're here, you know you can get on the pole with vX and vZ stored on it.
    // Now we'll begin simulating the 10k, along with straining effects.
    // the unpleasant part is this, though. It's computationally infeasible to compute
    // all strains ahead of time, so we'll have to basically simulate things in the absence of straining
    // which should let us knock down an acceptable number of 10k stick positions that we could have
    // and then we can back-compute exactly how the straining part works and where it ends up.
    
    // our first order of business is simulating our crouchslide off the pole (might not incorporate pole push effects)
    FancySlideInfo poleslide;
    if (!sim_slide(stickTab[dataPoint->targets.i], dataPoint->positions.posPole, fminf((stickTab[dataPoint->targets.i].magnitude / 64.0f) * (stickTab[dataPoint->targets.i].magnitude / 64.0f) * 32.0f, 8.0f), vX, vZ, stickTab[dataPoint->targets.i].angle + dataPoint->targets.cam, storedSlide, dataPoint->targets.cam, true, poleslide)) {
        return false;
    }
    // make sure we're going forward
    if(poleslide.endSpeed < 0.0f) {
        return false;
    }
    // and that first QF ends up in the air.
    if (assess_floor(poleslide.endPos) != 1) {
        return false;
    }
    bool lowX = (fabs(poleslide.endPos[0]) < 10000.0f);
    bool lowZ = (fabs(poleslide.endPos[2]) < 10000.0f);
    // it gets spicy here. Pretty much, lots of ways of straining are redundant. Ie, same end speed, end sliding speeds, etc..
    // and so, we would like to iterate through the ways of straining, and if a unique option (in the sense of producing
    // different end speeds) arises, we log it in a smaller table, for this means we must only iterate through the distinct
    // results of straining, not the distinct causes of straining which have massive redundancy.
    int counter = 0;
    
    for (int j = 0; j < 20129; j++) {
        int intendedDYaw = stickTab[j].angle + dataPoint->targets.cam - poleslide.endFacingAngle;
        intendedDYaw = (intendedDYaw + 65536) % 65536;
        float intendedMag = ((stickTab[j].magnitude / 64.0f) * (stickTab[j].magnitude / 64.0f)) * 32.0f;
        float speed = poleslide.endSpeed - 0.35f;
        speed += (intendedMag / 32.0f) * gCosineTable[intendedDYaw >> 4] * 1.5f;
        float sidewaysSpeed = (intendedMag / 32.0f) * gSineTable[intendedDYaw >> 4] * 10.0f;
        speed -= 1.0f;
        float airVelX = speed * gSineTable[poleslide.endFacingAngle >> 4];
        float airVelZ = speed * gCosineTable[poleslide.endFacingAngle >> 4];
        airVelX += sidewaysSpeed * gSineTable[((poleslide.endFacingAngle + 16384) % 65536) >> 4];
        airVelZ += sidewaysSpeed * gCosineTable[((poleslide.endFacingAngle + 16384) % 65536) >> 4];
        bool duplicate = false;
        for (int k = 0; k < counter; k++) {
            if ((!lowX) && (!lowZ) && speed == tableOfStrains[k].speed && airVelX == tableOfStrains[k].vX && airVelZ == tableOfStrains[k].vZ) {
                duplicate = true;
                break;
            }
            if( lowX && speed == tableOfStrains[k].speed && fabs(airVelX - tableOfStrains[k].vX) < 0.1f && airVelZ == tableOfStrains[k].vZ) {
                duplicate = true;
                break;
            }
            if( lowZ && speed == tableOfStrains[k].speed && fabs(airVelZ - tableOfStrains[k].vZ) < 0.1f && airVelX == tableOfStrains[k].vX) {
                duplicate = true;
                break;
            }
        }
        if (duplicate) {
            continue;
        }
        tableOfStrains[counter].index = j;
        tableOfStrains[counter].speed = speed;
        tableOfStrains[counter].vX = airVelX;
        tableOfStrains[counter].vZ = airVelZ;
        counter++;
    }
    // and now counter is the total number of entries in our table of unique ways to strain! And we can iterate over that.
    // the first thing we do is compute a "prototype movement", to see if we're on the 1-up, and to preliminarily skip ahead
    // and rule out some of the 1-up stick positions. Do the usual "simulate air movement until we hit something" thing.
    bool noGround = true;
    AirInfo crudePoleAir;
    for (int qf = 1; qf <= 4; qf++) {
        if (!sim_airstep(((qf == 1) ? poleslide.endPos : crudePoleAir.endPos), ((qf == 1) ? poleslide.endSpeed : crudePoleAir.endSpeed), poleslide.endFacingAngle, (qf == 1), crudePoleAir)) {
            break;
        }
        if(assess_floor(crudePoleAir.endPos) == 0) {
            break;
        }
        else if(assess_floor(crudePoleAir.endPos) == 2 || assess_floor(crudePoleAir.endPos) == 3) {
            noGround = false;
            break;
        }
    }
    // if we didn't land on the ground, fail.
    if (noGround) {
        return false;
    }
    // if we didn't land on the one-up, fail.
    if (!on_one_up(crudePoleAir.endPos)) {
        return false;
    }

    short cameraYawTen = fix(tenk_camera_yaw(dataPoint->positions.posPole, poleslide.endPos, dataPoint->positions.posCam1, stickTab[dataPoint->targets.i].angle + dataPoint->targets.cam, poleslide.endFacingAngle, trueFocus, &sPanDistance, camPos));
    // iterate over stick positions for the 10k.
    FancySlideInfo crudeTenkslide;
    for (int j = 0; j < 20129; j++) {
        // simulate 10k.
        if (!sim_slide(stickTab[j], crudePoleAir.endPos, crudePoleAir.endSpeed, crudePoleAir.endSpeed * gSineTable[poleslide.endFacingAngle >> 4], crudePoleAir.endSpeed * gCosineTable[poleslide.endFacingAngle >> 4], poleslide.endFacingAngle, poleslide.endSlidingAngle, cameraYawTen, false, crudeTenkslide)){
            continue;
        }
        // junk everything that doesn't get 1.5B or more speed.
        if (crudeTenkslide.endSpeed < -2.147e+09 || crudeTenkslide.endSpeed > -1.5e+09){
            continue;
        }
        // and now that we've verified that the 10k stick position is basically sane, NOW it's time to start
        // iterating over straining positions. Iterate over them, make sure they work alright and hit the 1-up.
        for (int k = 0; k < counter; k++) {
            float nextPos[3];
            nextPos[0] = poleslide.endPos[0];
            nextPos[1] = poleslide.endPos[1];
            nextPos[2] = poleslide.endPos[2];
            bool landMiss = true;
            for (int qf = 1; qf <= 4; qf++) {
                nextPos[0] += tableOfStrains[k].vX / 4.0f;
                nextPos[2] += tableOfStrains[k].vZ / 4.0f;
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
            // at this point, we know that the straining hits the 1-up platform. Simulate the 10k.
            FancySlideInfo tenkslide;
            if (!sim_slide(stickTab[j], nextPos, tableOfStrains[k].speed, tableOfStrains[k].vX, tableOfStrains[k].vZ, poleslide.endFacingAngle, poleslide.endSlidingAngle, cameraYawTen, false, tenkslide)) {
                continue;
            }
            // junk stuff that isn't above 1.5B speed.
            if (tenkslide.endSpeed < -2.147e+09 || tenkslide.endSpeed > -1.5e+09){
                continue;
            }
            // the 10k must end up in the air.
            if (assess_floor(tenkslide.endPos) != 1) {
                continue;
            }
            // simulate the air movement then, and make sure it ends up on the ground.
            AirInfo tenkair;
            if (!sim_airstep(tenkslide.endPos, tenkslide.endSpeed, tenkslide.endFacingAngle, true, tenkair)) {
                continue;
            }
            if (assess_floor(tenkair.endPos) != 2 && assess_floor(tenkair.endPos) != 3) {
                continue;
            }
            // and that we're stably walking against OOB. If you want to land on a specific slice of ground
            // you can check that as well.
            bool stable = stability_check(tenkair.endPos, tenkair.endSpeed, tenkslide.endFacingAngle);
            if (!stable) {             
                continue;         
            }
            // and we've got a hit!! Celebrate, log your data.
            (dataPoint->positions).posTenK2Air[0] = poleslide.endPos[0];
            (dataPoint->positions).posTenK2Air[1] = poleslide.endPos[1];
            (dataPoint->positions).posTenK2Air[2] = poleslide.endPos[2];
            (dataPoint->positions).posTenK2[0] = nextPos[0];
            (dataPoint->positions).posTenK2[1] = nextPos[1];
            (dataPoint->positions).posTenK2[2] = nextPos[2];
            (dataPoint->positions).pos21[0] = tenkair.endPos[0];
            (dataPoint->positions).pos21[1] = tenkair.endPos[1];
            (dataPoint->positions).pos21[2] = tenkair.endPos[2];
            (dataPoint->velocities).poleVelX = vX;
            (dataPoint->velocities).poleVelZ = vZ;
            (dataPoint->velocities).velTenK2X = tableOfStrains[k].vX;
            (dataPoint->velocities).velTenK2Z = tableOfStrains[k].vZ;
            (dataPoint->velocities).velTenK2 = tableOfStrains[k].speed;
            (dataPoint->velocities).vel21 = tenkair.endSpeed;
            (dataPoint->sticks).stickPoleX = correct_stick(stickTab[dataPoint->targets.i].stickX);
            (dataPoint->sticks).stickPoleY = correct_stick(stickTab[dataPoint->targets.i].stickY);
            (dataPoint->sticks).stickStrainX = correct_stick(stickTab[tableOfStrains[k].index].stickX);
            (dataPoint->sticks).stickStrainY = correct_stick(stickTab[tableOfStrains[k].index].stickY);
            (dataPoint->sticks).stickTenK2X = correct_stick(stickTab[j].stickX);
            (dataPoint->sticks).stickTenK2Y = correct_stick(stickTab[j].stickY);
            (dataPoint->angles).slidePole = (storedSlide + 65536) % 65536;
            (dataPoint->angles).camPole = dataPoint->targets.cam;
            (dataPoint->angles).camTenK2 = cameraYawTen;
            (dataPoint->angles).facingTenK2 = poleslide.endFacingAngle;
            (dataPoint->angles).slideTenK2 = poleslide.endSlidingAngle;
            printf("HIT!\n");
            return true;
        }  
    }
    return false;
}

bool med_check(AllData* dataPoint) {
    std::unordered_set<int> camera_yaws;

    float trueFocus[3];
    float sPanDistance;
    float camPos[3];

    // we were previously testing every 100 speeds, but now we can drop down to every 10 speeds.
    for (int deltaV = -5; deltaV <= 4; deltaV++) {
        int velocity = dataPoint->targets.speed + deltaV * 10;
        // enumerate all possible facing angles on the pole, as they determine camera yaw.
        // this gives you a set of possible camera yaws.
        // warning, when jongyon was testing empirically, he was getting different ranges of camera yaws.
        int maxCamYaw = 0;
        int minCamYaw = 65536;

        for (int t = -32768; t < 32768; t++) {
            int cyaw = fix(fine_camera_yaw(dataPoint->positions.posPole, dataPoint->positions.posCam1, (short)t, trueFocus, &sPanDistance, camPos, true));

            // Skip duplicate camera yaws
            if (camera_yaws.count(cyaw) > 0) {
                continue;
            }

            // Add camera yaw to list of previously found yaws
            camera_yaws.insert(cyaw);

            if (!validCameraAngles[cyaw]) {
                continue;
            }

            // alright, we know the velocity and the angle we came from, so we know our stored sliding speed.
            float vX = (float)velocity * gSineTable[dataPoint->targets.hau];
            float vZ = (float)velocity * gCosineTable[dataPoint->targets.hau];
            // iterate over all stick positions.
            for (int i = 0; i < 20129; i++) {
                // precisely simulate the crouchslide, along with the 8 magnitude walk thing.
                FancySlideInfo poleslide;
                if (!sim_slide(stickTab[i], dataPoint->positions.posPole, fminf((stickTab[i].magnitude / 64.0f) * (stickTab[i].magnitude / 64.0f) * 32.0f, 8.0f), vX, vZ, stickTab[i].angle + cyaw, dataPoint->targets.storedAngle, cyaw, true, poleslide)) {
                    continue;
                }
                // we must be going forward to 10k later.
                if(poleslide.endSpeed < 0.0f) {
                    continue;
                }
                // our position must end in the air, to 10k later. assess being 1 is the same as "being in the air"
                if (assess_floor(poleslide.endPos) != 1) {
                    continue;
                }
                // simulate 1 to 4 air QF's until you hit ground.
                bool bad = true;
                AirInfo poleair;
                for (int j = 1; j <= 4; j++) {
                    if (!sim_airstep(((j == 1) ? poleslide.endPos : poleair.endPos), ((j == 1) ? poleslide.endSpeed : poleair.endSpeed), poleslide.endFacingAngle, (j == 1), poleair)) {
                        break;
                    }
                    // if one of the QF's is OOB, break out.
                    if(assess_floor(poleair.endPos) == 0) {
                        break;
                    }
                    // if one of the QF's is on ground, set a flag and break out.
                    else if(assess_floor(poleair.endPos) == 2 || assess_floor(poleair.endPos) == 3) {
                        bad = false;
                        break;
                    }
                }
                // the flag for successfully hitting the ground: test it.
                if (bad) {
                    continue;
                }
                // are we on the one-up platform?
                if (!on_one_up(poleair.endPos)) {
                    continue;
                }
                // ok, there's a cam yaw and way to crouchslide off the pole with the appropriate stored velocity
                // that hits a 1-up platform somewhere.
                printf("medhit(%d,%d,%d,%d,%d)\n", dataPoint->targets.hau, velocity, i, maxCamYaw, minCamYaw);
                // iterate over every speed, write some data
                for (int epsilonV = -5; epsilonV <= 4; epsilonV++) {
                    (dataPoint->targets).speed = velocity + epsilonV;
                    (dataPoint->targets).i = i;
                    (dataPoint->targets).cam = cyaw;
                    // and do the full fine check.
                    if(fine_check(dataPoint, trueFocus, sPanDistance, camPos)) {
                        return true;
                    }
                }
            }
        }
    }
    return false;
}

bool crude_check (AllData* dataPoint) {
    // this is basically checking to find if there's any angle from which we can approach the pole
    // that'd let you store a certain exact speed on it.
    // The requirements are that there's a platform in a PU that we can crouchslide from, and end up snagging the pole
    // in 1F of air movement, with the exactly correct speed.
    int velocity = dataPoint->targets.speed;
    // iterate over haus we can approach the pole from, to see if there's any angle that works.
    for (int hau = 0; hau < 4096; hau++) {
        // our first thing to do to check if an angle works is to see if there's any way to approach the pole with that
        // angle and speed. So, trace back along that hau to figure out where you were 1 frame before snagging the pole.
        // this is the "target position". We'll try to 1QF crouchslide to it, and if that's possible, then 1F of air movement should 
        // take over from there and get us to the pole with the appropriate HAU and speed.
        float targetPos[3];
        targetPos[0] = dataPoint->positions.posPole[0] - ((float)velocity * gSineTable[hau]);
        targetPos[1] = -2700.0f;
        targetPos[2] = dataPoint->positions.posPole[2] - ((float)velocity * gCosineTable[hau]);
        // The first question for the target position is this: Since we're doing 4QF's of air movement to the pole from it,
        // then the PU it's in had better be divisible by 4 in both coordinates. Otherwise, one of the air movement QF's
        // would run into OOB before the pole.
        int puX = (int)(round(targetPos[0] / 65536.0f));
        if (puX % 4 != 0) {
            continue;
        }
        int puZ = (int)(round(targetPos[2] / 65536.0f));
        if (puZ % 4 != 0) {
            continue;
        }
        // our next test is whether the target position is OOB. This is equivalent to "assess" being 0.
        // so, if "assess" isn't 0, you're in-bounds and that means that all intermediate positions in your movement
        // are in-bounds as well.
        if (assess_floor(targetPos) == 0) {
            continue;
        }
        // we crouchslide for 1QF and have 1 full frame of air movement.
        // so, if your target position was in, say, PU (28,36), then you had better start off in PU (35, 45).
        int startPuX = (puX * 5) / 4;
        int startPuZ = (puZ * 5) / 4;
        // in the PU we're starting from, we iterate over platforms to start from, to see if we can crouchslide to our
        // target position. Each integer is keyed to a particular platform that might be good for
        // our PU movement.
        for (int k = 0; k <= 6; k++) {
            // I later realized that platform 2 is nearly impossible to get to in the first place
            // so I just did a quick hack to skip it.
            if (k == 2) {
                continue;
            }
            // assume we depart from the center of the platform in the appropriate PU, that gets a good approximation
            // of stuff.
            float center[3];
            center[0] = (float)(65536 * startPuX + keyCenter[k][0]);
            center[1] = -2700.0f;
            center[2] = (float)(65536 * startPuZ + keyCenter[k][1]);
            // Test whether crouchsliding from the center of that platform to our target position
            // ends up with us travelling along the HAU we need. If yes, proceed, if not, skip to the next platform.
            if(!(angle_match(center, targetPos, hau))) {
                continue;
            }
            // next up is seeing whether it's possible to crouchslide from our target platform and end up with the exact
            // speed we need. If we know what the target position is, then testing the four corners of the platform
            // gets you a range of speeds you can arrive at the target position with. That range of speeds should include
            // the exact speed we need.
            float maxDis = -1.0e+10;
            float minDis = 1.0e+10;
            for (int c = 0; c <= 3; c++) {
                float dis = sqrtf(
                (targetPos[0] - (float)(65536 * startPuX + keyFloors[k][c][0])) 
                * (targetPos[0] - (float)(65536 * startPuX + keyFloors[k][c][0])) 
                + (targetPos[2] - (float)(65536 * startPuZ + keyFloors[k][c][1])) 
                * (targetPos[2] - (float)(65536 * startPuZ + keyFloors[k][c][1])));
                if ( dis > maxDis) {
                    maxDis = dis;
                }
                if ( dis < minDis) {
                    minDis = dis;
                }
            }
            Surface* floor;
            float floorHeight;
            int floorIdx = find_floor(center, &floor, floorHeight, floors, total_floors);
            if (maxDis <= fabs((float)velocity) * (floor->normal[1] / 4.0f)) {
                continue;
            }
            if (minDis >= fabs((float)velocity) * (floor->normal[1] / 4.0f)) {
                continue;
            }
            // now at this point, you know that it's possible to start from a certain platform in a certain PU
            // and crouchslide to a given target position with the exact speed needed
            // and that then your HAU's will round off the right way
            // to end up hitting the pole after 1 full frame of air movement with the exact speed we need
            // at the exact angle we're testing.
            // so log data
            (dataPoint->targets).platKey = k;
            (dataPoint->targets).inPUX = startPuX;
            (dataPoint->targets).inPUZ = startPuZ;
            (dataPoint->targets).hau = hau;
            (dataPoint->targets).storedAngle = atan2s((targetPos[2] - center[2]), (targetPos[0] - center[0]));
            // and proceed to finer testing.
            if(med_check(dataPoint)) {
                return true;
            }
        }
    }
    return false;
}


int main(int argc, char* argv[]) {
    
    float cameraPosition[3];
    cameraPosition[0] = -1700.0f;
    cameraPosition[1] = -2300.0f;
    cameraPosition[2] = 500.0f;
    float polePosition[3];
    polePosition[0] = 6605.0f;
    polePosition[1] = -2970.0f;
    polePosition[2] = 266.0f;
    int highSpeed = 4232700;
    int lowSpeed = 4232600;
    
    unsigned int numMaxSolutions = 100;
       
    std::string outFile = "outData.csv";
    
    bool verbose = false;
    
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "-h") || !strcmp(argv[i], "--help")) {
            printf("BitFS Pole Setup Brute Forcer.\n");
            printf("This program accepts the following options:\n\n");
            printf("-b <max_speed> <min_speed>: The maximum and minimum speeds that are searched. Note that (430,380) corresponds to -430 speed to -380 speed, technically.\n");
            printf("             Default: %d %d\n", highSpeed, lowSpeed);
            printf("-cp <pos_x> <pos_y> <pos_z>: Position of the camera.\n");
            printf("             Default: %f %f %f\n", cameraPosition[0], cameraPosition[1], cameraPosition[2]);
            printf("-o: Path to the output file.\n");
            printf("    Default: %s\n", outFile.c_str());
            printf("-v: Verbose mode. Prints all parameters used in brute force.\n");
            printf("    Default: off\n");
            printf("-h --help: Prints this text.\n");
            exit(0);
        }
        else if (!strcmp(argv[i], "-b")) {
            highSpeed = std::stoi(argv[i + 1]);
            lowSpeed = std::stoi(argv[i + 2]);

            i += 1;
        }
        else if (!strcmp(argv[i], "-cp")) {
            cameraPosition[0] = std::stoi(argv[i + 1]);
            cameraPosition[1] = std::stoi(argv[i + 2]);
            cameraPosition[2] = std::stoi(argv[i + 3]);

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
            printf("Speed Bounds: (%d, %d)\n", -highSpeed, -lowSpeed);
            printf("Camera Position: (%f, %f, %f)\n", cameraPosition[0], cameraPosition[1], cameraPosition[2]);
        }
    }
    
    initialise_floors();
    initialise_keyFloors();
    init_stick_tables();
    init_camera_angles();

    std::ofstream wf(outFile);
    wf << std::fixed;
    wf << "Camera Position X, Camera Position Y, Camera Position Z, ";
    wf << "First Frame Position X, First Frame Position Y, First Frame Position Z, ";
    wf << "10k Position X, 10k Position Y, 10k Position Z, ";
    wf << "Landing Position X, Landing Position Y, Landing Position Z, ";
    wf << "Pole Velocity X, Pole Velocity Z, Into 10k Velocity X, Into 10k Velocity Z, ";
    wf << "Into 10k Velocity, Post 10k Velocity, ";
    wf << "Pole Stick X, Pole Stick Y, Strain Stick X, Strain Stick Y, 10k Stick X, 10k Stick Y, ";
    wf << "Stored Slide Yaw, Pole Camera Yaw, 10k Camera Yaw, 10k Facing Angle, 10k Slide Yaw, ";
    wf << "Platform ID, PU X, PU Z, Ingoing Speed, HAU Approach, " << std::endl;

    AllData* data = (AllData*) std::malloc(sizeof(AllData) * numMaxSolutions);

    if (!data) {
        printf("Failed to allocate memory for solution data!");
        return 0;
    }

    if (numMaxSolutions > 0) {
        data[0].positions.posCam1[0] = cameraPosition[0];
        data[0].positions.posCam1[1] = cameraPosition[1];
        data[0].positions.posCam1[2] = cameraPosition[2];
        data[0].positions.posPole[0] = polePosition[0];
        data[0].positions.posPole[1] = polePosition[1];
        data[0].positions.posPole[2] = polePosition[2];
    }

    int solutionCount = 0;
    bool proceed = false;
    for (int s = -(highSpeed / 100); s <= -(lowSpeed / 100) && solutionCount < numMaxSolutions; s++) {
        if(s % 10 == 0) {
            printf("%d\n", s * 100);
        }
        data[solutionCount].targets.speed = s * 100;
        if(crude_check(&data[solutionCount])) {
            solutionCount++;
            if (solutionCount < numMaxSolutions) {
                data[solutionCount].positions.posCam1[0] = cameraPosition[0];
                data[solutionCount].positions.posCam1[1] = cameraPosition[1];
                data[solutionCount].positions.posCam1[2] = cameraPosition[2];
                data[solutionCount].positions.posPole[0] = polePosition[0];
                data[solutionCount].positions.posPole[1] = polePosition[1];
                data[solutionCount].positions.posPole[2] = polePosition[2];
            }
        }
    }
    if(solutionCount == 0) {
        printf("FAILURE");
        return 0;
    }
    else {
        printf("SUCCESS");
        for (int i = 0; i < solutionCount; ++i) {
            wf << data[i].positions.posCam1[0] << ", " << data[i].positions.posCam1[1] << ", " << data[i].positions.posCam1[2] << ", ";
            wf << data[i].positions.posTenK2Air[0] << ", " << data[i].positions.posTenK2Air[1] << ", " << data[i].positions.posTenK2Air[2] << ", ";
            wf << data[i].positions.posTenK2[0] << ", " << data[i].positions.posTenK2[1] << ", " << data[i].positions.posTenK2[2] << ", ";
            wf << data[i].positions.pos21[0] << ", " << data[i].positions.pos21[1] << ", " << data[i].positions.pos21[2] << ", ";
            wf << data[i].velocities.poleVelX << ", " << data[i].velocities.poleVelZ << ", " << data[i].velocities.velTenK2X << ", " << data[i].velocities.velTenK2Z << ", ";
            wf << data[i].velocities.velTenK2 << ", " << data[i].velocities.vel21 << ", ";
            wf << data[i].sticks.stickPoleX << ", " << data[i].sticks.stickPoleY << ", " << data[i].sticks.stickStrainX << ", " << data[i].sticks.stickStrainY << ", " << data[i].sticks.stickTenK2X << ", " << data[i].sticks.stickTenK2Y << ", ";
            wf << data[i].angles.slidePole << ", " << data[i].angles.camPole << ", " << data[i].angles.camTenK2 << ", " << data[i].angles.facingTenK2 << ", " << data[i].angles.slideTenK2 << ", ";
            wf << data[i].targets.platKey << ", " << data[i].targets.inPUX << ", " << data[i].targets.inPUZ << ", " << data[i].targets.speed << ", " << data[i].targets.hau << std::endl;
        }
        wf.close();
        free(data);
        return 0;
    }
}

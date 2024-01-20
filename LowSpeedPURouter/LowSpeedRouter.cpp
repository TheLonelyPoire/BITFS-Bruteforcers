#include <fstream>
#include <cstring>
#include <string>
#include <unordered_map>
#include <iostream>
#include <cmath>
  
#include "../Common/CommonBruteforcerStructs.hpp"
#include "../Common/Camera.cuh"
#include "../Common/Donut.cuh"
#include "../Common/Floors.cuh"
#include "../Common/Movement.cuh"
#include "../Common/Stick.cuh"
#include "../Common/Surface.cuh"
#include "../Common/Trig.cuh"
#include "../Common/VMath.cuh"

#include "LowSpeedRouteStructs.hpp"

# define M_PI            3.14159265358979323846  /* pi */
   
using namespace BITFS;

bool attainableArctans[8192];

// Output Printing
int totalHits11, totalHits12, totalHits13, totalHits14 = 0;

void init_attainable_arctans() {
    for (int t = 0; t < 8192; t++) {
        attainableArctans[t] = false;
    }
    for (int i = 0; i < 20129; i++) {
        attainableArctans[gReverseArctanTable[fix(stickTab[i].angle)]] = true;
    }
}


bool move14(AllData* dataPoint, short fangle) {
    
    float trueFocus[3];
    float sPanDistance;
    float camPos[3];

    int camYaw = fix(fine_camera_yaw(dataPoint->positions.pos14, dataPoint->positions.posCam1, fangle, trueFocus, &sPanDistance, camPos, false));
    // we're going to do kind of a weird thing here. Namely, instead of iterating over HAU's and stick positions, we iterate
    // over HAU's and waiting frames!! But first, we work out exactly where we be aiming.
    float aimPoint[3];
    aimPoint[0] = dataPoint->positions.posPole[0] - (float)(dataPoint->targets.speed) * gSineTable[dataPoint->targets.hau];
    aimPoint[2] = dataPoint->positions.posPole[2] - (float)(dataPoint->targets.speed) * gCosineTable[dataPoint->targets.hau];
    // and how far to where we're aiming
    float dis = find_dis(dataPoint->positions.pos14, aimPoint);
    // and make damn sure that "we attain our exact needed speed" gives you enough distance to make it close to the aiming point
    Surface* floor;
    float floorheight;
    int floorIdx = find_floor(dataPoint->positions.pos14, &floor, floorheight, floors, total_floors);
    if (fabs(dis - (fabs((float)(dataPoint->targets.speed)) * floor->normal[1] / 4.0f)) > 150.0f) {
        return false;
    }
    // work out the direction from the pole to where you are now, as that approximates your facing angle
    int direction = atan2s(dataPoint->positions.pos14[2] - dataPoint->positions.posPole[2], dataPoint->positions.pos14[0] - dataPoint->positions.posPole[0]);
    // do our black magic computation for which AU's are attainable.     
    int dist13 = fmin(fix((direction - (16 * 16)) - (direction + (16 * 16))), fix((direction + (16 * 16)) - (direction - (16 * 16))));
    for (int t = 0; t < 8192; t++) {
        
        int fangle = fix(gArctanTable[t] + camYaw);
        
        int dist23 = fmin(fix(fangle - (direction - (16 * 16))), fix((direction - (16 * 16)) - fangle));
        int dist12 = fmin(fix(fangle - (direction + (16 * 16))), fix((direction + (16 * 16)) - fangle));
        if (dist12 + dist23 > dist13) {
            continue;
        }
        // ok, the AU is attainable, is it stable?
        if (!stability_check(dataPoint->positions.pos14, dataPoint->velocities.vel14Arrive, fangle)) {
            continue;
        }
        // log the speed you arrived with.
        float speed = dataPoint->velocities.vel14Arrive;
        // iterate over how long to wait. Long wait times can be sped up with ya wa hoo
        for (int delay = 0; delay <= 600; delay++) {
            if (delay > 0) {
                speed += 1.1f;
            }
            // try to solve for the stick position that hits the aim point
            StickTableData sticksol;
            bool success = infer_stick(dataPoint->positions.pos14, aimPoint, speed, fangle, camYaw, sticksol);
            
            // check to see that it's feasible
            if(!success || sticksol.magnitude >= 64.0f) {
                continue;
            }
            // simulate the slide
            FancySlideInfo lastslide;
            if (!sim_slide(sticksol, dataPoint->positions.pos14, speed, speed * gSineTable[fangle >> 4], speed * gCosineTable[fangle >> 4], fangle, fangle, camYaw, false, lastslide)) {
                continue;
            }
            // must travel along the needed HAU going in
            if (lastslide.endFacingAngle >> 4 != dataPoint->targets.hau) {
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
            if (fabs(airmove.endSpeed - (float)(dataPoint->targets.speed)) >= 1.5f) {
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
            float miss = find_dis(nextPos, dataPoint->positions.posPole);
            if (miss > 117.0f) {
                continue;
            }
            // log data
            printf("ROUTE FOUND\n");
            totalHits14++;
            (dataPoint->velocities).vel14 = speed;
            (dataPoint->velocities).poleVelX = airmove.endSpeed * gSineTable[lastslide.endFacingAngle >> 4];
            (dataPoint->velocities).poleVelZ = airmove.endSpeed * gCosineTable[lastslide.endFacingAngle >> 4];
            (dataPoint->sticks).stick14X = correct_stick(sticksol.stickX);
            (dataPoint->sticks).stick14Y = correct_stick(sticksol.stickY);
            (dataPoint->angles).facing14 = fangle;
            (dataPoint->angles).cam14 = camYaw;
            (dataPoint->angles).slidePole = lastslide.endSlidingAngle;
            (dataPoint->waits).delayFrames = delay + 20;
            return true;
        }  
    }
    return false;
}

bool move13(AllData* dataPoint, short fangle) {
    
    float trueFocus[3];
    float sPanDistance;
    float camPos[3];

    int camYaw = fix(fine_camera_yaw(dataPoint->positions.pos13, dataPoint->positions.posCam1, fangle, trueFocus, &sPanDistance, camPos, false));
    // the destination is the center of the platform you're headed for, in the PU you're headed for.
    float destination[3];
    destination[0] = keyCenter[dataPoint->targets.platKey][0] + dataPoint->targets.inPUX * 65536.0f;
    destination[2] = keyCenter[dataPoint->targets.platKey][1] + dataPoint->targets.inPUZ * 65536.0f;
    // direction is a good approximation of what your facing angle should be.
    int direction = atan2s(dataPoint->positions.pos13[2] - destination[2], dataPoint->positions.pos13[0] - destination[0]);
    // circle black magic for which AU to check.
    int dist13 = fmin(fix((direction - (32 * 16)) - (direction + (32 * 16))), fix((direction + (32 * 16)) - (direction - (32 * 16))));
    for (int t = 0; t < 8192; t++) {
        
        if (!attainableArctans[t]) {
            continue;
        }
        int fangle = fix(gArctanTable[t] + camYaw);
        
        int dist23 = fmin(fix(fangle - (direction - (32 * 16))), fix((direction - (32 * 16)) - fangle));
        int dist12 = fmin(fix(fangle - (direction + (32 * 16))), fix((direction + (32 * 16)) - fangle));
        if (dist12 + dist23 > dist13) {
            continue;
        }
        // check if it's stable
        if (!stability_check(dataPoint->positions.pos13, dataPoint->velocities.vel13, fangle)) {
            continue;
        }
        // iterate over stick positions
        for (int i = 0; i < 20129; i++) {
            // simulate the slide
            FancySlideInfo fineslide;
            if (!sim_slide(stickTab[i], dataPoint->positions.pos13, dataPoint->velocities.vel13, dataPoint->velocities.vel13 * gSineTable[fangle >> 4], dataPoint->velocities.vel13 * gCosineTable[fangle >> 4], fangle, fangle, camYaw, false, fineslide)) {
                continue;
            }
            // did it land in air?
            if (assess_floor(fineslide.endPos) != 1) {
                continue;
            }
            // simulate air movement
            AirInfo airmove;

            // did it land on ground?
            if(!sim_airstep(fineslide.endPos, fineslide.endSpeed, fineslide.endFacingAngle, true, airmove) || (assess_floor(airmove.endPos) != 2 && assess_floor(airmove.endPos) != 3)) {
                continue;
            }
            // did you hit the right PU?
            if((int)round(airmove.endPos[0] / 65536.0f) != dataPoint->targets.inPUX) {
                continue;
            }
            if((int)round(airmove.endPos[2] / 65536.0f) != dataPoint->targets.inPUZ) {
                continue;
            }
            // check to see if the floor you landed on is associated with the ID of the platform you're aiming for.
            Surface* floor;
            float floorheight;
            int floorIdx = find_floor(airmove.endPos, &floor, floorheight, floors, total_floors);
            if(dataPoint->targets.platKey == 0 && floorIdx != 20 && floorIdx != 21) {
                continue;
            }
            if(dataPoint->targets.platKey == 1 && floorIdx != 25 && floorIdx != 26) {
                continue;
            }
            if(dataPoint->targets.platKey == 2 && floorIdx != 23 && floorIdx != 24) {
                continue;
            }
            if(dataPoint->targets.platKey == 3 && floorIdx != 28 && floorIdx != 29) {
                continue;
            }
            if(dataPoint->targets.platKey == 4 && floorIdx != 18 && floorIdx != 19) {
                continue;
            }
            if(dataPoint->targets.platKey == 5 && floorIdx != 14 && floorIdx != 15) {
                continue;
            }
            if(dataPoint->targets.platKey == 6 && (floorIdx < 8 || floorIdx > 13)) {
                continue;
            }

            float nextspeed = airmove.endSpeed;
            // iterate over waiting frames.
            for (int wf = 0; wf <= 3; wf++) {
                if (wf > 0) {
                    nextspeed *= 0.98f;
                }
                // are we stable against OOB?
                bool stable = stability_check(airmove.endPos, nextspeed, fineslide.endFacingAngle);
                // also, if the speed we're trying to hit is above 0.94x our current speed, then that's bad
                // and waiting longer won't help.
                if (!stable || fabs((float)(dataPoint->targets.speed)) > 0.94f * fabs(nextspeed)) {
                    break;
                }
                // if the speed we're trying to hit is below 0.9x our current speed, that's bad, add more waiting frames.
                if(fabs((float)(dataPoint->targets.speed)) < 0.9f * fabs(nextspeed)) {
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
                totalHits13++;
                if(move14(dataPoint, fineslide.endFacingAngle)) {
                    return true;
                }
            }
        }
    }
    return false;
}

bool move12(AllData* dataPoint, short fangle) {
    float lb = 0.9f;
    float ub = 0.94f;

    float trueFocus[3];
    float sPanDistance;
    float camPos[3];

    int camYaw = fix(fine_camera_yaw(dataPoint->positions.pos12, dataPoint->positions.posCam1, fangle, trueFocus, &sPanDistance, camPos, false));
    // the below for loop is because basically, the PU's that have a shot at the main universe are in the overlap of two donuts
    // the donut of PU's we can reach, and the donut of PU's that can 1fcs to the main universe within our speed band.
    // so we've got two "islands" of PU's we're iterating over.
    for (int a = 0; a <= 1; a++) {
        // Specifying the island we're aiming for means we can use the angle data from our donut computation to figure out
        // the range of angles we're interested in. Warning. These HAU's might be negative. This is intended.
        int dist13 = fmin(fix(dataPoint->donuts.hauBand[a][0] * 16 - dataPoint->donuts.hauBand[a][1] * 16), fix(dataPoint->donuts.hauBand[a][1] * 16 - dataPoint->donuts.hauBand[a][0] * 16));
        // the above is some black magic for testing whether our angle will be in the hau band.
        for (int t = 0; t < 8192; t++) {
        
            if (!attainableArctans[t]) {
                continue;
            }
            int fangle = fix(gArctanTable[t] + camYaw);
            // black magic time.
            int dist23 = fmin(fix(fangle - dataPoint->donuts.hauBand[a][1] * 16), fix(dataPoint->donuts.hauBand[a][1] * 16 - fangle));
            int dist12 = fmin(fix(fangle - dataPoint->donuts.hauBand[a][0] * 16), fix(dataPoint->donuts.hauBand[a][0] * 16 - fangle));
            if (dist12 + dist23 > dist13) {
                continue;
            }
            // is the facing angle stable?
            if (!stability_check(dataPoint->positions.pos12, dataPoint->velocities.vel12, fangle)) {
                continue;
            }
            // iterate over stick positions
            for (int i = 0; i < 20129; i++) {
                // simulate the slide
                FancySlideInfo fineslide;
                if (!sim_slide(stickTab[i], dataPoint->positions.pos12, dataPoint->velocities.vel12, dataPoint->velocities.vel12 * gSineTable[fangle >> 4], dataPoint->velocities.vel12 * gCosineTable[fangle >> 4], fangle, fangle, camYaw, false, fineslide)) {
                    continue;
                }
                // is it in the air?
                if (assess_floor(fineslide.endPos) != 1) {
                    continue;
                }
                // simulate air movement
                AirInfo airmove;
                
                if (!sim_airstep(fineslide.endPos, fineslide.endSpeed, fineslide.endFacingAngle, true, airmove)) {
                    continue;
                }
                // is it on the ground?
                if(assess_floor(airmove.endPos) != 2 && assess_floor(airmove.endPos) != 3) {
                    continue;
                }
                // did you stably land?
                if (!stability_check(airmove.endPos, airmove.endSpeed, fineslide.endFacingAngle)) {
                    continue;
                }
     
                // Time to check if we're on the right ground.
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
            
                if (floorIdx < 8 || floorIdx > 32 || floorIdx == 22) {
                    continue;
                }
                //if (floorheight > -2866.0f) {
                //    continue;
                //}
                //if (floorIdx != 28 && floorIdx != 29) {
                //    continue;
                //}
                
                
                float nextspeed = airmove.endSpeed;
                // simulate waiting frames
                for (int wf = 0; wf <= 3; wf++) {
                    if (wf > 0) {
                        nextspeed *= 0.98f;
                    }
                    // make sure you're stable against OOB
                    if ((!stability_check(airmove.endPos, airmove.endSpeed, fineslide.endFacingAngle)) && wf > 0) {
                        break;
                    }
                    // basically, if the speeds are incompatible with getting to the target with our needed speed, give up
                    if (fmaxf(lb * fabs(nextspeed), fabs((float)(dataPoint->targets.speed)) / (0.94f)) > fminf(ub * fabs(nextspeed), fabs((float)(dataPoint->targets.speed)) / (0.9f * 0.98f * 0.98f * 0.98f))) {
                        break;
                    }
                    // work out where you're aiming for specifically
                    float destination[3];
                    destination[0] = keyCenter[dataPoint->targets.platKey][0] + dataPoint->targets.inPUX * 65536.0f;
                    destination[2] = keyCenter[dataPoint->targets.platKey][1] + dataPoint->targets.inPUZ * 65536.0f;
                    
                    float dis = find_dis(airmove.endPos, destination);
                    // a better check for whether our speed is compatible with both making it to our target
                    // AND making it to the target with the speed we need.
                    if (dis < ((1.0f + floor->normal[1]) / 4.0f) * fmaxf(lb * fabs(nextspeed), fabs((float)(dataPoint->targets.speed)) / (0.94f)) || dis > ((1.0f + floor->normal[1]) / 4.0f) * fminf(ub * fabs(nextspeed), fabs((float)(dataPoint->targets.speed)) / (0.9f * 0.98f * 0.98f * 0.98f))) {
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
                    printf("s");
                    totalHits12++;
                    if(move13(dataPoint, fineslide.endFacingAngle)) {
                        return true;
                    }
                }
            }
        }
    }
    return false;
}

bool move11(AllData* dataPoint) {
    float lb = 0.9f * 0.9f * 0.98f * 0.98f * 0.98f;
    float ub = 0.94f * 0.94f;
    int counter = 0;
    // keeping track of how many hits we have for our first motion is useful for partially running a computation
    // logging where you were, and skipping ahead.
    int crudeCamYaw = fix(crude_camera_yaw(dataPoint->positions.pos11, dataPoint->positions.posCam1));
    
    float trueFocus[3];
    float sPanDistance;
    float camPos[3];
    int camYaw;


    // iterate over AU's
    for (int t = 0; t < 8192; t++) {

        // can we do that AU?
        if (!attainableArctans[t]) {
            continue;
        }
        int fangle = fix(gArctanTable[t] + crudeCamYaw);
        camYaw = fine_camera_yaw(dataPoint->positions.pos11, dataPoint->positions.posCam1, fangle, trueFocus, &sPanDistance, camPos, false);

        // is it stable?
        if (!stability_check(dataPoint->positions.pos11, dataPoint->velocities.vel11, fangle)) {
            continue;
        }
        // iterate over sticks
        for (int i = 0; i < 20129; i++) {
            // simulate crouchslide.
            FancySlideInfo fineslide;
            if (!sim_slide(stickTab[i], dataPoint->positions.pos11, dataPoint->velocities.vel11, dataPoint->velocities.vel11 * gSineTable[fangle >> 4], dataPoint->velocities.vel11 * gCosineTable[fangle >> 4], fangle, fangle, camYaw, false, fineslide)) {
                continue;
            }
            if (assess_floor(fineslide.endPos) != 1) {
                continue;
            }
            //simulate air steps.
            AirInfo airmove;
            bool goodAir = false;
            for (int qf = 1; qf <= 4; qf++) {
                if (!sim_airstep(((qf == 1) ? fineslide.endPos : airmove.endPos), ((qf == 1) ? fineslide.endSpeed : airmove.endSpeed), fineslide.endFacingAngle, (qf == 1), airmove) || assess_floor(airmove.endPos) == 0) {
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
            if (!stability_check(airmove.endPos, airmove.endSpeed, fineslide.endFacingAngle)) {
                continue;
            }
            // we must check that we've landed on an PU of the same parity as our target, it's a severe constraint.
            if (((int)(round(airmove.endPos[0] / 65536.0f)) % 2) != (dataPoint->targets.inPUX % 2) || ((int)(round(airmove.endPos[2] / 65536.0f)) % 2) != (dataPoint->targets.inPUZ % 2)) {
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
            
            if (floorIdx < 8 || floorIdx > 32 || floorIdx == 22) {
                continue;
            }
            //if (floorheight > -2866.0f) {
            //    continue;
            //}
            //if (floorIdx != 28 && floorIdx != 29) {
            //    continue;
            //}
            
            
            float nextspeed = airmove.endSpeed;
            // iterate over waiting frames.
            for (int wf = 0; wf <= 3; wf++) {
                if (wf > 0) {
                    nextspeed *= 0.98f;
                }
                // is it stable?
                if ((!stability_check(airmove.endPos, nextspeed, fineslide.endFacingAngle)) && wf > 0) {
                    break;
                }
                // check to see if our speed is acceptable, though admittedly this is pretty fucking arcane.
                if (fmaxf(lb * fabs(nextspeed), fabs((float)(dataPoint->targets.speed)) / 0.94f) >= fminf(ub * fabs(nextspeed), fabs((float)(dataPoint->targets.speed)) / (0.98f * 0.98f * 0.98f * 0.9f))) {
                    break;
                }
                // where we're aiming for.
                float destination[3];
                destination[0] = keyCenter[dataPoint->targets.platKey][0] + dataPoint->targets.inPUX * 65536.0f;
                destination[2] = keyCenter[dataPoint->targets.platKey][1] + dataPoint->targets.inPUZ * 65536.0f;
                
                // computing the donut of "where we can reach" and "where can reach our target with the needed speed"
                // so we can aim for the intersection.
                DonutData glazed = donut_compute(airmove.endPos, 0.9f * ((1.0f + floor->normal[1]) / 4.0f) * fabs(nextspeed), 0.94f * ((1.0f + floor->normal[1]) / 4.0f) * fabs(nextspeed), destination, ((1.0f + 0.98f) / 4.0f) * fmaxf(lb * fabs(nextspeed), fabs((float)(dataPoint->targets.speed)) / 0.94f), ((1.0f + 1.0f) / 4.0f) * fminf(ub * fabs(nextspeed), fabs((float)(dataPoint->targets.speed)) / (0.98f * 0.98f * 0.98f * 0.9f)));
                if(glazed.overlapArea < 10000.0f) {
                    continue;
                }
                // log data, continue.
                printf("f(%d,%d)\n", counter, fangle);
                totalHits11++;
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
                if(counter > 0) {
                    if(move12(dataPoint, fineslide.endFacingAngle)) {
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
    cameraPosition[2] = 500.0f;
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
    int targetPlat = 6;
    int targetPUX = 75;
    int targetPUZ = -30;
    // for best results, pick 2 speed closer to 0 than the first output you got from the pole thing
    // gives you more robust solutions that way.
    int targetSpeed = -4232649;
    int targetHAU = 1272;
       
    std::string outFile = "verifiedVersion.csv";
    
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
    wf << "Pole Velocity X, Pole Velocity Z, Pole Slide Yaw" << std::endl;
    
    if(move11(&data)) {
        wf << data.positions.posCam1[0] <<", " << data.positions.posCam1[1] <<", " << data.positions.posCam1[2] <<", ";
        wf << data.positions.pos11[0] <<", " << data.positions.pos11[1] <<", " << data.positions.pos11[2] <<", ";
        wf << data.angles.facing11 <<", " << data.angles.cam11 <<", " << data.velocities.vel11 <<", " << data.sticks.stick11X <<", " << data.sticks.stick11Y <<", " << data.waits.waiting11 <<", ";
        wf << data.positions.pos12[0] <<", " << data.positions.pos12[1] <<", " << data.positions.pos12[2] <<", ";
        wf << data.angles.facing12 <<", " << data.angles.cam12 <<", " << data.velocities.vel12 <<", " << data.sticks.stick12X <<", " << data.sticks.stick12Y <<", " << data.waits.waiting12 <<", ";
        wf << data.positions.pos13[0] <<", " << data.positions.pos13[1] <<", " << data.positions.pos13[2] <<", ";
        wf << data.angles.facing13 <<", " << data.angles.cam13 <<", " << data.velocities.vel13 <<", " << data.sticks.stick13X <<", " << data.sticks.stick13Y <<", " << data.waits.waiting13 <<", ";
        wf << data.positions.pos14[0] <<", " << data.positions.pos14[1] <<", " << data.positions.pos14[2] <<", ";
        wf << data.angles.facing14 <<", " << data.angles.cam14 <<", " << data.velocities.vel14 <<", " << data.sticks.stick14X <<", " << data.sticks.stick14Y <<", " << data.waits.delayFrames <<", ";
        wf << data.velocities.poleVelX <<", " << data.velocities.poleVelZ <<", " << data.angles.slidePole << std::endl;
        wf.close();
        return 0;
    }
    else {
        printf("FAILURE");
        printf("Total Stage 1 Hits: %d\n", totalHits11);
        printf("Total Stage 2 Hits: %d\n", totalHits12);
        printf("Total Stage 3 Hits: %d\n", totalHits13);
        printf("Total Stage 4 Hits: %d\n", totalHits14);
        return 0;
    }
}
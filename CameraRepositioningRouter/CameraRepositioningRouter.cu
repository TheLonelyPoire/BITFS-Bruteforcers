
#include "cuda_runtime.h"
#include "device_launch_parameters.h"

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

#include "CameraRepositioningStructs.hpp"

# define M_PI            3.14159265358979323846  /* pi */

using namespace BITFS;

bool attainableArctans[8192];

// Output Printing
int totalHits21, totalHits22, totalHits23 = 0;

// Solution counter
int counter = 0;

// Highest Speed Solution
AllData best_solution;
int best_solution_counter = 0;
bool foundSolution = false;

void init_attainable_arctans() {
    for (int t = 0; t < 8192; t++) {
        attainableArctans[t] = false;
    }
    for (int i = 0; i < 20129; i++) {
        attainableArctans[gReverseArctanTable[fix(stickTab[i].angle)]] = true;
    }
}

// tests whether an XZ point 5 is in the convex hull of the four points. Points must be in clockwise
// or counterclockwise order, you can't throw them in arbitrarily.
bool membership_test(float* point1, float* point2, float* point3, float* point4, float* point5) {

    // first off, turn point5 into x z format.
    float point5XZ[2];
    point5XZ[0] = point5[0];
    point5XZ[1] = point5[2];
    // these are normal vectors for the line going from point 1 to point 2, etc...
    // importantly, they are not normalized, and may be pointed the wrong way
    // and we don't know the requisite constants associated with them.
    // however, their dot product with points 1 and 2 (etc...) will be the same thing
    // and we can use their dot product with the other points (3 and 4, or analogues)
    // to determine the proper orientation of these normals
    // and then just check that the dot product of point5 w.r.t. all these vectors is what it should be
    // to certify that it's in the convex hull
    // because every convex shape is an intersection of halfspaces.
    float normal12[2];
    float normal23[2];
    float normal34[2];
    float normal41[2];

    normal12[0] = point2[1] - point1[1];
    normal12[1] = point1[0] - point2[0];
    // note that the dot product of this with point1 is point1[0](point2[1] - point1[1]) + point1[1](point1[0] - point2[0])
    // which is point1[0] * point2[1] - point1[1] * point2[0]
    // and the dot product of this with point2 is point2[0](point2[1] - point1[1]) + point2[1](point1[0] - point2[0])
    // which also reduces to point1[0] * point2[1] - point1[1] * point2[0]
    // since the dot products are the same, it's a normal vector. We can get normal vectors for the rest as well.
    normal23[0] = point3[1] - point2[1];
    normal23[1] = point2[0] - point3[0];
    normal34[0] = point4[1] - point3[1];
    normal34[1] = point3[0] - point4[0];
    normal41[0] = point1[1] - point4[1];
    normal41[1] = point4[0] - point1[0];
    // and then we check whether <normal, point its defined with> - <normal, nonspecified point> is more or less than 0
    // if it's more than 0, then since the nonspecified points are on the correct side of the hyperplane, then
    // the correct side of the hyperplane has low inner products, so if point5 is in the convex hull
    // then <normal, point its defined with> - <normal, point5> will also be > 0.
    // similar arguments apply to the others. So this lets us infer the proper orientation of the normal vectors
    // and makes sure that we have the correct sides of all the halfspaces.
    if ((dot_prod_2(normal12, point1) - dot_prod_2(normal12, point3))
        * (dot_prod_2(normal12, point1) - dot_prod_2(normal12, point5XZ)) < 0) {
        return false;
    }
    else if ((dot_prod_2(normal23, point2) - dot_prod_2(normal23, point4))
        * (dot_prod_2(normal23, point2) - dot_prod_2(normal23, point5XZ)) < 0) {
        return false;
    }
    else if ((dot_prod_2(normal34, point3) - dot_prod_2(normal34, point1))
        * (dot_prod_2(normal34, point3) - dot_prod_2(normal34, point5XZ)) < 0) {
        return false;
    }
    else if ((dot_prod_2(normal41, point4) - dot_prod_2(normal41, point2))
        * (dot_prod_2(normal41, point4) - dot_prod_2(normal41, point5XZ)) < 0) {
        return false;
    }
    else {
        return true;
    }
}


void move23(AllData* dataPoint) {

    int camYaw = fix(crude_camera_yaw(dataPoint->positions.pos23, dataPoint->positions.posCam1));

    // the destination is the main universe.
    // yes, yes, not actually accurate, but given how stupidly inaccurate we are at these high speeds
    // doing a shotgun blast at the main universe will definitely find a working route if one exists to be found.

    float destination[3];
    destination[0] = 0.0f;
    destination[2] = 0.0f;
    // direction is a good approximation of what your facing angle should be.
    int direction = atan2s(dataPoint->positions.pos23[2] - destination[2], dataPoint->positions.pos23[0] - destination[0]);
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
        if (!stability_check(dataPoint->positions.pos23, dataPoint->velocities.vel23, fangle)) {
            continue;
        }
        // iterate over stick positions
        for (int i = 0; i < 20129; i++) {
            // simulate the slide
            FancySlideInfo fineslide;
            if (!sim_slide(stickTab[i], dataPoint->positions.pos23, dataPoint->velocities.vel23, dataPoint->velocities.vel23 * gSineTable[fangle >> 4], dataPoint->velocities.vel23 * gCosineTable[fangle >> 4], fangle, fangle, camYaw, false, fineslide)) {
                continue;
            }
            // did it land in air?
            if (assess_floor(fineslide.endPos) != 1) {
                continue;
            }
            // simulate air movement
            AirInfo airmove;

            // did it land on ground?
            if (!sim_airstep(fineslide.endPos, fineslide.endSpeed, fineslide.endFacingAngle, true, airmove) || (assess_floor(airmove.endPos) != 2 && assess_floor(airmove.endPos) != 3)) {
                continue;
            }

            // the camera is going to move to a 1.5/98.5 mix of your new position, and its old position, on frame 1.
            // I don't think it's exact, it'll be off a bit when we do it in reality
            // but given our stupid-huge distances, the inaccuracy shouldn't matter.


            float camPos[3];
            float focus[3];
            float sPanDistance;
            fine_camera_yaw(airmove.endPos, dataPoint->positions.posCam1, fineslide.endFacingAngle, focus, &sPanDistance, camPos, false);
            float newCamPosition[3];
            newCamPosition[0] = 0.985f * dataPoint->positions.posCam1[0] + 0.015f * camPos[0];
            newCamPosition[1] = 0.985f * dataPoint->positions.posCam1[1] + 0.015f * camPos[1];
            newCamPosition[2] = 0.985f * dataPoint->positions.posCam1[2] + 0.015f * camPos[2];

            if (!membership_test(dataPoint->targets.point1, dataPoint->targets.point2, dataPoint->targets.point3, dataPoint->targets.point4, newCamPosition)) {
                continue;
            }
            // yaay! You got the camera to the right spot!

            // are we stable against OOB?
            bool stable = stability_check(airmove.endPos, airmove.endSpeed, fineslide.endFacingAngle);

            // also, gotta beat the benchmark speed.
            if (!stable || dataPoint->targets.benchmark > fabs(airmove.endSpeed)) {
                continue;
            }

            // log data if the solution is the fastest solution seen so far.
            if (abs(airmove.endSpeed) > abs(best_solution.velocities.vel24) || !foundSolution /* This second condition is probably unecessary.*/) {
                dataPoint->positions.pos24[0] = airmove.endPos[0];
                dataPoint->positions.pos24[1] = airmove.endPos[1];
                dataPoint->positions.pos24[2] = airmove.endPos[2];
                dataPoint->positions.posCam2[0] = newCamPosition[0];
                dataPoint->positions.posCam2[1] = newCamPosition[1];
                dataPoint->positions.posCam2[2] = newCamPosition[2];
                dataPoint->velocities.vel24 = airmove.endSpeed;
                dataPoint->sticks.stick23X = correct_stick(stickTab[i].stickX);
                dataPoint->sticks.stick23Y = correct_stick(stickTab[i].stickY);
                dataPoint->angles.facing23 = fangle;
                dataPoint->angles.cam23 = camYaw;

                best_solution = AllData(*dataPoint); // I think this copy constructor works

                foundSolution = true;
                best_solution_counter = counter;

                printf("T");
            }
            else {
                printf("t");
            }
            
            totalHits23++;

            // you need to do something here so we only log the data from the highest-speed thing that makes it this far!!
            // not sure what to do here.
            // return true;
        }
    }
    // return false;
}


void move22(AllData* dataPoint) {
    float lb = 0.9f;
    float ub = 0.94f;
    int camYaw = fix(crude_camera_yaw(dataPoint->positions.pos22, dataPoint->positions.posCam1));
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
            if (!stability_check(dataPoint->positions.pos22, dataPoint->velocities.vel22, fangle)) {
                continue;
            }
            // iterate over stick positions
            for (int i = 0; i < 20129; i++) {
                // simulate the slide
                FancySlideInfo fineslide;
                if (!sim_slide(stickTab[i], dataPoint->positions.pos22, dataPoint->velocities.vel22, dataPoint->velocities.vel22 * gSineTable[fangle >> 4], dataPoint->velocities.vel22 * gCosineTable[fangle >> 4], fangle, fangle, camYaw, false, fineslide)) {
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
                if (assess_floor(airmove.endPos) != 2 && assess_floor(airmove.endPos) != 3) {
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


                if (floorIdx < 8 || floorIdx > 32 || floorIdx == 22) {
                    continue;
                }

                float nextspeed = airmove.endSpeed;

                // an upper bound on our distance is one which would require a multiplier > 0.94 on the last slide.
                // there are two lower bounds. Requiring a multiplier < 0.9 on the last slide
                // or, one where, after a 0.94x multiplier, we're still below our benchmarking speed.
                if (fmaxf(lb * fabs(nextspeed), dataPoint->targets.benchmark) > ub * fabs(nextspeed)) {
                    continue;
                }

                // work out where you're aiming for specifically. Namely, the vicinity of the main universe.
                float destination[3];
                destination[0] = 0.0f;
                destination[2] = 0.0f;
                float dis = find_dis(airmove.endPos, destination);

                if (dis < ((1.0f + floor->normal[1]) / 4.0f) * fmaxf(lb * fabs(airmove.endSpeed), dataPoint->targets.benchmark)
                    || dis >((1.0f + floor->normal[1]) / 4.0f) * ub * fabs(airmove.endSpeed)) {
                    continue;
                }

                // the counter can be uncommented and a print statement added if you want to see.
                // otherwise, just log the data, proceed to the last step.
                (dataPoint->positions).pos23[0] = airmove.endPos[0];
                (dataPoint->positions).pos23[1] = airmove.endPos[1];
                (dataPoint->positions).pos23[2] = airmove.endPos[2];
                (dataPoint->velocities).vel23 = airmove.endSpeed;
                (dataPoint->sticks).stick22X = correct_stick(stickTab[i].stickX);
                (dataPoint->sticks).stick22Y = correct_stick(stickTab[i].stickY);
                (dataPoint->angles).facing22 = fangle;
                (dataPoint->angles).cam22 = camYaw;
                printf("s");
                totalHits22++;
                move23(dataPoint);
            }
        }
    }
    // return false;
}

void move21(AllData* dataPoint) {
    float lb = 0.9f * 0.9f;
    float ub = 0.94f * 0.94f;
    // keeping track of how many hits we have for our first motion is useful for partially running a computation
    // logging where you were, and skipping ahead.
    int camYaw = fix(crude_camera_yaw(dataPoint->positions.pos21, dataPoint->positions.posCam1));
    // iterate over AU's
    for (int t = 0; t < 8192; t++) {
        // can we do that AU?
        if (!attainableArctans[t]) {
            continue;
        }
        int fangle = fix(gArctanTable[t] + camYaw);

        // is it stable?
        if (!stability_check(dataPoint->positions.pos21, dataPoint->velocities.vel21, fangle)) {
            continue;
        }
        // iterate over sticks
        for (int i = 0; i < 20129; i++) {
            // simulate crouchslide.
            FancySlideInfo fineslide;
            if (!sim_slide(stickTab[i], dataPoint->positions.pos21, dataPoint->velocities.vel21, dataPoint->velocities.vel21 * gSineTable[fangle >> 4], dataPoint->velocities.vel21 * gCosineTable[fangle >> 4], fangle, fangle, camYaw, false, fineslide)) {
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

            // check to see if our speed is acceptable, though admittedly this is pretty fucking arcane.
            if (1.98f * fmaxf(lb * fabs(airmove.endSpeed), dataPoint->targets.benchmark) >= 2.0f * ub * fabs(airmove.endSpeed)) {
                continue;
            }
            // where we're aiming for.
            float destination[3];
            destination[0] = 0.0f;
            destination[2] = 0.0f;

            // computing the donut of "where we can reach" and "where can reach our target with the needed speed"
            // so we can aim for the intersection.

            DonutData chocolate = donut_compute(airmove.endPos, 0.9f * ((1.0f + floor->normal[1]) / 4.0f) * fabs(airmove.endSpeed), 0.94f * ((1.0f + floor->normal[1]) / 4.0f) * fabs(airmove.endSpeed), destination, ((1.0f + 0.98f) / 4.0f) * fmaxf(lb * fabs(airmove.endSpeed), dataPoint->targets.benchmark), ((1.0f + 1.0f) / 4.0f) * ub * fabs(airmove.endSpeed));
            if (chocolate.overlapArea < 10000.0f) {
                continue;
            }

            // log data, continue.
            printf("f(%d,%d)\n", counter, fangle);
            totalHits21++;
            (dataPoint->positions).pos22[0] = airmove.endPos[0];
            (dataPoint->positions).pos22[1] = airmove.endPos[1];
            (dataPoint->positions).pos22[2] = airmove.endPos[2];
            (dataPoint->velocities).vel22 = airmove.endSpeed;
            (dataPoint->sticks).stick21X = correct_stick(stickTab[i].stickX);
            (dataPoint->sticks).stick21Y = correct_stick(stickTab[i].stickY);
            (dataPoint->angles).facing21 = fangle;
            (dataPoint->angles).cam21 = camYaw;
            (dataPoint->donuts) = chocolate;
            // fuck with the counter to continue with partially terminated computations.
            if (counter > 24) {
                move22(dataPoint);
            }
            if (counter > 32)
                return;

            counter++;
        }
    }
    // return false;
}

int main(int argc, char* argv[]) {

    // changeable
    float cameraPosition[3];
    cameraPosition[0] = -1700.0f;
    cameraPosition[1] = -2300.0f;
    cameraPosition[2] = 500.0f;
    // changeable
    float firstPosition[3];
    firstPosition[0] = 343413856.0f;
    firstPosition[1] = -2916.003f;
    firstPosition[2] = 372965888.0f;
    // changeable
    float firstSpeed = -1509522432.0f;
    // changeable

    float point1[2];
    float point2[2];
    float point3[2];
    float point4[2];
    // changeable
    point1[0] = -21113.07031f;
    point1[1] = -24022.07031f;
    point2[0] = -19407.55469f;
    point2[1] = -24728.56836f;
    point3[0] = -6953.545898f;
    point3[1] = 29887.54688f;
    point4[0] = -8956.849609f;
    point4[1] = 29057.69336f;


    float targetSpeed = 6.0e+08 / (0.92f * 0.92f * 0.92f * 0.92f * (53.0f / 73.0f));

    std::string outFile = "camFile.csv";

    bool verbose = false;

    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "-h") || !strcmp(argv[i], "--help")) {
            printf("BitFS Second Phase Cam PU Route Brute Forcer.\n");
            printf("This program accepts the following options:\n\n");
            printf("-ms <speed>: Speed that Mario starts out with.\n");
            printf("             Default: %f\n", firstSpeed);
            printf("-ts <speed>: Speed that Mario must end near.\n");
            printf("             Default: %f\n", targetSpeed);
            printf("-cp <pos_x> <pos_y> <pos_z>: Position of the camera.\n");
            printf("             Default: %f %f %f\n", cameraPosition[0], cameraPosition[1], cameraPosition[2]);
            printf("-fp <pos_x> <pos_y> <pos_z>: Mario's starting position.\n");
            printf("             Default: %f %f %f\n", firstPosition[0], firstPosition[1], firstPosition[2]);
            printf("-pl <point1x> <point1z> <point2x> <point2z> <point3x> <point3z> <point4x> <point4z>: Region points\n");
            printf("             Default: %f %f %f %f %f %f %f %f\n", point1[0], point1[1], point2[0], point2[1], point3[0], point3[1], point4[0], point4[1]);
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
        else if (!strcmp(argv[i], "-pl")) {
            point1[0] = std::stoi(argv[i + 1]);
            point1[1] = std::stoi(argv[i + 2]);
            point2[0] = std::stoi(argv[i + 3]);
            point2[1] = std::stoi(argv[i + 4]);
            point3[0] = std::stoi(argv[i + 5]);
            point3[1] = std::stoi(argv[i + 6]);
            point4[0] = std::stoi(argv[i + 7]);
            point4[1] = std::stoi(argv[i + 8]);

            i += 8;
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
            printf("Target Speed: %f\n", targetSpeed);
            printf("Camera Position: (%f, %f, %f)\n", cameraPosition[0], cameraPosition[1], cameraPosition[2]);
            printf("First Position: (%f, %f, %f)\n", firstPosition[0], firstPosition[1], firstPosition[2]);
            printf("Points: ((%f, %f), (%f, %f), (%f, %f), (%f, %f))\n", point1[0], point1[1], point2[0], point2[1], point3[0], point3[1], point4[0], point4[1]);
        }
    }

    // initialize the tables

    AllData data;
    init_reverse_atan();
    initialise_floors();
    init_stick_tables();
    init_attainable_arctans();

    // and the data

    data.positions.posCam1[0] = cameraPosition[0];
    data.positions.posCam1[1] = cameraPosition[1];
    data.positions.posCam1[2] = cameraPosition[2];
    data.positions.pos21[0] = firstPosition[0];
    data.positions.pos21[1] = firstPosition[1];
    data.positions.pos21[2] = firstPosition[2];
    data.targets.point1[0] = point1[0];
    data.targets.point1[1] = point1[1];
    data.targets.point2[0] = point2[0];
    data.targets.point2[1] = point2[1];
    data.targets.point3[0] = point3[0];
    data.targets.point3[1] = point3[1];
    data.targets.point4[0] = point4[0];
    data.targets.point4[1] = point4[1];
    data.targets.benchmark = targetSpeed;
    data.velocities.vel21 = firstSpeed;

    std::ofstream wf(outFile);
    wf << std::fixed;
    wf << "Camera Position X, Camera Position Y, Camera Position Z, ";
    wf << "First Position X, First Position Y, First Position Z, ";
    wf << "First Facing Angle, First Cam Angle, First Velocity, First Stick X, First Stick Y, ";
    wf << "Second Position X, Second Position Y, Second Position Z, ";
    wf << "Second Facing Angle, Second Cam Angle, Second Velocity, Second Stick X, Second Stick Y, ";
    wf << "Third Position X, Third Position Y, Third Position Z, ";
    wf << "Third Facing Angle, Third Cam Angle, Third Velocity, Third Stick X, Third Stick Y, ";
    wf << "Fourth Position X, Fourth Position Y, Fourth Position Z, ";
    wf << "Fourth Velocity, ";
    wf << "Camera2 Position X, Camera2 Position Y, Camera2 Position Z, " << std::endl;

    move21(&data);

    if(foundSolution){
        wf << best_solution.positions.posCam1[0] << "," << best_solution.positions.posCam1[1] << "," << best_solution.positions.posCam1[2] << ",";
        wf << best_solution.positions.pos21[0] << "," << best_solution.positions.pos21[1] << "," << best_solution.positions.pos21[2] << ",";
        wf << best_solution.angles.facing21 << "," << best_solution.angles.cam21 << "," << best_solution.velocities.vel21 << "," << best_solution.sticks.stick21X << "," << best_solution.sticks.stick21Y << ",";
        wf << best_solution.positions.pos22[0] << "," << best_solution.positions.pos22[1] << ", " << best_solution.positions.pos22[2] << ",";
        wf << best_solution.angles.facing22 << "," << best_solution.angles.cam22 << "," << best_solution.velocities.vel22 << "," << best_solution.sticks.stick22X << "," << best_solution.sticks.stick22Y << ",";
        wf << best_solution.positions.pos23[0] << "," << best_solution.positions.pos23[1] << "," << best_solution.positions.pos23[2] << ",";
        wf << best_solution.angles.facing23 << "," << best_solution.angles.cam23 << "," << best_solution.velocities.vel23 << "," << best_solution.sticks.stick23X << "," << best_solution.sticks.stick23Y << ",";
        wf << best_solution.positions.pos24[0] << "," << best_solution.positions.pos24[1] << ", " << best_solution.positions.pos24[2] << ", ";
        wf << best_solution.velocities.vel24 << ",";
        wf << best_solution.positions.posCam2[0] << "," << best_solution.positions.posCam2[1] << "," << best_solution.positions.posCam2[2] << std::endl;
        wf.close();

        std::cout << "Best solution counter value: " << best_solution_counter << "\n";

        return 0;
    }
    else {
        printf("FAILURE");
        return 0;
    }
}


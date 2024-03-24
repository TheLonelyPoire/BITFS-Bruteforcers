#include "Stick.cuh"
#include "math.h"

#include <iostream>

#include "Floors.cuh"
#include "Surface.cuh"
#include "Trig.cuh"

namespace BITFS {

    __device__ StickTableData stickTabG[20129];
    StickTableData stickTab[20129];

    // novel function here. Basically, some stick positions are redundant, so what this does is iterates through the stick positions,
    //computes their magnitudes and angles, iterates through stick positions again to look for any exact copies, and if there's no exact
    //copy, adds its data to the table of unique stick positions. This cuts down on the number of stick positions to test.
    __global__ void init_stick_tablesG(bool backwardsOnly) {
        int counter = 0;
        for (int x = -122; x < 122; x++) {
            if (abs(x) < 2 && !(x == 0)) {
                continue;
            }
            for (int y = -121; y <= (backwardsOnly ? 0 : 122); y++) {
                if (abs(y) < 2 && !(y == 0)) {
                    continue;
                }
                float mag = sqrtf(x * x + y * y);
                float xS = x;
                float yS = y;
                if (mag > 64.0f) {
                    xS *= (64.0f / mag);
                    yS *= (64.0f / mag);
                    mag = 64.0f;
                }
                int an = atan2s(-yS, xS);
                bool duplicate = false;
                for (int i = 0; i < counter; i++) {
                    if (mag == stickTabG[i].magnitude && an == stickTabG[i].angle) {
                        duplicate = true;
                        break;
                    }
                }
                if (duplicate) {
                    continue;
                }
                stickTabG[counter].stickX = x;
                stickTabG[counter].stickY = y;
                stickTabG[counter].magnitude = mag;
                stickTabG[counter].angle = an;
                counter++;
            }
        }
    }


    // novel function here. Basically, some stick positions are redundant, so what this does is iterates through the stick positions,
    //computes their magnitudes and angles, iterates through stick positions again to look for any exact copies, and if there's no exact
    //copy, adds its data to the table of unique stick positions. This cuts down on the number of stick positions to test.
    void init_stick_tables(bool backwardsOnly) {
        int counter = 0;
        for (int x = -122; x < 122; x++) {
            if (abs(x) < 2 && !(x == 0)) {
                continue;
            }
            for (int y = -121; y <= (backwardsOnly ? 0 : 122); y++) {
                if (abs(y) < 2 && !(y == 0)) {
                    continue;
                }
                float mag = sqrtf(x * x + y * y);
                float xS = x;
                float yS = y;
                if (mag > 64.0f) {
                    xS *= (64.0f / mag);
                    yS *= (64.0f / mag);
                    mag = 64.0f;
                }
                int an = atan2s(-yS, xS);
                bool duplicate = false;
                for (int i = 0; i < counter; i++) {
                    if (mag == stickTab[i].magnitude && an == stickTab[i].angle) {
                        duplicate = true;
                        break;
                    }
                }
                if (duplicate) {
                    continue;
                }
                stickTab[counter].stickX = x;
                stickTab[counter].stickY = y;
                stickTab[counter].magnitude = mag;
                stickTab[counter].angle = an;
                counter++;
            }
        }

        std::cout << "Initialized " << counter << " Stick Table Entries!\n";
    }


    // converts internal stick positions back into input stick positions.
    __host__ __device__ int correct_stick(int stick) {
        if (stick >= 2) {
            stick = stick + 6;
            return stick;
        }
        else if (stick <= -2) {
            stick = stick - 6;
            return stick;
        }
        else {
            return 0;
        }
    }


    // Works out the stick position needed to hit a given target with a 1 frame crouchslide. Return value is whether or not a valid stick position was computed.
    __host__ __device__ bool infer_stick(float* startPos, float* endPos, float startSpeed, int angle, int startCameraYaw, StickTableData& output) {
        Surface* floorSet;
        #if !defined(__CUDA_ARCH__)
            floorSet = floors;
        #else
            floorSet = floorsG;
        #endif

        Surface* floor;
        float floorHeight;
        int floorIdx = find_floor(startPos, &floor, floorHeight, floorSet, total_floors);

        int travelAngle = fix(angle);
        float lPrime;

        // don't ask me to explain where this shit came from. It's a third-order taylor expansion of a quadratic equation
        // used to derive the stick position. It's done this way because the quadratic was susceptible to catastrophic cancellation
        // and division by zero, and this isnt. I at least verified it works, by testing all stick positions and comparing them to
        // the one recommended by this block of black magic.
        if (fabs(sm64_sins(travelAngle)) > fabs(sm64_coss(travelAngle))) {
            float cotang = sm64_coss(travelAngle) / sm64_sins(travelAngle);
            float wCotang = (startPos[2] - endPos[2]) / (startPos[0] - endPos[0]);
            float epsOverAlphC = 4.0f * (wCotang - cotang) / ((1.0f + cotang * wCotang) * (1.0f + cotang * wCotang));
            lPrime = -640.0f * 0.25f * (1.0f + cotang * wCotang) * (epsOverAlphC + (1.0f / 4.0f) * cotang * epsOverAlphC * epsOverAlphC + (1.0f / 8.0f) * cotang * cotang * epsOverAlphC * epsOverAlphC * epsOverAlphC);
        }
        else {
            float tang = sm64_sins(travelAngle) / sm64_coss(travelAngle);
            float wTang = (startPos[0] - endPos[0]) / (startPos[2] - endPos[2]);
            float epsOverAlphW = 4.0f * (tang - wTang) / ((tang * wTang + 1.0f) * (tang * wTang + 1.0f));
            lPrime = -640.0f * 0.25f * (tang * wTang + 1.0f) * (epsOverAlphW + (1.0f / 4.0f) * wTang * epsOverAlphW * epsOverAlphW + (1.0f / 8.0f) * wTang * wTang * epsOverAlphW * epsOverAlphW * epsOverAlphW);
        }

        float endSpeed = -(4.0f / floor->normal[1]) * sqrtf((startPos[0] - endPos[0]) * (startPos[0] - endPos[0]) + (startPos[2] - endPos[2]) * (startPos[2] - endPos[2]));
        float mPrime = 1600.0f * ((endSpeed / startSpeed) - 0.92f);

        int cameraYaw = (65536 + startCameraYaw) % 65536;
        float intendedMag = sqrtf(lPrime * lPrime + mPrime * mPrime);
        int intendedDYaw = atan2s(mPrime, lPrime);
        intendedDYaw = (intendedDYaw + 65536) % 65536;

        int stickAngle = intendedDYaw + travelAngle - cameraYaw;
        stickAngle = (stickAngle + 65536) % 65536;
        float stickMagnitude = sqrtf(128.0f * intendedMag);
        float yS = -stickMagnitude * sm64_coss(stickAngle);
        float xS = stickMagnitude * sm64_sins(stickAngle);

        int x = round(xS);
        int y = round(yS);

        output.stickX = x;
        output.stickY = y;

        if (x * x + y * y < 0)
        {
            printf("ERROR: SQUARE OF STICK VALUES HAS OVERFLOWED!\n\n");
            return false;
        }
        else if (x * x == 1 || y * y == 1) // Raw stick values of +/-1 are not allowed
        {
            return false;
        }
        else
        {
            output.magnitude = sqrtf(x * x + y * y);
        }
        output.angle = atan2s(-y, x);
        return true;
    }

}
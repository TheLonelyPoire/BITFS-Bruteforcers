#include "Stick.cuh"
#include "math.h"

#include "Floor.cuh"
#include "Surface.cuh"
#include "Trig.cuh"

namespace BITFS {

    __device__ StickTableData stickTab[20129];

    // novel function here. Basically, some stick positions are redundant, so what this does is iterates through the stick positions,
    //computes their magnitudes and angles, iterates through stick positions again to look for any exact copies, and if there's no exact
    //copy, adds its data to the table of unique stick positions. This cuts down on the number of stick positions to test.
    __global__ void init_stick_tables() {
        int counter = 0;
        for (int x = -122; x < 122; x++) {
            if (abs(x) < 2 && !(x == 0)) {
                continue;
            }
            for (int y = -121; y <= 122; y++) {
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
                int an = atan2sG(-yS, xS);
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
    }


    // converts internal stick positions back into input stick positions.
    __device__ int correct_stick(int stick) {
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


    // works out the stick position needed to hit a given target with a 1 frame crouchslide.
    __device__ StickTableData infer_stick(float* startPos, float* endPos, float startSpeed, int angle, int startCameraYaw) {

        Surface* floor;
        float floorHeight;
        int floorIdx = find_floor(startPos, &floor, floorHeight, floorsG, total_floorsG);

        int travelAngle = fix(angle);
        float lPrime;

        // don't ask me to explain where this shit came from. It's a third-order taylor expansion of a quadratic equation
        // used to derive the stick position. It's done this way because the quadratic was susceptible to catastrophic cancellation
        // and division by zero, and this isnt. I at least verified it works, by testing all stick positions and comparing them to
        // the one recommended by this block of black magic.
        if (fabs(gSineTableG[travelAngle >> 4]) > fabs(gCosineTableG[travelAngle >> 4])) {
            float cotang = gCosineTableG[travelAngle >> 4] / gSineTableG[travelAngle >> 4];
            float wCotang = (startPos[2] - endPos[2]) / (startPos[0] - endPos[0]);
            float epsOverAlphC = 4.0f * (wCotang - cotang) / ((1.0f + cotang * wCotang) * (1.0f + cotang * wCotang));
            lPrime = -640.0f * 0.25f * (1.0f + cotang * wCotang) * (epsOverAlphC + (1.0f / 4.0f) * cotang * epsOverAlphC * epsOverAlphC + (1.0f / 8.0f) * cotang * cotang * epsOverAlphC * epsOverAlphC * epsOverAlphC);
        }
        else {
            float tang = gSineTableG[travelAngle >> 4] / gCosineTableG[travelAngle >> 4];
            float wTang = (startPos[0] - endPos[0]) / (startPos[2] - endPos[2]);
            float epsOverAlphW = 4.0f * (tang - wTang) / ((tang * wTang + 1.0f) * (tang * wTang + 1.0f));
            lPrime = -640.0f * 0.25f * (tang * wTang + 1.0f) * (epsOverAlphW + (1.0f / 4.0f) * wTang * epsOverAlphW * epsOverAlphW + (1.0f / 8.0f) * wTang * wTang * epsOverAlphW * epsOverAlphW * epsOverAlphW);
        }

        float endSpeed = -(4.0f / floor->normal[1]) * sqrtf((startPos[0] - endPos[0]) * (startPos[0] - endPos[0]) + (startPos[2] - endPos[2]) * (startPos[2] - endPos[2]));
        float mPrime = 1600.0f * ((endSpeed / startSpeed) - 0.92f);

        int cameraYaw = (65536 + startCameraYaw) % 65536;
        float intendedMag = sqrtf(lPrime * lPrime + mPrime * mPrime);
        int intendedDYaw = atan2sG(mPrime, lPrime);
        intendedDYaw = (intendedDYaw + 65536) % 65536;

        int stickAngle = intendedDYaw + travelAngle - cameraYaw;
        stickAngle = (stickAngle + 65536) % 65536;
        float stickMagnitude = sqrtf(128.0f * intendedMag);
        float yS = -stickMagnitude * gCosineTableG[stickAngle >> 4];
        float xS = stickMagnitude * gSineTableG[stickAngle >> 4];

        int x = round(xS);
        int y = round(yS);

        struct StickTableData solution;
        solution.stickX = x;
        solution.stickY = y;
        solution.magnitude = sqrtf(x * x + y * y);
        solution.angle = atan2sG(-y, x);
        return solution;
    }

}
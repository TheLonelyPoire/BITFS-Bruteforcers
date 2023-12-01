#include "Camera.cuh"
#include "math.h"

#include "Floors.cuh"
#include "Surface.cuh"
#include "Trig.cuh"
#include "VMath.cuh"

namespace BITFS {

    bool validCameraAngles[65536];

    void init_camera_angles() {
        for (int i = 0; i < 65536; i++) {
            validCameraAngles[i] = false;
        }
        for (int hau = 0; hau < 4096; hau++) {
            validCameraAngles[fix(atan2s(-gCosineTable[hau], -gSineTable[hau]))] = true;
        }
    }

    // accurate at PU distances!
    __host__ __device__ int crude_camera_yaw(float* currentPosition, float* lakituPosition) {

        // Choose the appropriate trig tables depending on whether the function is called from the host or from the device
        float* cosineTable, * sineTable;
#if !defined(__CUDA_ARCH__)
        cosineTable = gCosineTable;
        sineTable = gSineTable;
#else
        cosineTable = gCosineTableG;
        sineTable = gSineTableG;
#endif

        float cameraFocus[3] = { currentPosition[0], currentPosition[1], currentPosition[2] };

        float dx = cameraFocus[0] - lakituPosition[0];
        float dz = cameraFocus[2] - lakituPosition[2];

        int cameraYaw = atan2s(dz, dx);

        return atan2s(-cosineTable[fix((int)cameraYaw) >> 4], -sineTable[fix((int)cameraYaw) >> 4]);
    }


    __host__ __device__ int fine_camera_yaw(float* currentPosition, float* lakituPosition, short faceAngle, float* focus, float* sPanDistance, float* camPos, bool onPole) {

        float baseCameraDist = 1000.0;
        unsigned short baseCameraPitch = 0x05B0;
        unsigned short baseCameraYaw = -8192;

        float cameraPos[3]; 

        vec3f_set_dist_and_angle(currentPosition, cameraPos, baseCameraDist, baseCameraPitch, baseCameraYaw);

        camPos[0] = cameraPos[0];
        camPos[1] = cameraPos[1];
        camPos[2] = cameraPos[2];

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
        pan[2] = sm64_sins(0xC00) * cameraDist;

        temp[0] = pan[0];
        temp[1] = pan[1];
        temp[2] = pan[2];

        pan[0] = temp[2] * sm64_sins(faceAngle) + temp[0] * sm64_coss(faceAngle);
        pan[2] = temp[2] * sm64_coss(faceAngle) - temp[0] * sm64_sins(faceAngle);

        // rotate in the opposite direction
        cameraYaw = -cameraYaw;

        temp[0] = pan[0];
        temp[1] = pan[1];
        temp[2] = pan[2];

        pan[0] = temp[2] * sm64_sins(faceAngle) + temp[0] * sm64_coss(faceAngle);
        pan[2] = temp[2] * sm64_coss(faceAngle) - temp[0] * sm64_sins(faceAngle);

        // Only pan left or right
        pan[2] = 0.f;

        cameraYaw = -cameraYaw;

        temp[0] = onPole ? -pan[0] : pan[0];
        temp[1] = pan[1];
        temp[2] = pan[2];

        sPanDistance[0] = temp[0];

        pan[0] = temp[2] * sm64_sins(cameraYaw) + temp[0] * sm64_coss(cameraYaw);
        pan[2] = temp[2] * sm64_coss(cameraYaw) - temp[0] * sm64_sins(cameraYaw);

        float cameraFocus[3] = { currentPosition[0] + pan[0], currentPosition[1] + 125.0f + pan[1], currentPosition[2] + pan[2] };

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

        vec3f_set_dist_and_angle(lakituPosition, cameraFocus, cameraDist, cameraPitch, cameraYaw);

        focus[0] = cameraFocus[0];
        focus[1] = cameraFocus[1];
        focus[2] = cameraFocus[2];

        return atan2s(lakituPosition[2] - cameraFocus[2], lakituPosition[0] - cameraFocus[0]);
    }


    __host__ __device__ int tenk_camera_yaw(float* twoAgoPosition, float* oneAgoPosition, float* lakituPosition, short twoAgoFaceAngle, short oneAgoFaceAngle, float* oldFocus, float* oldPan, float* oldCamPos) {

        // Choose the appropriate trig tables and floor set depending on whether the function is called from the host or from the device
        float* cosineTable, * sineTable;
        Surface* floorSet;
#if !defined(__CUDA_ARCH__)
        cosineTable = gCosineTable;
        sineTable = gSineTable;
        floorSet = floors;
#else
        cosineTable = gCosineTableG;
        sineTable = gSineTableG;
        floorSet = floorsG;
#endif

        float baseCameraDist = 1400.0;
        short baseCameraPitch = 0x05B0;
        short baseCameraYaw = -8192;
        baseCameraPitch = baseCameraPitch + 2304;

        float cameraPos[3] = { twoAgoPosition[0] + baseCameraDist * cosineTable[fix((int)baseCameraPitch) >> 4] * sineTable[fix((int)baseCameraYaw) >> 4],
                            twoAgoPosition[1] + 125.0f + baseCameraDist * sineTable[fix((int)baseCameraPitch) >> 4],
                            twoAgoPosition[2] + baseCameraDist * cosineTable[fix((int)baseCameraPitch) >> 4] * cosineTable[fix((int)baseCameraYaw) >> 4]
        };
        // posY was removed from the camera position height because it's close to 0.

        float pan[3] = { 0, 0, 0 };
        float temp[3] = { 0, 0, 0 };

        // Get distance and angle from camera to Mario.
        float dx = twoAgoPosition[0] - oldCamPos[0];
        float dy = twoAgoPosition[1] - oldCamPos[1];
        float dz = twoAgoPosition[2] - oldCamPos[2];

        float cameraDist = sqrtf(dx * dx + dy * dy + dz * dz);
        float cameraPitch = atan2s(sqrtf(dx * dx + dz * dz), dy);
        float cameraYaw = atan2s(dz, dx);

        // The camera will pan ahead up to about 30% of the camera's distance to Mario.
        pan[2] = sineTable[0xC0] * cameraDist;

        temp[0] = pan[0];
        temp[1] = pan[1];
        temp[2] = pan[2];

        pan[0] = temp[2] * sineTable[fix((int)twoAgoFaceAngle) >> 4] + temp[0] * cosineTable[fix((int)twoAgoFaceAngle) >> 4];
        pan[2] = temp[2] * cosineTable[fix((int)twoAgoFaceAngle) >> 4] - temp[0] * sineTable[fix((int)twoAgoFaceAngle) >> 4];

        // rotate in the opposite direction
        cameraYaw = -cameraYaw;

        temp[0] = pan[0];
        temp[1] = pan[1];
        temp[2] = pan[2];

        pan[0] = temp[2] * sineTable[fix((int)cameraYaw) >> 4] + temp[0] * cosineTable[fix((int)cameraYaw) >> 4];
        pan[2] = temp[2] * cosineTable[fix((int)cameraYaw) >> 4] - temp[0] * sineTable[fix((int)cameraYaw) >> 4];

        // Only pan left or right
        pan[2] = 0.f;

        cameraYaw = -cameraYaw;

        float sPanDistance = *oldPan + (pan[0] - *oldPan) * 0.025f;

        temp[0] = sPanDistance;
        temp[1] = pan[1];
        temp[2] = pan[2];

        pan[0] = temp[2] * sineTable[fix((int)cameraYaw) >> 4] + temp[0] * cosineTable[fix((int)cameraYaw) >> 4];
        pan[2] = temp[2] * cosineTable[fix((int)cameraYaw) >> 4] - temp[0] * sineTable[fix((int)cameraYaw) >> 4];

        float newFocus[3] = { twoAgoPosition[0] + pan[0], twoAgoPosition[1] + 125.0f + pan[1], twoAgoPosition[2] + pan[2] };
        float cameraFocus[3];

        cameraFocus[0] = oldFocus[0] + 0.8f * (newFocus[0] - oldFocus[0]);
        cameraFocus[1] = oldFocus[1] + 0.3f * (newFocus[1] - oldFocus[1]);
        cameraFocus[2] = oldFocus[2] + 0.8f * (newFocus[2] - oldFocus[2]);

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

        cameraFocus[0] = lakituPosition[0] + cameraDist * cosineTable[fix((int)cameraPitch) >> 4] * sineTable[fix((int)cameraYaw) >> 4];
        cameraFocus[1] = lakituPosition[1] + cameraDist * sineTable[fix((int)cameraPitch) >> 4];
        cameraFocus[2] = lakituPosition[2] + cameraDist * cosineTable[fix((int)cameraPitch) >> 4] * cosineTable[fix((int)cameraYaw) >> 4];

        // at this point, we're done with the camera computations that take place at the end of the frame which ends with you standing idle on the ground below the pole! Time for the events of the next frame!

        Surface* floor;
        float floorY;
        int floorIdx = find_floor(oneAgoPosition, &floor, floorY, floorSet, total_floors);
        float focusY = (floorY - oneAgoPosition[1]) * 0.9f;

        pan[0] = 0.0f;
        pan[1] = 0.0f;
        pan[2] = 0.0f;

        // Get distance and angle from camera to Mario.
        dx = oneAgoPosition[0] - cameraPos[0];
        dy = oneAgoPosition[1] - cameraPos[1];
        dz = oneAgoPosition[2] - cameraPos[2];

        cameraDist = sqrtf(dx * dx + dy * dy + dz * dz);
        cameraPitch = atan2s(sqrtf(dx * dx + dz * dz), dy);
        cameraYaw = atan2s(dz, dx);

        // The camera will pan ahead up to about 30% of the camera's distance to Mario.
        pan[2] = sineTable[0xC0] * cameraDist;

        temp[0] = pan[0];
        temp[1] = pan[1];
        temp[2] = pan[2];

        pan[0] = temp[2] * sineTable[fix((int)oneAgoFaceAngle) >> 4] + temp[0] * cosineTable[fix((int)oneAgoFaceAngle) >> 4];
        pan[2] = temp[2] * cosineTable[fix((int)oneAgoFaceAngle) >> 4] - temp[0] * sineTable[fix((int)oneAgoFaceAngle) >> 4];

        // rotate in the opposite direction
        cameraYaw = -cameraYaw;

        temp[0] = pan[0];
        temp[1] = pan[1];
        temp[2] = pan[2];

        pan[0] = temp[2] * sineTable[fix((int)cameraYaw) >> 4] + temp[0] * cosineTable[fix((int)cameraYaw) >> 4];
        pan[2] = temp[2] * cosineTable[fix((int)cameraYaw) >> 4] - temp[0] * sineTable[fix((int)cameraYaw) >> 4];

        // Only pan left or right
        pan[2] = 0.f;

        cameraYaw = -cameraYaw;

        sPanDistance = sPanDistance + (pan[0] - sPanDistance) * 0.025f;

        temp[0] = sPanDistance;
        temp[1] = pan[1];
        temp[2] = pan[2];

        pan[0] = temp[2] * sineTable[fix((int)cameraYaw) >> 4] + temp[0] * cosineTable[fix((int)cameraYaw) >> 4];
        pan[2] = temp[2] * cosineTable[fix((int)cameraYaw) >> 4] - temp[0] * sineTable[fix((int)cameraYaw) >> 4];

        newFocus[0] = oneAgoPosition[0] + pan[0];
        newFocus[1] = oneAgoPosition[1] + focusY + 125.0f + pan[1];
        newFocus[2] = oneAgoPosition[2] + pan[2];

        cameraFocus[0] = cameraFocus[0] + 0.8f * (newFocus[0] - cameraFocus[0]);
        cameraFocus[1] = cameraFocus[1] + 0.3f * (newFocus[1] - cameraFocus[1]);
        cameraFocus[2] = cameraFocus[2] + 0.8f * (newFocus[2] - cameraFocus[2]);

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

        cameraFocus[0] = lakituPosition[0] + cameraDist * cosineTable[fix((int)cameraPitch) >> 4] * sineTable[fix((int)cameraYaw) >> 4];
        cameraFocus[1] = lakituPosition[1] + cameraDist * sineTable[fix((int)cameraPitch) >> 4];
        cameraFocus[2] = lakituPosition[2] + cameraDist * cosineTable[fix((int)cameraPitch) >> 4] * cosineTable[fix((int)cameraYaw) >> 4];

        // alright, at this point, we are done with all this shit! Just the final yaw calculation that takes place on the last frame.


        return atan2s(lakituPosition[2] - cameraFocus[2], lakituPosition[0] - cameraFocus[0]);
    }

}
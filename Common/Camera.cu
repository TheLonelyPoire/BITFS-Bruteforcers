#include "Camera.cuh"
#include "math.h"

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


    __host__ __device__ int fine_camera_yaw(float* currentPosition, float* lakituPosition, short faceAngle, bool onPole) {

        // Choose the appropriate trig tables depending on whether the function is called from the host or from the device
        float* cosineTable, * sineTable;
        #if !defined(__CUDA_ARCH__)
            cosineTable = gCosineTable;
            sineTable = gSineTable;
        #else
            cosineTable = gCosineTableG;
            sineTable = gSineTableG;
        #endif

        float baseCameraDist = 1400.0;
        short baseCameraPitch = 0x05B0;
        short baseCameraYaw = -8192;

        //Surface* floor;
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

        float cameraPos[3] = { currentPosition[0] + baseCameraDist * cosineTable[fix((int)baseCameraPitch) >> 4] * sineTable[fix((int)baseCameraYaw) >> 4],
                            currentPosition[1] + 125.0f + baseCameraDist * sineTable[fix((int)baseCameraPitch) >> 4],
                            currentPosition[2] + baseCameraDist * cosineTable[fix((int)baseCameraPitch) >> 4] * cosineTable[fix((int)baseCameraYaw) >> 4]
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
        pan[2] = sineTable[0xC0] * cameraDist;

        temp[0] = pan[0];
        temp[1] = pan[1];
        temp[2] = pan[2];

        pan[0] = temp[2] * sineTable[fix((int)faceAngle) >> 4] + temp[0] * cosineTable[fix((int)faceAngle) >> 4];
        pan[2] = temp[2] * cosineTable[fix((int)faceAngle) >> 4] - temp[0] * sineTable[fix((int)faceAngle) >> 4];

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

        temp[0] = onPole ? -pan[0] : pan[0];
        temp[1] = pan[1];
        temp[2] = pan[2];

        pan[0] = temp[2] * sineTable[fix((int)cameraYaw) >> 4] + temp[0] * cosineTable[fix((int)cameraYaw) >> 4];
        pan[2] = temp[2] * cosineTable[fix((int)cameraYaw) >> 4] - temp[0] * sineTable[fix((int)cameraYaw) >> 4];

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

        cameraFocus[0] = lakituPosition[0] + cameraDist * cosineTable[fix((int)cameraPitch) >> 4] * sineTable[fix((int)cameraYaw) >> 4];
        cameraFocus[1] = lakituPosition[1] + cameraDist * sineTable[fix((int)cameraPitch) >> 4];
        cameraFocus[2] = lakituPosition[2] + cameraDist * cosineTable[fix((int)cameraPitch) >> 4] * cosineTable[fix((int)cameraYaw) >> 4];

        return atan2s(lakituPosition[2] - cameraFocus[2], lakituPosition[0] - cameraFocus[0]);
    }


    __host__ __device__ int tenk_camera_yaw(float* currentPosition, float* lakituPosition, short faceAngle, float* trueCameraFocus, float* truePan, bool onPole) {
        
        // Choose the appropriate trig tables depending on whether the function is called from the host or from the device
        float* cosineTable, * sineTable;
        #if !defined(__CUDA_ARCH__)
            cosineTable = gCosineTable;
            sineTable = gSineTable;
        #else
            cosineTable = gCosineTableG;
            sineTable = gSineTableG;
        #endif
        
        float baseCameraDist = 1400.0;
        short baseCameraPitch = 0x05B0;
        short baseCameraYaw = -8192;

        //Surface* floor;
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

        float cameraPos[3] = { currentPosition[0] + baseCameraDist * cosineTable[fix((int)baseCameraPitch) >> 4] * sineTable[fix((int)baseCameraYaw) >> 4],
                            currentPosition[1] + 125.0f + baseCameraDist * sineTable[fix((int)baseCameraPitch) >> 4],
                            currentPosition[2] + baseCameraDist * cosineTable[fix((int)baseCameraPitch) >> 4] * cosineTable[fix((int)baseCameraYaw) >> 4]
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
        pan[2] = sineTable[0xC0] * cameraDist;

        temp[0] = pan[0];
        temp[1] = pan[1];
        temp[2] = pan[2];

        pan[0] = temp[2] * sineTable[fix((int)faceAngle) >> 4] + temp[0] * cosineTable[fix((int)faceAngle) >> 4];
        pan[2] = temp[2] * cosineTable[fix((int)faceAngle) >> 4] - temp[0] * sineTable[fix((int)faceAngle) >> 4];

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

        temp[0] = onPole ? -pan[0] : 0.975f * truePan[0] + 0.025f * pan[0];
        temp[1] = pan[1];
        temp[2] = pan[2];

        truePan[0] = temp[0];
        truePan[1] = temp[1];
        truePan[2] = temp[2];

        pan[0] = temp[2] * sineTable[fix((int)cameraYaw) >> 4] + temp[0] * cosineTable[fix((int)cameraYaw) >> 4];
        pan[2] = temp[2] * cosineTable[fix((int)cameraYaw) >> 4] - temp[0] * sineTable[fix((int)cameraYaw) >> 4];

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

        cameraFocus[0] = lakituPosition[0] + cameraDist * cosineTable[fix((int)cameraPitch) >> 4] * sineTable[fix((int)cameraYaw) >> 4];
        cameraFocus[1] = lakituPosition[1] + cameraDist * sineTable[fix((int)cameraPitch) >> 4];
        cameraFocus[2] = lakituPosition[2] + cameraDist * cosineTable[fix((int)cameraPitch) >> 4] * cosineTable[fix((int)cameraYaw) >> 4];

        trueCameraFocus[0] = trueCameraFocus[0] + 0.8f * (cameraFocus[0] - trueCameraFocus[0]);
        trueCameraFocus[1] = trueCameraFocus[1] + 0.3f * (cameraFocus[1] - trueCameraFocus[1]);
        trueCameraFocus[2] = trueCameraFocus[2] + 0.8f * (cameraFocus[2] - trueCameraFocus[2]);

        return atan2s(lakituPosition[2] - trueCameraFocus[2], lakituPosition[0] - trueCameraFocus[0]);
    }

}
#include "Camera.cuh"

namespace BITFS {

    // accurate at PU distances!
    __device__ int crude_camera_yawG(float* currentPosition, float* lakituPosition) {

        float cameraFocus[3] = { currentPosition[0], currentPosition[1], currentPosition[2] };

        float dx = cameraFocus[0] - lakituPosition[0];
        float dz = cameraFocus[2] - lakituPosition[2];

        int cameraYaw = atan2sG(dz, dx);

        return atan2sG(-gCosineTableG[fix((int)cameraYaw) >> 4], -gSineTableG[fix((int)cameraYaw) >> 4]);
    }


    // CPU version of the above function.
    int crude_camera_yaw(float* currentPosition, float* lakituPosition) {

        float cameraFocus[3] = { currentPosition[0], currentPosition[1], currentPosition[2] };

        float dx = cameraFocus[0] - lakituPosition[0];
        float dz = cameraFocus[2] - lakituPosition[2];

        int cameraYaw = atan2s(dz, dx);

        return atan2s(-gCosineTable[fix((int)cameraYaw) >> 4], -gSineTable[fix((int)cameraYaw) >> 4]);
    }

}
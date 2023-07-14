#include "Camera.cuh"

namespace BITFS {

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

}
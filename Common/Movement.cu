#include "Movement.cuh"
#include "math.h"

#include "Floor.cuh"
#include "Trig.cuh"

namespace BITFS {

    // computes 1QF of crouchslide, given stick position, starting position, starting speed, starting facing angle, camera angle.
    __host__ __device__ bool sim_slide(StickTableData stick, float* startPos, float forward_speed, float vX, float vZ, int faceAngle, int slideYaw, int camera, bool usePolePlatform, FancySlideInfo& output) {

        // Choose the appropriate trig tables depending on whether the function is called from the host or from the device
        float *cosineTable, *sineTable;
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

        // initialize some variables we'll be reusing because they get updated
        float nextPos[3];
        nextPos[0] = startPos[0];
        nextPos[1] = startPos[1];
        nextPos[2] = startPos[2];
        int facingAngle = fix(faceAngle);
        int slidingYaw = fix(slideYaw);
        int cameraYaw = camera;

        float intendedMag = ((stick.magnitude / 64.0f) * (stick.magnitude / 64.0f)) * 32.0f;

        // and the angle stuff
        int intendedDYaw = stick.angle + cameraYaw - slidingYaw;
        intendedDYaw = fix(intendedDYaw);
        float forward = cosineTable[intendedDYaw >> 4];
        float sideward = sineTable[intendedDYaw >> 4];

        // and the 10k loss factor stuff.
        if (forward < 0.0f && forward_speed >= 0.0f) {
            forward *= 0.5f + 0.5f * forward_speed / 100.0f;
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

        Surface* floor;
        float floorHeight;
        int floorIdx = find_floor(nextPos, &floor, floorHeight, floorSet, total_floors);

        if (floorIdx == -1 && !usePolePlatform){
            return false;
        }

        float normalY = usePolePlatform ? 0.99999994 : floor->normal[1];
        int slopeAngle = usePolePlatform ? 0 : atan2s(floor->normal[2], floor->normal[0]);
        slopeAngle = fix(slopeAngle);
        float steepness = usePolePlatform ? 0 : sqrtf(floor->normal[0] * floor->normal[0] + floor->normal[2] * floor->normal[2]);
        velX += 7.0f * steepness * sineTable[slopeAngle >> 4];
        velZ += 7.0f * steepness * cosineTable[slopeAngle >> 4];
        velX *= lossFactor;
        velZ *= lossFactor;
        nextPos[0] = nextPos[0] + normalY * (velX / 4.0f);
        nextPos[2] = nextPos[2] + normalY * (velZ / 4.0f);

        // and update the sliding yaw
        slidingYaw = atan2s(velZ, velX);
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
            forward_speed = sqrtf(velX * velX + velZ * velZ);
        }
        else {
            forward_speed = -sqrtf(velX * velX + velZ * velZ);
        }

        // and your new angle
        facingAngle = slidingYaw + newFacingDYaw;
        facingAngle = fix(facingAngle);
        slidingYaw = fix(slidingYaw);

        // and set output fields
        output.endFacingAngle = facingAngle;
        output.endSlidingAngle = slidingYaw;
        output.endSpeed = forward_speed;
        output.endPos[0] = nextPos[0];
        output.endPos[1] = nextPos[1];
        output.endPos[2] = nextPos[2];

        return true;
    }


    //simulates 1QF of HAU-aligned travel in the air, including floor snap up for recalculating things and being precise about it.
    __host__ __device__ bool sim_airstep(float* initialPos, float initialSpeed, int initialAngle, bool first, AirInfo& output) {
        float* cosineTable, *sineTable;
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
        
        float nextPos[3];
        nextPos[0] = initialPos[0];
        nextPos[1] = initialPos[1];
        nextPos[2] = initialPos[2];
        int angle = (65536 + initialAngle) % 65536;
        float speed;
        // simulate the speed loss from going forward or backwards.
        if (first) {
            speed = initialSpeed > 0.0f ? initialSpeed - 0.35f : initialSpeed + 0.35f;
            speed = speed > 0.0f ? speed - 1.0f : speed + 2.0f;
        }
        else {
            speed = initialSpeed;
        }

        float velX = speed * sineTable[angle >> 4];
        float velZ = speed * cosineTable[angle >> 4];

        nextPos[0] += velX / 4.0f;
        nextPos[2] += velZ / 4.0f;

        Surface* floor;
        float floorheight;
        int floorIdx = find_floor(nextPos, &floor, floorheight, floorSet, total_floors);
        
        if (floorIdx == -1) {
            output.endSpeed = initialSpeed;
            output.endPos[0] = initialPos[0];
            output.endPos[1] = initialPos[1];
            output.endPos[2] = initialPos[2];
            return false;
        }

        nextPos[1] = fmaxf(floorheight, nextPos[1]);

        output.endSpeed = speed;
        output.endPos[0] = nextPos[0];
        output.endPos[1] = nextPos[1];
        output.endPos[2] = nextPos[2];

        return true;
    }

}
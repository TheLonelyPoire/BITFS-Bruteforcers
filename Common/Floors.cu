#include "Floors.cuh"
#include "Trig.cuh"

namespace BITFS {

    __device__ Surface floorsG[total_floors];
    Surface floors[total_floors];

    int keyFloors[7][4][2];
    int keyCenter[7][2];
    float thatNearOneConstant;

    __global__ void initialise_floorsG() {
        // I chopped down the number of relevant floors, fewer things to worry about here
        floorsG[0] = Surface(2662, -2457, -306, 2662, -2764, 61, 3072, -2764, 61);
        floorsG[1] = Surface(2662, -2457, -306, 3072, -2764, 61, 3072, -2457, -306);
        // 0 and 1 are the metal ramp up from the bottleneck
        floorsG[2] = Surface(-4453, -2661, -613, -4607, -3071, -306, -4453, -2661, -306);
        floorsG[3] = Surface(-4453, -2661, -613, -4607, -3071, -613, -4607, -3071, -306);
        // 2 and 3 are the left side of the 1-up platform
        floorsG[4] = Surface(-4453, -2661, -306, -4146, -2661, -306, -4146, -2661, -613);
        floorsG[5] = Surface(-4453, -2661, -306, -4146, -2661, -613, -4453, -2661, -613);
        // 4 and 5 are the top of the 1-up platform
        floorsG[6] = Surface(-4453, -2743, -306, -4453, -2743, 307, -4146, -2743, 307);
        floorsG[7] = Surface(-4453, -2743, -306, -4146, -2743, 307, -4146, -2743, -306);
        // 6 and 7 are the thingy sticking out of the 1-up platform
        floorsG[8] = Surface(3379, -2764, -347, 4301, -2764, -40, 3994, -2764, -347);
        floorsG[9] = Surface(3379, -2764, -347, 4301, -2764, 573, 4301, -2764, -40);
        floorsG[10] = Surface(3379, -2764, -347, 3072, -2764, -40, 3072, -2764, 573);
        floorsG[11] = Surface(3379, -2764, -347, 3072, -2764, 573, 3379, -2764, 881);
        floorsG[12] = Surface(3379, -2764, -347, 3994, -2764, 881, 4301, -2764, 573);
        floorsG[13] = Surface(3379, -2764, -347, 3379, -2764, 881, 3994, -2764, 881);
        floorsG[14] = Surface(2662, -2764, -347, 2048, -2764, -347, 2048, -2764, 881);
        floorsG[15] = Surface(2662, -2764, -347, 2048, -2764, 881, 2662, -2764, 881);
        floorsG[16] = Surface(3072, -2764, 61, 2662, -2764, 471, 3072, -2764, 471);
        floorsG[17] = Surface(3072, -2764, 61, 2662, -2764, 61, 2662, -2764, 471);
        // the octagon region, and chokepoint.
        floorsG[18] = Surface(-7065, -2764, -511, -7986, -2764, 512, -7065, -2764, 512);
        floorsG[19] = Surface(-7065, -2764, -511, -7986, -2764, -511, -7986, -2764, 512);
        // starting rectangle"dynamic initialization is not supported for a device variable"
        floorsG[20] = Surface(-7065, -2764, 307, -6553, -2866, 307, -6553, -2866, -306);
        floorsG[21] = Surface(-7065, -2764, 307, -6553, -2866, -306, -7065, -2764, -306);
        // starting slope down
        floorsG[22] = Surface(-6553, -2866, 307, -7065, -3071, 322, -6041, -3071, 307);
        // steeple triangle 1
        floorsG[23] = Surface(-306, -2866, 307, 0, -2866, 922, 0, -2866, -306);
        floorsG[24] = Surface(-306, -2866, 307, 0, -2866, -306, -306, -2866, -306);
        // rectangle before wavy lava moving plat
        floorsG[25] = Surface(-6041, -2866, -306, -6553, -2866, 307, -6041, -2866, 307);
        floorsG[26] = Surface(-6041, -2866, -306, -6553, -2866, -306, -6553, -2866, 307);
        // lowest rectangle in beginning region
        floorsG[27] = Surface(5222, -2917, 573, 6298, -2917, 573, 6298, -2917, -40);
        // 27 and 30 are the triangles for our rectangle of interest
        floorsG[28] = Surface(5222, -2917, 573, 4301, -2764, -40, 4301, -2764, 573);
        floorsG[29] = Surface(5222, -2917, 573, 5222, -2917, -40, 4301, -2764, -40);
        // ramp leading to rectangle of interest
        floorsG[30] = Surface(5222, -2917, 573, 6298, -2917, -40, 5222, -2917, -40);
        // other rectangle of interest normal
        floorsG[31] = Surface(-921, -3020, 307, -921, -3020, 922, -306, -2866, 307);
        floorsG[32] = Surface(-921, -3020, 922, 0, -2866, 922, -306, -2866, 307);
        // ramp down to lava after track platform ride ends
        floorsG[33] = Surface(-3993, -3071, -613, -4146, -2661, -306, -3993, -3071, -306);
        floorsG[34] = Surface(-3993, -3071, -613, -4146, -2661, -613, -4146, -2661, -306);
        // right side of the 1-up platform
        floorsG[35] = Surface(-7065, -3071, 322, -6553, -2866, 307, -7065, -2866, 307);
        // the other weird steeple triangle
        floorsG[36] = Surface(-8191, -3071, 8192, 8192, -3071, -8191, -8191, -3071, -8191);
        floorsG[37] = Surface(-8191, -3071, 8192, 8192, -3071, 8192, 8192, -3071, -8191);
        // lava
    }


    void initialise_floors() {
        // I chopped down the number of relevant floors, fewer things to worry about here
        floors[0] = Surface(2662, -2457, -306, 2662, -2764, 61, 3072, -2764, 61);
        floors[1] = Surface(2662, -2457, -306, 3072, -2764, 61, 3072, -2457, -306);
        // 0 and 1 are the metal ramp up from the bottleneck
        floors[2] = Surface(-4453, -2661, -613, -4607, -3071, -306, -4453, -2661, -306);
        floors[3] = Surface(-4453, -2661, -613, -4607, -3071, -613, -4607, -3071, -306);
        // 2 and 3 are the left side of the 1-up platform
        floors[4] = Surface(-4453, -2661, -306, -4146, -2661, -306, -4146, -2661, -613);
        floors[5] = Surface(-4453, -2661, -306, -4146, -2661, -613, -4453, -2661, -613);
        // 4 and 5 are the top of the 1-up platform
        floors[6] = Surface(-4453, -2743, -306, -4453, -2743, 307, -4146, -2743, 307);
        floors[7] = Surface(-4453, -2743, -306, -4146, -2743, 307, -4146, -2743, -306);
        // 6 and 7 are the thingy sticking out of the 1-up platform
        floors[8] = Surface(3379, -2764, -347, 4301, -2764, -40, 3994, -2764, -347);
        floors[9] = Surface(3379, -2764, -347, 4301, -2764, 573, 4301, -2764, -40);
        floors[10] = Surface(3379, -2764, -347, 3072, -2764, -40, 3072, -2764, 573);
        floors[11] = Surface(3379, -2764, -347, 3072, -2764, 573, 3379, -2764, 881);
        floors[12] = Surface(3379, -2764, -347, 3994, -2764, 881, 4301, -2764, 573);
        floors[13] = Surface(3379, -2764, -347, 3379, -2764, 881, 3994, -2764, 881);
        floors[14] = Surface(2662, -2764, -347, 2048, -2764, -347, 2048, -2764, 881);
        floors[15] = Surface(2662, -2764, -347, 2048, -2764, 881, 2662, -2764, 881);
        floors[16] = Surface(3072, -2764, 61, 2662, -2764, 471, 3072, -2764, 471);
        floors[17] = Surface(3072, -2764, 61, 2662, -2764, 61, 2662, -2764, 471);
        // the octagon region, and chokepoint.
        floors[18] = Surface(-7065, -2764, -511, -7986, -2764, 512, -7065, -2764, 512);
        floors[19] = Surface(-7065, -2764, -511, -7986, -2764, -511, -7986, -2764, 512);
        // starting rectangle"dynamic initialization is not supported for a device variable"
        floors[20] = Surface(-7065, -2764, 307, -6553, -2866, 307, -6553, -2866, -306);
        floors[21] = Surface(-7065, -2764, 307, -6553, -2866, -306, -7065, -2764, -306);
        // starting slope down
        floors[22] = Surface(-6553, -2866, 307, -7065, -3071, 322, -6041, -3071, 307);
        // steeple triangle 1
        floors[23] = Surface(-306, -2866, 307, 0, -2866, 922, 0, -2866, -306);
        floors[24] = Surface(-306, -2866, 307, 0, -2866, -306, -306, -2866, -306);
        // rectangle before wavy lava moving plat
        floors[25] = Surface(-6041, -2866, -306, -6553, -2866, 307, -6041, -2866, 307);
        floors[26] = Surface(-6041, -2866, -306, -6553, -2866, -306, -6553, -2866, 307);
        // lowest rectangle in beginning region
        floors[27] = Surface(5222, -2917, 573, 6298, -2917, 573, 6298, -2917, -40);
        // 27 and 30 are the triangles for our rectangle of interest
        floors[28] = Surface(5222, -2917, 573, 4301, -2764, -40, 4301, -2764, 573);
        floors[29] = Surface(5222, -2917, 573, 5222, -2917, -40, 4301, -2764, -40);
        // ramp leading to rectangle of interest
        floors[30] = Surface(5222, -2917, 573, 6298, -2917, -40, 5222, -2917, -40);
        // other rectangle of interest normal
        floors[31] = Surface(-921, -3020, 307, -921, -3020, 922, -306, -2866, 307);
        floors[32] = Surface(-921, -3020, 922, 0, -2866, 922, -306, -2866, 307);
        // ramp down to lava after track platform ride ends
        floors[33] = Surface(-3993, -3071, -613, -4146, -2661, -306, -3993, -3071, -306);
        floors[34] = Surface(-3993, -3071, -613, -4146, -2661, -613, -4146, -2661, -306);
        // right side of the 1-up platform
        floors[35] = Surface(-7065, -3071, 322, -6553, -2866, 307, -7065, -2866, 307);
        // the other weird steeple triangle
        floors[36] = Surface(-8191, -3071, 8192, 8192, -3071, -8191, -8191, -3071, -8191);
        floors[37] = Surface(-8191, -3071, 8192, 8192, -3071, 8192, 8192, -3071, -8191);
        // lava
    }


    void initialise_keyFloors() {
        // 0 is the starting slope down
        keyFloors[0][0][0] = (int)round(floors[21].min_x);
        keyFloors[0][0][1] = (int)round(floors[21].min_z);
        keyFloors[0][1][0] = (int)round(floors[21].min_x);
        keyFloors[0][1][1] = (int)round(floors[21].max_z);
        keyFloors[0][2][0] = (int)round(floors[21].max_x);
        keyFloors[0][2][1] = (int)round(floors[21].min_z);
        keyFloors[0][3][0] = (int)round(floors[21].max_x);
        keyFloors[0][3][1] = (int)round(floors[21].max_z);
        // 1 is the lowest rectangle in the starting region
        keyFloors[1][0][0] = (int)round(floors[26].min_x);
        keyFloors[1][0][1] = (int)round(floors[26].min_z);
        keyFloors[1][1][0] = (int)round(floors[26].min_x);
        keyFloors[1][1][1] = (int)round(floors[26].max_z);
        keyFloors[1][2][0] = (int)round(floors[26].max_x);
        keyFloors[1][2][1] = (int)round(floors[26].min_z);
        keyFloors[1][3][0] = (int)round(floors[26].max_x);
        keyFloors[1][3][1] = (int)round(floors[26].max_z);
        // 2 is the center rectangle. Kinda shitty choice bc hard to hit.
        keyFloors[2][0][0] = (int)round(floors[23].min_x);
        keyFloors[2][0][1] = (int)round(floors[23].min_z);
        keyFloors[2][1][0] = (int)round(floors[23].min_x);
        keyFloors[2][1][1] = (int)round(floors[23].max_z);
        keyFloors[2][2][0] = (int)round(floors[23].max_x);
        keyFloors[2][2][1] = (int)round(floors[23].min_z);
        keyFloors[2][3][0] = (int)round(floors[23].max_x);
        keyFloors[2][3][1] = (int)round(floors[23].max_z);
        // 3 is the ramp down to the pole
        keyFloors[3][0][0] = (int)round(floors[29].min_x);
        keyFloors[3][0][1] = (int)round(floors[29].min_z);
        keyFloors[3][1][0] = (int)round(floors[29].min_x);
        keyFloors[3][1][1] = (int)round(floors[29].max_z);
        keyFloors[3][2][0] = (int)round(floors[29].max_x);
        keyFloors[3][2][1] = (int)round(floors[29].min_z);
        keyFloors[3][3][0] = (int)round(floors[29].max_x);
        keyFloors[3][3][1] = (int)round(floors[29].max_z);
        // 4 is starting rectangle, kinda high
        keyFloors[4][0][0] = (int)round(floors[19].min_x);
        keyFloors[4][0][1] = (int)round(floors[19].min_z);
        keyFloors[4][1][0] = (int)round(floors[19].min_x);
        keyFloors[4][1][1] = (int)round(floors[19].max_z);
        keyFloors[4][2][0] = (int)round(floors[19].max_x);
        keyFloors[4][2][1] = (int)round(floors[19].min_z);
        keyFloors[4][3][0] = (int)round(floors[19].max_x);
        keyFloors[4][3][1] = (int)round(floors[19].max_z);
        // 5 is the rectangle before the octagon
        keyFloors[5][0][0] = (int)round(floors[15].min_x);
        keyFloors[5][0][1] = (int)round(floors[15].min_z);
        keyFloors[5][1][0] = (int)round(floors[15].min_x);
        keyFloors[5][1][1] = (int)round(floors[15].max_z);
        keyFloors[5][2][0] = (int)round(floors[15].max_x);
        keyFloors[5][2][1] = (int)round(floors[15].min_z);
        keyFloors[5][3][0] = (int)round(floors[15].max_x);
        keyFloors[5][3][1] = (int)round(floors[15].max_z);
        // 6 is the octagon, and the best one.
        keyFloors[6][0][0] = (int)round(floors[10].min_x);
        keyFloors[6][0][1] = (int)round(floors[8].min_z);
        keyFloors[6][1][0] = (int)round(floors[10].min_x);
        keyFloors[6][1][1] = (int)round(floors[13].max_z);
        keyFloors[6][2][0] = (int)round(floors[8].max_x);
        keyFloors[6][2][1] = (int)round(floors[8].min_z);
        keyFloors[6][3][0] = (int)round(floors[8].max_x);
        keyFloors[6][3][1] = (int)round(floors[13].max_z);

        keyCenter[0][0] = (int)round(0.5f * floors[21].min_x + 0.5f * floors[21].max_x);
        keyCenter[0][1] = (int)round(0.5f * floors[21].min_z + 0.5f * floors[21].max_z);
        keyCenter[1][0] = (int)round(0.5f * floors[26].min_x + 0.5f * floors[26].max_x);
        keyCenter[1][1] = (int)round(0.5f * floors[26].min_z + 0.5f * floors[26].max_z);
        keyCenter[2][0] = (int)round(0.5f * floors[23].min_x + 0.5f * floors[23].max_x);
        keyCenter[2][1] = (int)round(0.5f * floors[23].min_z + 0.5f * floors[23].max_z);
        keyCenter[3][0] = (int)round(0.5f * floors[29].min_x + 0.5f * floors[29].max_x);
        keyCenter[3][1] = (int)round(0.5f * floors[29].min_z + 0.5f * floors[29].max_z);
        keyCenter[4][0] = (int)round(0.5f * floors[19].min_x + 0.5f * floors[19].max_x);
        keyCenter[4][1] = (int)round(0.5f * floors[19].min_z + 0.5f * floors[19].max_z);
        keyCenter[5][0] = (int)round(0.5f * floors[15].min_x + 0.5f * floors[15].max_x);
        keyCenter[5][1] = (int)round(0.5f * floors[15].min_z + 0.5f * floors[15].max_z);
        keyCenter[6][0] = (int)round(0.5f * floors[10].min_x + 0.5f * floors[8].max_x);
        keyCenter[6][1] = (int)round(0.5f * floors[8].min_z + 0.5f * floors[13].max_z);

        thatNearOneConstant = floors[30].normal[1];
    }


    __host__ __device__ int find_floor(float* position, Surface** floor, float& floor_y, Surface floor_set[], int n_floor_set) {
        short x = (short)(int)position[0];
        short y = (short)(int)position[1];
        short z = (short)(int)position[2];

        int floor_idx = -1;

        for (int i = 0; i < n_floor_set; ++i) {
            if (x < floor_set[i].min_x || x > floor_set[i].max_x || z < floor_set[i].min_z || z > floor_set[i].max_z) {
                continue;
            }

            if ((floor_set[i].vertices[0][2] - z) * (floor_set[i].vertices[1][0] - floor_set[i].vertices[0][0]) - (floor_set[i].vertices[0][0] - x) * (floor_set[i].vertices[1][2] - floor_set[i].vertices[0][2]) < 0) {
                continue;
            }
            if ((floor_set[i].vertices[1][2] - z) * (floor_set[i].vertices[2][0] - floor_set[i].vertices[1][0]) - (floor_set[i].vertices[1][0] - x) * (floor_set[i].vertices[2][2] - floor_set[i].vertices[1][2]) < 0) {
                continue;
            }
            if ((floor_set[i].vertices[2][2] - z) * (floor_set[i].vertices[0][0] - floor_set[i].vertices[2][0]) - (floor_set[i].vertices[2][0] - x) * (floor_set[i].vertices[0][2] - floor_set[i].vertices[2][2]) < 0) {
                continue;
            }

            float height = -(x * floor_set[i].normal[0] + floor_set[i].normal[2] * z + floor_set[i].origin_offset) / floor_set[i].normal[1];

            if (y - (height + -78.0f) < 0.0f) {
                continue;
            }

            floor_y = height;
            *floor = &floor_set[i];
            floor_idx = i;
            break;
        }

        return floor_idx;
    }


    __host__ __device__ int assess_floor(float* position) {
        Surface* floorSet;
        #if !defined(__CUDA_ARCH__)
            floorSet = floors;
        #else
            floorSet = floorsG;
        #endif
        
        Surface* floor;
        float floorheight;
        int floorIdx = find_floor(position, &floor, floorheight, floorSet, total_floors);
        if (floorIdx == -1) {
            return 0;
        }
        else if (position[1] > floorheight + 100.0f) {
            return 1;
        }
        else if (position[1] <= floorheight + 0.1f && floor->normal[1] >= 0.99f) {
            return 2;
        }
        else if (position[1] <= floorheight + 0.1f && floor->normal[1] < 0.99f) {
            return 3;
        }
        else {
            return 4;
        }
    }


    __host__ __device__ bool stability_check(float* position, float speed, int angle) {

        // Choose the appropriate trig tables/floor set depending on whether the function is called from the host or from the device
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

        float nextPos[3];
        nextPos[0] = position[0];
        nextPos[1] = position[1];
        nextPos[2] = position[2];
        int correctedAngle = fix(angle);
        float velX = speed * sineTable[correctedAngle >> 4];
        float velZ = speed * cosineTable[correctedAngle >> 4];

        Surface* floor;
        float floorheight;
        int floorIdx = find_floor(position, &floor, floorheight, floorSet, total_floors);
        nextPos[0] += floor->normal[1] * (velX / 4.0f);
        nextPos[2] += floor->normal[1] * (velZ / 4.0f);

        return assess_floor(nextPos) == 0;
    }


    __host__ __device__ bool on_one_up(float* position) {

        Surface* floorSet;
        #if !defined(__CUDA_ARCH__)
            floorSet = floors;
        #else
            floorSet = floorsG;
        #endif

        if (assess_floor(position) == 3) {
            Surface* floor;
            float floorheight;
            int floorIdx = find_floor(position, &floor, floorheight, floorSet, total_floors);

            if (position[0] > 0.0f && (floorIdx == 2 || floorIdx == 3)) {
                return true;
            }
            else if (position[0] < 0.0f && (floorIdx == 33 || floorIdx == 34)) {
                return true;
            }
            else {
                return false;
            }
        }
        else {
            return false;
        }
    }

}
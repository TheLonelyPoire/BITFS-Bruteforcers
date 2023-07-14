#include "Floor.cuh"
#include "Trig.cuh"

namespace BITFS {

    __device__ Surface floorsG[total_floorsG];


    __global__ void initialise_floors() {
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


    __device__ int find_floor(float* position, Surface** floor, float& floor_y, Surface floor_set[], int n_floor_set) {
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


    __device__ int assess_floor(float* position) {
        Surface* floor;
        float floorheight;
        int floorIdx = find_floor(position, &floor, floorheight, floorsG, total_floorsG);
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


    __device__ bool stability_check(float* position, float speed, int angle) {
        float nextPos[3];
        nextPos[0] = position[0];
        nextPos[1] = position[1];
        nextPos[2] = position[2];
        int correctedAngle = fix(angle);
        float velX = speed * gSineTableG[correctedAngle >> 4];
        float velZ = speed * gCosineTableG[correctedAngle >> 4];

        Surface* floor;
        float floorheight;
        int floorIdx = find_floor(position, &floor, floorheight, floorsG, total_floorsG);
        nextPos[0] += floor->normal[1] * (velX / 4.0f);
        nextPos[2] += floor->normal[1] * (velZ / 4.0f);

        return assess_floor(nextPos) == 0;
    }
}
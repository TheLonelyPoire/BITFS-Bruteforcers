#pragma once

#include "cuda.h"
#include "math.h"
#include "cuda_runtime.h"

namespace BITFS {

    class Surface {
    public:
        short vertices[3][3];
        float normal[3];
        float origin_offset;
        float lower_y;
        float upper_y;

        float min_x;
        float max_x;
        float min_z;
        float max_z;


        __host__ __device__ Surface() {} // Needs to be defined here rather than in the source file for things to build, for some reason

        __host__ __device__ Surface(short x0, short y0, short z0, short x1, short y1, short z1, short x2, short y2, short z2);

        __host__ __device__ Surface(short verts[3][3]);

        __host__ __device__ void set_vertices(short verts[3][3]);

        __host__ __device__ void calculate_normal();
    };

}
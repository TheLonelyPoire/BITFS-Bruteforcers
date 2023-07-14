#include "Surface.cuh"
#include "math.h"

namespace BITFS {

    __host__ __device__ Surface::Surface(short x0, short y0, short z0, short x1, short y1, short z1, short x2, short y2, short z2) {
        short verts[3][3] = { {x0, y0, z0}, {x1, y1, z1}, {x2, y2, z2} };
        set_vertices(verts);
    }


    __host__ __device__ Surface::Surface(short verts[3][3]) {
        set_vertices(verts);
    }


    __host__ __device__ void Surface::set_vertices(short verts[3][3]) {
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                vertices[i][j] = verts[i][j];
            }
        }

        lower_y = fminf(fminf(vertices[0][1], vertices[1][1]), vertices[2][1]) - 5;
        upper_y = fmaxf(fmaxf(vertices[0][1], vertices[1][1]), vertices[2][1]) + 5;

        min_x = fminf(fminf(vertices[0][0], vertices[1][0]), vertices[2][0]);
        max_x = fmaxf(fmaxf(vertices[0][0], vertices[1][0]), vertices[2][0]);
        min_z = fminf(fminf(vertices[0][2], vertices[1][2]), vertices[2][2]);
        max_z = fmaxf(fmaxf(vertices[0][2], vertices[1][2]), vertices[2][2]);

        calculate_normal();
    }


    __host__ __device__ void Surface::calculate_normal() {
        normal[0] = (vertices[1][1] - vertices[0][1]) * (vertices[2][2] - vertices[1][2]) - (vertices[1][2] - vertices[0][2]) * (vertices[2][1] - vertices[1][1]);
        normal[1] = (vertices[1][2] - vertices[0][2]) * (vertices[2][0] - vertices[1][0]) - (vertices[1][0] - vertices[0][0]) * (vertices[2][2] - vertices[1][2]);
        normal[2] = (vertices[1][0] - vertices[0][0]) * (vertices[2][1] - vertices[1][1]) - (vertices[1][1] - vertices[0][1]) * (vertices[2][0] - vertices[1][0]);

        float mag = sqrtf(normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2]);

        mag = (float)(1.0 / mag);
        normal[0] *= mag;
        normal[1] *= mag;
        normal[2] *= mag;

        origin_offset = -(normal[0] * vertices[0][0] + normal[1] * vertices[0][1] + normal[2] * vertices[0][2]);
    }

}
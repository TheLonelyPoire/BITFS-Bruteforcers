#include <fstream>
#include <cstring>
#include <string>
#include <unordered_map>
#include <iostream>
#include <filesystem>
#include <cmath>
#include <chrono>
#include "cuda.h"
#include "cuda_runtime.h"
#include "device_launch_parameters.h"
#include "device_atomic_functions.h"

#include "../Common/CommonBruteforcerStructs.hpp"
#include "../Common/Camera.cuh"
#include "../Common/Floors.cuh"
#include "../Common/Movement.cuh"
#include "../Common/Stick.cuh"
#include "../Common/Surface.cuh"
#include "../Common/Trig.cuh"
#include "../Common/VMath.cuh"

#include "ISTStructs.hpp"

using namespace BITFS;
namespace fs = std::filesystem;

#define MAX_DEPARTURES 1000000
#define MAX_ARRIVALS 1000000
#define MAX_LANDINGS 1000000


__device__ PhaseOneInfo* departureLog;
__device__ int nDepartures;
__device__ PhaseTwoInfo* arrivalLog;
__device__ int nArrivals;
__device__ PhaseThreeInfo* landingLog;
__device__ int nLandings;

__device__ int testI = -1;


// Returns the timestamp of the argument as a string with the format MONTH_DAY_HOUR_MINUTE.
std::string get_timestamp(std::chrono::system_clock::time_point tp)
{
    std::time_t time_t = std::chrono::system_clock::to_time_t(tp);
    std::tm* tm = std::localtime(&time_t);

    return std::to_string(tm->tm_year + 1900) + "_" + std::to_string(tm->tm_mon + 1) + "_" + std::to_string(tm->tm_mday) + "_" + std::to_string(tm->tm_hour) + "_" + std::to_string(tm->tm_min);
}


__global__ void copy_pointers_to_gpu(PhaseOneInfo* p1, PhaseTwoInfo* p2, PhaseThreeInfo* p3) {
    departureLog = p1;
    arrivalLog = p2;
    landingLog = p3;
}


void craft_matrix(float* matrix, float* norm, float* origin) {
    
    float lateralDir[3];
    float leftDir[3];
    float forwardDir[3];
    float upDir[3];
    
    lateralDir[0] = 0.0f;
    lateralDir[1] = 0.0f;
    lateralDir[2] = 1.0f;
    upDir[0] = norm[0];
    upDir[1] = norm[1];
    upDir[2] = norm[2];

    vec3_normalize(upDir);
    vec3_cross(leftDir, upDir, lateralDir);
    vec3_normalize(leftDir);
    vec3_cross(forwardDir, leftDir, upDir);
    vec3_normalize(forwardDir);

    matrix[0] = leftDir[0];
	matrix[1] = leftDir[1];
	matrix[2] = leftDir[2];
    matrix[3] = 0.0f;

    matrix[4] = upDir[0];
    matrix[5] = upDir[1];
    matrix[6] = upDir[2];
    matrix[7] = 0.0f;

    matrix[8] = forwardDir[0];
    matrix[9] = forwardDir[1];
    matrix[10] = forwardDir[2];
    matrix[11] = 0.0f;

    matrix[12] = origin[0];
    matrix[13] = origin[1];
    matrix[14] = origin[2];
    matrix[15] = 1.0f;
}


    
void fill_coords(short* coords, float* norm, float* origin, short* defaulttri) {
    
    float m[16];
    craft_matrix(m, norm, origin);
    for (int i = 0; i < 3; i++) {
        
        float vx = defaulttri[3 * i + 0];
        float vy = defaulttri[3 * i + 1];
        float vz = defaulttri[3 * i + 2];

        coords[3 * i + 0] = (short)(int)(vx * m[0] + vy * m[4] + vz * m[8] + m[12]);
        coords[3 * i + 1] = (short)(int)(vx * m[1] + vy * m[5] + vz * m[9] + m[13]);
        coords[3 * i + 2] = (short)(int)(vx * m[2] + vy * m[6] + vz * m[10] + m[14]);
    }
}


    
void craft_triangle(Surface* floor, short* coords) {
    *floor = Surface(coords[0], coords[1], coords[2], coords[3], coords[4], coords[5], coords[6], coords[7], coords[8]);
}


    
float approach_by_increment(float goal, float src, float inc) {
	float newVal;

	if (src <= goal) {
		if (goal - src < inc) {
			newVal = goal;
		}
		else {
			newVal = src + inc;
		}
	}
	else if (goal - src > -inc) {
		newVal = goal;
	}
	else {
		newVal = src - inc;
	}

	return newVal;
}


    
void platform_update(float* norm) {
    norm[0] = approach_by_increment(0.0f, norm[0], 0.01f);
    norm[1] = approach_by_increment(1.0f, norm[1], 0.01f);
    norm[2] = approach_by_increment(0.0f, norm[2], 0.01f);
    
}


    
// fills the height with the height we'd snap up to if we were at a suitable height to interact with the floor triangle
// and lava otherwise. Returns true if our floor is the floor triangle of interest, and false otherwise.
__device__ bool on_surface(Surface tri, float* position, float* height) {

    short x = (short)(int)position[0];
    short y = (short)(int)position[1];
    short z = (short)(int)position[2];
    *height = -3071.0f;

    if (x < tri.min_x || x > tri.max_x || z < tri.min_z || z > tri.max_z) {
        return false;
    }
    if ((tri.vertices[0][2] - z) * (tri.vertices[1][0] - tri.vertices[0][0]) - (tri.vertices[0][0] - x) * (tri.vertices[1][2] - tri.vertices[0][2]) < 0) {
        return false;
    }
    if ((tri.vertices[1][2] - z) * (tri.vertices[2][0] - tri.vertices[1][0]) - (tri.vertices[1][0] - x) * (tri.vertices[2][2] - tri.vertices[1][2]) < 0) {
        return false;
    }
    if ((tri.vertices[2][2] - z) * (tri.vertices[0][0] - tri.vertices[2][0]) - (tri.vertices[2][0] - x) * (tri.vertices[0][2] - tri.vertices[2][2]) < 0) {
        return false;
    }
    
    float elevation = -(x * tri.normal[0] + tri.normal[2] * z + tri.origin_offset) / tri.normal[1];
    
    if (y - (elevation + -78.0f) < 0.0f) {
        return false;
    }

    *height = elevation;
    return true;
}


    
// given an angle in HAU's, and a bully position, this is where Mario gets pushed to.
__device__ void pushed_to(float* position, BullyData bully, int hau) {
    position[0] = bully.posBully[0] + 113.0f * gSineTableG[hau];
    position[1] = bully.posBully[1];
    position[2] = bully.posBully[2] + 113.0f * gCosineTableG[hau];
}


    
// this checks whether there is indeed a place that Mario can be to get pushed to his indicated position.
__device__ bool angle_check(float* position, BullyData bully, Surface floorone, Surface floortwo) {
    
    // we want to test whether we're under the floor triangles or not. We'll spuriously get declared as "not on the floor triangle"
    // if we're too low, so we boost up the height a little bit.
    // also, midspot is the furthest spot at which we can interact with the bully, 63 units away.
    // aka, 63/113 of the way from the starting spot to the position we get pushed to.
    float startspot[3];
    float midspot[3];
    float p = 63.0f / 113.0f;
    float height;
    startspot[0] = bully.posBully[0];
    startspot[1] = bully.posBully[1] + 78.0f;
    startspot[2] = bully.posBully[2];
    midspot[0] = p * position[0] + (1.0f - p) * startspot[0];
    midspot[1] = startspot[1];
    midspot[2] = p * position[2] + (1.0f - p) * startspot[2];

    // the conditional logic is pretty complex. Basically, we have a line from the bully to midspot that is "where mario can get
    // pushed by the bully and end up at position", and somewhere in that line we want to know whether there's a point that's not 
    // under floorone (so mario can exist there) but under floortwo (so mario can get squished there).
    // since the platform is untilting, being under floortwo is easier than being under floorone.
    // case 1: The center of the bully is under floorone. If midspot is also under floorone, then the line between them is
    // under floorone, so nowhere works, return false. If it isn't under floorone, then there's some point on the line where things
    // go from "not under floorone" to "under floorone", and such a point is under floortwo. Success.
    // case 2: The center of the bully isn't under floortwo. If midspot is also not under floortwo, then the line between them
    // is never under floortwo anywhere, nowhere works, return false. If it is, then there's a point between them where things go from
    // "not under floortwo" to "under floortwo", and such a point isn't under floorone. Success.
    // case 3: the bully is under floor two but not under floor one. The immediate vicinity of the bully is the spot we need for success.
    if (on_surface(floorone, startspot, &height)) {
        return !on_surface(floorone, midspot, &height);
    }
    else if (!on_surface(floortwo, startspot, &height)) {
        return on_surface(floortwo, midspot, &height);
    }
    else {
        return true;
    }
}


    
// simulates Mario getting squishpushed. Returns true if he moves off the pyramid, at over 100 height, and the pyramid tilts
// back under him. Updates the position as it goes.
__device__ bool squish_simulator(Surface floortwo, Surface floorthree, Surface ceil, float* position) {

    // return false if you don't get squished.
    if (-0.5f >= ceil.normal[1]) {
        return false;
    }

    // work out your squish push. Defacto multiplier is 1 for the first QF of squish push, btw.
    int surfangle = atan2s(ceil.normal[2], ceil.normal[0]);
    float mvelx = sm64_sins(surfangle) * 10.0f;
    float mvelz = sm64_coss(surfangle) * 10.0f;
    float defacto = 1.0f;
    float height;

    
    // during a quarterframe, you update your position. You may or may not be on the pyramid platform.
    // if you're on it, update your height. If you're not, and it's the first QF or your position is too low, you lose bc lava snap.
    // if you're not, and it's a later QF and your position is high enough, you win if the platform tilts back under you.
    // if you go through all QF's and never leave the platform, you lose.
    for (int i = 0; i < 4; i++) {
        position[0] += defacto * (mvelx / 4.0f);
        position[2] += defacto * (mvelz / 4.0f);

        if (on_surface(floortwo, position, &height)) {
            position[1] = height;
            defacto = floortwo.normal[1];
        }
        else if (i == 0 || position[1] <= -3071.0f + 100.0f) {
            return false;
        }
        else {
            return on_surface(floorthree, position, &height);
        }
    }
    return false;
}




// this, given a defacto speed, and mario's height, computes the lower and upper bounds on Mario's speed which hit the 1-up platform.
__device__ void compute_vel_bounds(float defacto, float* start, int* bounds) {

    // leftbound and rightbound are, given mario's height, the leftmost and rightmost he can land on the 1-up.
    // this is done via algebra shenanigans I don't feel like explaining.
    // from this, the bounds follow by rearranging the equation
    // start[0] + ((v * defacto / 4) + ((v-1) * 1 / 4)) * gSineTableG[1021] = 2 * 65536 + bounds.
    float leftp = (start[1] - (-2661.0f)) / ((-3071.0f) - (-2661.0f));
    float rightp = (start[1] + 78.0f - (-2661.0f)) / ((-3071.0f) - (-2661.0f));
    float leftbound = leftp * (-4607.0f) + (1.0f - leftp) * (-4453.0f);
    float rightbound = rightp * (-4607.0f) + (1.0f - rightp) * (-4453.0f);
    bounds[0] = floorf( (4.0f / (defacto + 1.0f) ) * ( ( (-start[0] + 2.0f * 65536.0f + leftbound) / gSineTableG[1021] ) + 0.25f) );
    bounds[1] = ceilf( (4.0f / (defacto + 1.0f) ) * ( ( (-start[0] + 2.0f * 65536.0f + rightbound) / gSineTableG[1021] ) + 0.25f) );
}




__global__ void first_pass(BullyData bully, Surface floorone, Surface floortwo, Surface floorthree, Surface ceiltwo) {

    // from the thread id, get the angle in HAU's.
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= 4096) {
        return;
    }

    // figure out where mario gets pushed to.
    float pos[3];
    pushed_to(pos, bully, idx);

    // quit if the "is there anywhere mario can be to both get squished and get pushed to the position" check comes back false.
    if (!angle_check(pos, bully, floorone, floortwo)) {
        return;
    }

    // quit if the "does the squish push send mario off the platform" test comes back false. It updates mario's position
    // while doing the simulation, btw.
    if (!squish_simulator(floortwo, floorthree, ceiltwo, pos)) {
        return;
    }

    // work out the velocity bounds on whether you can arrive at the one-up.
    float defacto = floorthree.normal[1];
    int bounds[2];
    compute_vel_bounds(defacto, pos, bounds);
    if (bounds[0] > bounds[1]) {
        return;
    }
    // this clause shouldn't ever fire but just in case, junk stuff where your height is too low.
    if (pos[1] <= -3071.0f + 100.0f) {
        return;
    }

    for (int v = bounds[0]; v <= bounds[1]; v++) {

        int solIdx = atomicAdd(&nDepartures, 1);
        if (solIdx > MAX_DEPARTURES) {
            return;
        }
        struct PhaseOneInfo* data = &(departureLog[solIdx]);
        data->startpos[0] = pos[0];
        data->startpos[1] = pos[1];
        data->startpos[2] = pos[2];
        data->vel = v;
        data->hau = idx;
    }
}




__global__ void air_simulate(int nDeparturesCPU, float defacto, BullyData cam) {

    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= nDeparturesCPU) {
        return;
    }

    // fetch the data
    struct PhaseOneInfo* info = &(departureLog[idx]);

    // find the frame one position.
    float frameonepos[3];
    float vel = (float)info->vel;
    frameonepos[0] = info->startpos[0] + defacto * ((vel * gSineTableG[1021]) / 4.0f);
    frameonepos[1] = info->startpos[1];
    frameonepos[2] = info->startpos[2] + defacto * ((vel * gCosineTableG[1021]) / 4.0f);

    // a basic sanity check on the frame one position is that, if we find the floor associated with it, the height of the floor should
    // be over 100 units below us, otherwise we'd snap to the floor. And the position shouldn't be out-of-bounds.
    Surface* floor;
    float floorheight;
    int floorIdx = find_floor(frameonepos, &floor, floorheight, floorsG, total_floors);
    if (floorIdx == -1) {
        return;
    }
    if(floorheight > frameonepos[1] - 100.0f) {
        return;
    }

    // and where we arrive at.
    struct AirInfo frametwoair;
    sim_airstep(frameonepos, vel, 1021 * 16, true, frametwoair);

    // use a lack of height-gain as a proxy for "whoops shit we didn't hit the one-up platform"
    if (frametwoair.endPos[1] < frameonepos[1] + 0.01f) {
        return;
    }

    float lakitu[3];
    lakitu[0] = cam.posBully[0];
    lakitu[1] = cam.posBully[1];
    lakitu[2] = cam.posBully[2];
    float focus[3];
    float pan;
    float campos[3];

    // ok, at this point, we probably hit the 1-up platform. Time to compute the camera yaw. 
    // A critical note here is that it seems that facing angle is NOT fully pinned down by the HAU you're traveling at. 
    // I know that the HAU is 1021 so I'll arbitrarily assume that the facing angle is 1021 * 16 + 8 (precisely halfway
    // through the band of AU's that correspond to HAU 1021). And the 49152 corresponds to Mario facing perfectly 
    // in the -X direction when he runs off the edge of the pyramid, which appears to be a roughly acceptable assumption to make.
    int bwee = fine_camera_yaw(info->startpos, lakitu, 49152, focus, &pan, campos, false);
    int camyaw = tenk_camera_yaw(info->startpos, frameonepos, lakitu, 49152, (1021 * 16) + 8, focus, &pan, campos);

    int solIdx = atomicAdd(&nArrivals, 1);
    if (solIdx > MAX_ARRIVALS) {
        return;
    }
    struct PhaseTwoInfo* data = &(arrivalLog[solIdx]);
    data->f1pos[0] = frameonepos[0];
    data->f1pos[1] = frameonepos[1];
    data->f1pos[2] = frameonepos[2];
    data->tenkpos[0] = frametwoair.endPos[0];
    data->tenkpos[1] = frametwoair.endPos[1];
    data->tenkpos[2] = frametwoair.endPos[2];
    data->tenkvelin = frametwoair.endSpeed;
    data->camyaw = camyaw;
    data->id = idx;
}




__global__ void tenk_simulate(int nArrivalsCPU, int minx, int maxx, int miny, int maxy, float speedthresh) {

    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= nArrivalsCPU * NUM_STICK_TABLE_ENTRIES_BACKWARDS) { // 10128 is the number of stick table entries when the backwards parameter is true
        return;
    }

    int tag = idx % nArrivalsCPU;
    int i = (idx - tag) / nArrivalsCPU;


    // automatically throw out things if the stick position isn't an acceptable one.
    if (stickTabG[i].stickX < minx || stickTabG[i].stickX > maxx || stickTabG[i].stickY < miny || stickTabG[i].stickY > maxy) {
        return;  
    }

    // then fetch data.
    struct PhaseTwoInfo* p2_data = &(arrivalLog[tag]);

    // for simulating the 10k, we have the problem that facing angle and slide yaw can be slightly different though the HAU
    // is the same, and we only know the HAU of arrival. So we assume 1021 * 16 + 8, because that's halfway through the band of
    // AU's that correspond to HAU 1021.
    struct FancySlideInfo tenk;
    if(!sim_slide(stickTabG[i], p2_data->tenkpos, p2_data->tenkvelin, p2_data->tenkvelin * gSineTableG[1021], p2_data->tenkvelin * gCosineTableG[1021], 1021 * 16 + 8, 1021 * 16 + 8, p2_data->camyaw, false, tenk)) {
        return;   
    };
    
    // Junk it if the solution doesn't get sufficient negative speed.
    if (tenk.endSpeed >= speedthresh){
        return;
    }

    // and if it doesn't end up in the air.
    if (assess_floor(tenk.endPos) != 1) {
        return;
    }

    // simulate the air movement then, and make sure it ends up on the ground. Repeatedly simulate the airsteps
    // and if we hit OOB, fail, and if we land on ground, succeed.
    AirInfo tenkair;
    for (int qf = 1; qf <= 12; qf++) {
        if (!sim_airstep(((qf == 1) ? tenk.endPos : tenkair.endPos), ((qf == 1) ? tenk.endSpeed : tenkair.endSpeed), tenk.endFacingAngle, (qf % 4 == 1), tenkair)) {
            return;
        }
        if(assess_floor(tenkair.endPos) == 0) {
            return;
        }
        else if(assess_floor(tenkair.endPos) == 2 || assess_floor(tenkair.endPos) == 3) {
            break;
        }
    }

    // see whether we landed on suitable ground.
    Surface* floor;
    float floorheight;
    int floorIdx = find_floor(tenkair.endPos, &floor, floorheight, floorsG, total_floors);
    if (floorIdx < 27 || floorIdx > 32) {
        return;
    }
    
    // and that we're stably walking against OOB.
    if (!stability_check(tenkair.endPos, tenkair.endSpeed, tenk.endFacingAngle)) {
        return; 
    };

    // and lo, a solution hath been found.
    int solIdx = atomicAdd(&nLandings, 1);
    if (solIdx > MAX_LANDINGS) {
        return;
    }
    struct PhaseThreeInfo* p3_data = &(landingLog[solIdx]);
    p3_data->landpos[0] = tenkair.endPos[0];
    p3_data->landpos[1] = tenkair.endPos[1];
    p3_data->landpos[2] = tenkair.endPos[2];
    p3_data->pux = (int)(round(tenkair.endPos[0] / 65536.0f));
    p3_data->puz = (int)(round(tenkair.endPos[2] / 65536.0f));
    p3_data->landvel = tenkair.endSpeed;
    p3_data->platid = floorIdx;
    p3_data->stickX = correct_stick(stickTabG[i].stickX);
    p3_data->stickY = correct_stick(stickTabG[i].stickY);
    p3_data->id = tag;
}




main(int argc, char* argv[]) {

    // initialize base parameters. minx, miny, and minz are the lowest xyz's we're looking at.
    // and then we count up by num in all three coordinates, at a certain float granularity.
    // we know we're on Mythra, and we know we're dealing with the upper-right quadrant where the normal is
    // +X and -Z. So minx and minz should be positive and negative, respectively.
    float minx = 0.17f;
    float miny = 0.8f;
    float minz = -0.49f;

    float grany = 0.001f;
    float granx = 0.00025f;
    float granz = 0.00025f;

    int numx = 200;
    int numy = 50;
    int numz = 200;

    // initialize the stick ranges we're dealing with, and platform slope ranges we're interested in, and speed threshold to beat.
    // to be thorough (and because I don't know the exact limits) we'll do a maximally lax scan the first time.
    int minstickx = -35;
    int maxstickx = 0;
    int minsticky = -128;
    int maxsticky = -20;
    float mindefacto = 0.8725;
    float maxdefacto = 0.8765;
    float speedthresh = -7.0e+06;

    // initialize threads and the bully state and cheat by storing camera data in another bully state.
    int nThreads = 256;
    struct BullyData bully;
    struct BullyData cam;
    bully.posBully[0] = -3120.0f;
    bully.posBully[1] = -2994.0f;
    bully.posBully[2] = -896.0f;
    cam.posBully[0] = -1700.0f;
    cam.posBully[1] = -2300.0f;
    cam.posBully[2] = 500.0f;


    auto startTime = std::chrono::system_clock::now();
    std::string timestamp = get_timestamp(startTime);

    std::string outRunInfoFile = "runInformation.txt";
    std::string outFullSolutionsFile = "ISTResults.csv";
    std::string outFileNormalStages = "normalStagesReached.bin";

    bool verbose = false;

    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "-h") || !strcmp(argv[i], "--help")) {
            printf("BitFS IST Brute Forcer.\n");
            printf("This program accepts the following options:\n\n");
            printf("-gran <gx> <gy> <gz>: The granularity of samples in the x, y, and z directions.\n");
            printf("             Default: %f %f %f\n", granx, grany, granz);
            printf("-num <num_x> <num_y> <num_z>: The number of samples in the x, y, and z directions starting from the min coordinates.\n");
            printf("             Default: %d %d %d\n", numx, numy, numz);
            printf("-min <min_x> <min_y> <min_z>: The minimum x, y, and z coordinates for the platform normal.\n");
            printf("             Default: %f %f %f\n", minx, miny, minz);
            printf("-de <min_nY> <max_nY>: Bounds on defacto speed multiplier.\n");
            printf("             Default: %f %f\n", mindefacto, maxdefacto);
            printf("-sti <min_x> <max_x> <min_y> <max_y>: Stick position bounds.\n");
            printf("             Default: %d %d %d %d\n", minstickx, maxstickx, minsticky, maxsticky);
            printf("-cp <pos_x> <pos_y> <pos_z>: Position of the camera.\n");
            printf("             Default: %f %f %f\n", cam.posBully[0], cam.posBully[1], cam.posBully[2]);
            printf("-bp <pos_x> <pos_y> <pos_z>: Bully's starting position.\n");
            printf("             Default: %f %f %f\n", bully.posBully[0], bully.posBully[1], bully.posBully[2]);
            printf("-o: Path to the output file.\n");
            printf("    Default: %s\n", outFullSolutionsFile.c_str());
            printf("-v: Verbose mode. Prints all parameters used in brute force.\n");
            printf("    Default: off\n");
            printf("-h --help: Prints this text.\n");
            exit(0);
        }
        else if (!strcmp(argv[i], "-gran")) {
            granx = std::stof(argv[i + 1]);
            grany = std::stof(argv[i + 2]);
            granz = std::stof(argv[i + 3]);

            i += 3;
        }
        else if (!strcmp(argv[i], "-num")) {
            numx = std::stoi(argv[i + 1]);
            numy = std::stoi(argv[i + 2]);
            numz = std::stoi(argv[i + 3]);

            i += 3;
        }
        else if (!strcmp(argv[i], "-min")) {
            minx = std::stof(argv[i + 1]);
            miny = std::stof(argv[i + 2]);
            minz = std::stof(argv[i + 3]);

            i += 3;
        }
        else if (!strcmp(argv[i], "-de")) {
            mindefacto = std::stof(argv[i + 1]);
            maxdefacto = std::stof(argv[i + 2]);

            i += 2;
        }
        else if (!strcmp(argv[i], "-sti")) {
            minstickx = std::stoi(argv[i + 1]);
            maxstickx = std::stoi(argv[i + 2]);
            minsticky = std::stoi(argv[i + 3]);
            maxsticky = std::stoi(argv[i + 3]);

            i += 4;
        }
        else if (!strcmp(argv[i], "-cp")) {
            cam.posBully[0] = std::stof(argv[i + 1]);
            cam.posBully[1] = std::stof(argv[i + 2]);
            cam.posBully[2] = std::stof(argv[i + 3]);

            i += 3;
        }
        else if (!strcmp(argv[i], "-bp")) {
            bully.posBully[0] = std::stof(argv[i + 1]);
            bully.posBully[1] = std::stof(argv[i + 2]);
            bully.posBully[2] = std::stof(argv[i + 3]);

            i += 3;
        }
        else if (!strcmp(argv[i], "-o")) {
            outFullSolutionsFile = argv[i + 1];
            i += 1;
        }
        else if (!strcmp(argv[i], "-v")) {
            verbose = true;
        }
        if (verbose) {
            printf("Normal Granularity: (%f, %f, %f)\n", granx, grany, granz);
            printf("Number of Samples: (%d, %d, %d)\n", numx, numy, numz);
            printf("Origin of Samples: (%f, %f, %f)\n", minx, miny, minz);
            printf("Defacto Multiplier Bounds: (%f, %f)\n", mindefacto, maxdefacto);
            printf("Stick Bounds: (%d, %d) X, (%d, %d) Y\n", minstickx, maxstickx, minsticky, maxsticky);
            printf("Camera Position: (%f, %f, %f)\n", cam.posBully[0], cam.posBully[1], cam.posBully[2]);
            printf("Bully Position: (%f, %f, %f)\n", bully.posBully[0], bully.posBully[1], bully.posBully[2]);
        }
    }

    // Attempt to create the directory
    std::string dir_name = "output/" + timestamp + "/";
    try {
        std::filesystem::create_directory(dir_name);
        std::cout << "Run directory \'" << dir_name << "\' created successfully." << std::endl;
    }
    catch (const std::filesystem::filesystem_error& e) {
        std::cerr << "Failed to create directory: " << e.what() << std::endl;
    }

    // Run information filestream
    std::ofstream wfRunInformation(dir_name + outRunInfoFile);
    wfRunInformation << "Norm Min Y: " << miny << "\n";
    wfRunInformation << "Norm Min X: " << minx << "\n";
    wfRunInformation << "Norm Min Z: " << minz << "\n";

    wfRunInformation << "Norm Num Y: " << numy << "\n";
    wfRunInformation << "Norm Num X: " << numx << "\n";
    wfRunInformation << "Norm Num Z: " << numz << "\n";

    wfRunInformation << "Granularity Y: " << grany << "\n";
    wfRunInformation << "Granularity X: " << granx << "\n";
    wfRunInformation << "Granularity Z: " << granz << "\n";

    wfRunInformation << "Defacto Multiplier Bounds: " << mindefacto << "," << maxdefacto << "\n";

    wfRunInformation << "Stick Bounds X: " << minstickx << "," << maxstickx << "\n";
    wfRunInformation << "Stick Bounds Y: " << minsticky << "," << maxsticky << "\n";

    wfRunInformation << "Camera Position: " << cam.posBully[0] << "," << cam.posBully[1] << "," << cam.posBully[2] << "\n";
    wfRunInformation << "Bully Position: " << bully.posBully[0] << "," << bully.posBully[1] << "," << bully.posBully[2] << "\n";

    wfRunInformation.close();


    // Full solution normal CSV filstream
    std::ofstream wfSolutionsCSV(dir_name + outFullSolutionsFile);
    wfSolutionsCSV << std::fixed;
    wfSolutionsCSV << "X, Y, Z" << std::endl;

    // Normal Stages binary output filestream
    std::ofstream wfNormalStages(dir_name + outFileNormalStages, std::ios::out | std::ios::binary);

    // standard GPU setup
    struct PhaseOneInfo* departuresGPU;
    cudaMalloc((void**)&departuresGPU, MAX_DEPARTURES * sizeof(struct PhaseOneInfo));
    struct PhaseTwoInfo* arrivalsGPU;
    cudaMalloc((void**)&arrivalsGPU, MAX_ARRIVALS * sizeof(struct PhaseTwoInfo));
    struct PhaseThreeInfo* landingsGPU;
    cudaMalloc((void**)&landingsGPU, MAX_LANDINGS * sizeof(struct PhaseThreeInfo));


    // we only have one floor triangle and one ceiling triangle to worry about, so initialize them. origin is the origin of Mythra.
    short floortri[9] = { 307, 307, -306, -306, 307, -306, -306, 307, 307 };
    short ceiltri[9] = {-306, 307, 307, -306, 307, -306, 0, 0, 0};
    float origin[3] = {-2866.0f, -3225.0f, -715.0f};

    // initialize floors and stick tables and GPU pointers.
    copy_pointers_to_gpu << <1, 1 >> > (departuresGPU, arrivalsGPU, landingsGPU);
    initialise_floorsG << <1, 1 >> > ();
    init_stick_tablesG << <1, 1 >> >(true);

    char* normalStages;
    normalStages = (char*)std::calloc(sizeof(char), numy * numx * numz);

    // begin iterating!
    for (int ny = 0; ny < numy; ny++) {
        std::cout << "ny = " << ny << "\n";
        for (int nx = 0; nx < numx; nx++) {
            for (int nz = 0; nz < numz; nz++) {
                
                int norm_index = nz + (nx + (ny * numx)) * numz;

                // initialize the platform normal right before the squish cancel frame
                float norm[3];
                norm[0] = minx + nx * granx;
                norm[1] = miny + ny * grany;
                norm[2] = minz + nz * granz;

                // automatically throw it out if we're in the wrong quadrant
                if (norm[0] < 0.0f || norm[2] > 0.0f) {
                    continue;
                }
                
                // firstnorm will be right before the SC frame, secondnorm will be the SC frame, thirdnorm will be the departure frame.
                // so we have norm be our first normal, update it one frame to make our second normal, update it one frame to make
                // our third normal.
                float firstnorm[3];
                float secondnorm[3];
                float thirdnorm[3];
                vec3_copy(firstnorm, norm);
                platform_update(norm);
                vec3_copy(secondnorm, norm);
                platform_update(norm);
                vec3_copy(thirdnorm, norm);

                // we have the base coordinates of our triangles of interest, but they must be adjusted via our
                // platform normal vector, so we know the floor triangles on the three frames and ceiling triangle on the SC frame.
                // so we fill up the arrays of shorts with the coordinates of the relevant triangles at the relevant normals.
                short firstfloor[9];
                short secondfloor[9];
                short thirdfloor[9];
                short secondceil[9];
                fill_coords(firstfloor, firstnorm, origin, floortri);
                fill_coords(secondfloor, secondnorm, origin, floortri);
                fill_coords(thirdfloor, thirdnorm, origin, floortri);
                fill_coords(secondceil, secondnorm, origin, ceiltri);

                // and use them to make surfaces, that we can use to find things like heights and slopes.
                Surface floorone;
                Surface floortwo;
                Surface floorthree;
                Surface ceiltwo;
                craft_triangle(&floorone, firstfloor);
                craft_triangle(&floortwo, secondfloor);
                craft_triangle(&floorthree, thirdfloor);
                craft_triangle(&ceiltwo, secondceil);

                // automatically throw things out if their platform slope is wrong, or if they don't produce a squish cancel.
                if (floorthree.normal[1] > maxdefacto || floorthree.normal[1] < mindefacto || -0.5f >= ceiltwo.normal[1]) {
                    continue;
                }

                // printf("(%f,%f,%f)\n", norm[0], norm[1], norm[2]);
                normalStages[norm_index] = 1;

                // sweet, time to start bruteforcing!
                int nFirstBlocks = (4096 + nThreads - 1) / nThreads;
                int nDeparturesCPU = 0;
                cudaMemcpyToSymbol (nDepartures, &nDeparturesCPU, sizeof(int), 0, cudaMemcpyHostToDevice);
                first_pass << <nFirstBlocks, nThreads >> > (bully, floorone, floortwo, floorthree, ceiltwo);
                cudaMemcpyFromSymbol (&nDeparturesCPU, nDepartures, sizeof(int), 0, cudaMemcpyDeviceToHost);
                // throw usual errors if too many or too few solutions.
                if (nDeparturesCPU > MAX_DEPARTURES) {
                    fprintf(stderr, "Warning: The number of departures has been exceeded. No more will be recorded. Increase the internal maximum to prevent this from happening.\n");
                    nDeparturesCPU = MAX_DEPARTURES;
                }
                if (nDeparturesCPU == 0) {
                    continue;
                }

                normalStages[norm_index] = 2;

                // printf("1");

                // and now proceed further, to simulate the departures.
                int nSecondBlocks = (nDeparturesCPU + nThreads - 1) / nThreads;
                int nArrivalsCPU = 0;
                cudaMemcpyToSymbol (nArrivals, &nArrivalsCPU, sizeof(int), 0, cudaMemcpyHostToDevice);
                air_simulate<< <nSecondBlocks, nThreads >> > (nDeparturesCPU, floorthree.normal[1], cam);
                cudaMemcpyFromSymbol (&nArrivalsCPU, nArrivals, sizeof(int), 0, cudaMemcpyDeviceToHost);
                if (nArrivalsCPU > MAX_ARRIVALS) {
                    fprintf(stderr, "Warning: The number of arrivals has been exceeded. No more will be recorded. Increase the internal maximum to prevent this from happening.\n");
                    nDeparturesCPU = MAX_ARRIVALS;
                }
                if (nArrivalsCPU == 0) {
                    continue;
                }

                printf("2: ");
                printf("(%f,%f,%f)\n", norm[0], norm[1], norm[2]);
                normalStages[norm_index] = 3;
            
                // and proceed further to simulate the landings.
                int nThirdBlocks = (nArrivalsCPU * NUM_STICK_TABLE_ENTRIES_BACKWARDS + nThreads - 1) / nThreads;
                int nLandingsCPU = 0;
                cudaMemcpyToSymbol (nLandings, &nLandingsCPU, sizeof(int), 0, cudaMemcpyHostToDevice);
                tenk_simulate<< <nThirdBlocks, nThreads >> > (nArrivalsCPU, minstickx, maxstickx, minsticky, maxsticky, speedthresh);
                cudaMemcpyFromSymbol (&nLandingsCPU, nLandings, sizeof(int), 0, cudaMemcpyDeviceToHost);
                if (nLandingsCPU > MAX_LANDINGS) {
                    fprintf(stderr, "Warning: The number of landings has been exceeded. No more will be recorded. Increase the internal maximum to prevent this from happening.\n");
                    nLandingsCPU = MAX_LANDINGS;
                }
                if (nLandingsCPU == 0) {
                    continue;
                }

                printf("3 it's a hit!\n");
                normalStages[norm_index] = 4;

                // There's room to actually extract the data and write it in a CSV. The problem is that I think there will
                // be much data per xyz that actually gets a solution because a lot more stuff gets to the final stage
                // than gets to the final stage in a FST bruteforcer. So I'm leaving off the data extraction and logging
                // till another day, and instead we'll just write the xyz of solutions in a CSV so we can make a black-and-white
                // graph of solutions, as a crappy stopgap that should suffice.
                wfSolutionsCSV << norm[0] << ", " << norm[1] << ", " << norm[2] << std::endl;
            
            
            }
        }
    }
    wfSolutionsCSV.close();

    cudaFree(departuresGPU);
    cudaFree(arrivalsGPU);
    cudaFree(landingsGPU);

    wfNormalStages.write(normalStages, numy * numx * numz);
    free(normalStages);
    wfNormalStages.close();

    printf("Complete!");

    return 0;
}
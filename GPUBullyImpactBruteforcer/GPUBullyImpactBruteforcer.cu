#include <fstream>
#include <cstring>
#include <string> 
#include <unordered_map>
#include <iostream>
#include <cmath>
#include "cuda.h"
#include "cuda_runtime.h"
#include "device_launch_parameters.h"
#include "device_atomic_functions.h"

#include "../Common/BloomFilter.cuh"
#include "../Common/IOUtils.hpp"
#include "../Common/Trig.cuh"
#include "../Common/VMath.cuh"

#include "BullyImpactStructs.hpp"

# define MAX_INITIALS_PER_ARRIVAL 10000
# define MAX_GOOD_IMPACTS_PER_ARRIVAL 10000
# define MAX_IMPACTS 1000000

using namespace BITFS;


// so, the masterFilter is going to be the bloom filter. initialImpactsLog is going to be where we store the immediate post-impact
// state of a bully if it looks good and isn't a duplicate. goodImpactsLog is going to be where we store how long the bully is stable 
// for. finalImpactsLog is going to be where we accumulate all our solutions in. nInitials and nImpacts are incremented to tell us
// how far in the impact logs to look for, and nSolutions is incremented to tell us how big the final impact log is going to be.
__device__ bool* masterFilter;
__device__ BullyData* initialImpactsLog;
__device__ int nInitials;
__device__ SecondaryData* goodImpactsLog;
__device__ int nImpacts;
__device__ ImpactData* finalImpactsLog;
__device__ int nSolutions = 0;




// The bully collision simulator.
__device__ BullyData sim_bully_collision(float* marioPos, float* bullyPos, int facingAngle, int bullyMovingAngle, float marioVel, float bullySpeed) {

    float offsetX = marioPos[0] - bullyPos[0];
    float offsetZ = marioPos[2] - bullyPos[2];

    // Removed unecessary distance calculation with sqrtf

    int pushAngle;

    if (offsetX * offsetX + offsetZ * offsetZ == 0.0f) {
        pushAngle = fix(facingAngle);
    }
    else {
        pushAngle = fix(atan2s(offsetZ, offsetX));
    }

    int bullyOldYaw = fix(bullyMovingAngle);

    float newMarioX = bullyPos[0] + 115.0f * sm64_sins(pushAngle);
    float newMarioZ = bullyPos[2] + 115.0f * sm64_coss(pushAngle);

    float marioSpeed = -1.0f * marioVel;
    int marioYaw = fix(facingAngle + 0x8000);

    float marioVelX = marioSpeed * sm64_sins(marioYaw);
    float marioVelZ = marioSpeed * sm64_coss(marioYaw);

    float rx = bullyPos[0] - newMarioX;
    float rz = bullyPos[2] - newMarioZ;

    float bullyVelX = bullySpeed * sm64_sins(bullyOldYaw);
    float bullyVelZ = bullySpeed * sm64_coss(bullyOldYaw);

    float projectedV1 = (rx * marioVelX + rz * marioVelZ) / (rx * rx + rz * rz);
    float projectedV2 = (-rx * bullyVelX - rz * bullyVelZ) / (rx * rx + rz * rz);

    bullyVelX += (53.0f / 73.0f) * projectedV1 * rx - projectedV2 * -rx;
    bullyVelZ += (53.0f / 73.0f) * projectedV1 * rz - projectedV2 * -rz;

    int bullyYaw = fix(atan2s(bullyVelZ, bullyVelX));
    float bullyVel = sqrtf(bullyVelX * bullyVelX + bullyVelZ * bullyVelZ);

    struct BullyData solution;
    solution.posBully[0] = bullyPos[0];
    solution.posBully[1] = bullyPos[1];
    solution.posBully[2] = bullyPos[2];
    solution.angle = bullyYaw;
    solution.velBully = bullyVel;

    return solution;

}




// This takes a bully and time-evolves it for up to "ticks" ticks of the HAU-clock. If the bully "survives" (read: stays in the FST
// position when it's in the main universe, and doesn't zip off to a PU) for x ticks of the HAU-clock then this function spits out 
// x * 16 as an estimate for the number of frames the bully is stable for.
__device__ int stability_frames(BullyData bully, int ticks) {

    
    // we aren't keeping track of y elevation, so we have a bunch of floats.
    // location is where the bully starts off, in the main universe.
    // nearoob is where the bully tries (and fails) to move to, out-of-bounds.
    // farlocation is where the bully reflects to, somewhere off in the PU's.
    // faroob is where the bully tries (and fails) to move to after that, somewhere very far away
    // and then hopefully the bully reflects back to its starting location.
    float location[2];
    float nearoob[2];
    float farlocation[2];
    float faroob[2];

    // initialize the bully starting angle and position. It'll evolve as time goes on.
    int theta = fix(bully.angle);
    location[0] = bully.posBully[0];
    location[1] = bully.posBully[2];


    // iterate over intervals of 16-frames (so each tick of this for loop simulates a forward-then-back pair of bully motions).
    // after each tick, we can skip forward 14 frames because that will land us in the middle of the bully performing its next
    // meaningfully distinct behavior.
    for (int i = 0; i < ticks; i++) {

        
        // populate the positions. First nearoob, then the angle flips around to send the bully to farlocation, then faroob
        // then the angle flips around to send the bully to location again.
        nearoob[0] = location[0] + sm64_sins(theta) * bully.velBully;
        nearoob[1] = location[1] + sm64_coss(theta) * bully.velBully;
        theta = fix(theta + 32767);
        farlocation[0] = location[0] + sm64_sins(theta) * bully.velBully;
        farlocation[1] = location[1] + sm64_coss(theta) * bully.velBully;
        faroob[0] = farlocation[0] + sm64_sins(theta) * bully.velBully;
        faroob[1] = farlocation[1] + sm64_coss(theta) * bully.velBully;
        theta = fix(theta + 32767);
        location[0] = farlocation[0] + sm64_sins(theta) * bully.velBully;
        location[1] = farlocation[1] + sm64_coss(theta) * bully.velBully;

        
        // now, as stated, there are three critical checks we need to do. First, ensuring that nearoob and faroob are both
        // out of bounds. Second, ensuring that the new location (where the bully returned to when it reflected) is at the FST 
        // coordinates. an important note is that i is actually "how many "ticks" of the bully clock have passed" and it must be 
        // multiplied by 16 to estimate the number of frames, since one "tick" happens every 16 frames.
        if ( (short)(int)nearoob[0] > -8192 && (short)(int)nearoob[0] < 8192 && (short)(int)nearoob[1] > -8192 && (short)(int)nearoob[1] < 8192) {
            return i * 16;
        }
        if ( (short)(int)faroob[0] > -8192 && (short)(int)faroob[0] < 8192 && (short)(int)faroob[1] > -8192 && (short)(int)faroob[1] < 8192) {
            return i * 16;
        }
        if (fabs(location[0] + 3120.0f) > 0.1f || fabs(location[1] + 896.0f) > 0.1f) {
            return i * 16;
        }

            
        // one last bit, since i is bully clock "ticks" and each tick is 16 frames and we've advanced 2 frames while simulating
        // thereby making theta 2 less than what it was, we need to subtract 14 from theta so that, on the next "tick", 
        // theta starts off 16 frames ahead of where it was on the previous tick, and the angle is 16 less than what it started as.
        theta = fix(theta - 14);
    }


    // if we got out of the loop, the bully is stable for a minimum of "ticks" ticks of the HAU-clock. return this fact.
    return ticks * 16;
}




__global__ void copy_pointers_to_gpu(bool* p1, BullyData* p2, SecondaryData* p3, ImpactData* p4) {
    masterFilter = p1;
    initialImpactsLog = p2;
    goodImpactsLog = p3;
    finalImpactsLog = p4;
}




// takes the bloom filter that's on the GPU and purifies it by wiping it clean with all 0's.
__global__ void purify_filter() {

    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= 65536) {
        return;
    }
    masterFilter[idx] = false;

    
}




// this gets a bully collision, checks whether a mario-bully collision is possible, and if so, simulates the impact.
// then impacts with the wrong bully parity or which make the bully go too slow or too fast are thrown out.
// a bloom filter for unique impacts is run, and only after that is the collision saved. We have up to 4 million things to test
// so it's useful to have less data get to the end to run through the more expensive simulation step later.
// a critical warning about this, however. This means that for each solution that actually gets written to the end file
// there are multiple bully positions which would produce that same impact! So I should probably come up with some code later
// to take a promising solution and find the bully positions that produce that exact solution.
__global__ void initial_assessment(ApproachData mario, BullyData bullyCentral, int nx, int nz, float minx, float minz, float gran) {

    
    // get the number of increments in the x and z direction from the thread id. The % nz is intended.
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= nx * nz) {
        return;
    }
    int overx = idx % nz;
    int overz = (idx - overx) / nz;

    
    // load up our bully data.
    struct BullyData bully;
    bully.posBully[0] = minx + (overx * gran);
    bully.posBully[1] = bullyCentral.posBully[1];
    bully.posBully[2] = minz + (overz * gran);
    bully.angle = bullyCentral.angle;
    bully.velBully = bullyCentral.velBully;


    // compute the distance that Mario is at from the bully position. If it's too high, automatically give up.
    if (find_dis(mario.posArrive, bully.posBully) > 63.0f) {
        return;
    }


    // ok, Mario can impact with the bully. Simulate the impact, to update the bully state.
    bully = sim_bully_collision(mario.posArrive, bully.posBully, mario.facingArrive, bully.angle, mario.velArrive, bully.velBully);


    // throw out if the angle impact parity is odd (because that moves the pivot) or the bully velocity is too low (<400 million)
    // or too high (>1 billion).
    if (bully.angle % 2 == 1 || bully.velBully < 4.0e+08 || bully.velBully > 1.0e+09) {
        return;
    }


    // see if the bully returns to its FST location. stability_frames(bully, 1) would be 16 if 
    // the bully returns to the FST location and 0 otherwise.
    bool toFST = (stability_frames(bully, 1) > 0);
    if (!toFST) {
        return;
    }


    // now that we know we can hit the bully and give it a good speed and the pivot stays in the main universe
    // and it rounds off to the FST location, we hash it to get four locations in the bloom filter. 
    // Check them to eliminate duplicates (two bully positions which produce the same impact)
    // I just filled the last piece of the hash function with some random shit.
    int locations[4];
    bool unique = false;
    hash(bully.velBully, (float)bully.angle, 23462432.0f, locations, 16);
    for (int k = 0; k < 4; k++) {
        if (!masterFilter[locations[k]]) {
            unique = true;
            masterFilter[locations[k]] = true;
        }
    }
    if (!unique) {
        return;
    }


    // increment our solution counter and record our solution.
    int solIdx = atomicAdd(&nInitials, 1);
    if (solIdx > MAX_INITIALS_PER_ARRIVAL) {
        return;
    }
    struct BullyData* data = &(initialImpactsLog[solIdx]);
    data->posBully[0] = bully.posBully[0];
    data->posBully[1] = bully.posBully[1];
    data->posBully[2] = bully.posBully[2];
    data->angle = bully.angle;
    data->velBully = bully.velBully;
}




// evolves the bullies in time to see how long they last.
__global__ void time_evolution(int size) {


    // get the place-to-look-at from the thread id.
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= size) {
        return;
    }

    
    // simulate the bully time evolution and throw it out if the bully is sufficiently unstable (< 400 frames ie about 15 seconds)
    int duration = stability_frames(initialImpactsLog[idx], 200);
    if (duration < 400) {
        return;
    }


    // increment the solution counter and record our pointer to where we have to look, and how stable it is.
    int solIdx = atomicAdd(&nImpacts, 1);
    if (solIdx > MAX_GOOD_IMPACTS_PER_ARRIVAL) {
        return;
    }
    struct SecondaryData* data = &(goodImpactsLog[solIdx]);
    data->tag = idx;
    data->frames = duration;
}




// adds the data to our ever-growing solution list. identifier tells us which mario-arrival to link the bully data up with.
__global__ void append_info(ApproachData approach, int size) {


    // initialize our counter
    int counter = 0;


    // iterate up to the number of impacts we're looking at.
    for (int i = 0; i < size; i++) {


        // continue if we are over our bound.
        if (nSolutions + counter >= MAX_IMPACTS) {
            continue;
        }


        // start off by snagging the data we need.
        SecondaryData* info = &(goodImpactsLog[i]);
        BullyData* bully = &(initialImpactsLog[info->tag]);


        // write data to the structs
        finalImpactsLog[nSolutions + counter].bully.posBully[0] = bully->posBully[0];
        finalImpactsLog[nSolutions + counter].bully.posBully[1] = bully->posBully[1];
        finalImpactsLog[nSolutions + counter].bully.posBully[2] = bully->posBully[2];
        finalImpactsLog[nSolutions + counter].bully.angle = bully->angle;
        finalImpactsLog[nSolutions + counter].bully.velBully = bully->velBully;
        finalImpactsLog[nSolutions + counter].routeID = approach.solutionID;
        finalImpactsLog[nSolutions + counter].frames = info->frames;


        // then increment the counter
        counter++;
    }


    // and increase the total number of solutions we have logged.
    nSolutions += counter;
}


// Function to extract specific columns (with help from ChatGPT).
int load_arrival_structs_from_csv(std::string csv_path, ApproachData* &arrival_data_array) {
    
    std::vector<std::unordered_map<std::string, std::string>> csv_data = parse_csv(csv_path);

    int nApproaches = csv_data.size();
    arrival_data_array = (struct ApproachData*)std::malloc(nApproaches * sizeof(struct ApproachData));

    int counter = 0;
    for (const auto& row : csv_data) {

        // Arrival Position X
        auto kv_pair = row.find(" Arrival Position X");
        if (kv_pair != row.end()) {
            arrival_data_array[counter].posArrive[0] = std::stof(kv_pair->second);
        }

        // Arrival Position Y
         kv_pair = row.find(" Arrival Position Y");
        if (kv_pair != row.end()) {
            arrival_data_array[counter].posArrive[1] = std::stof(kv_pair->second);
        }

        // Arrival Position Z
        kv_pair = row.find(" Arrival Position Z");
        if (kv_pair != row.end()) {
            arrival_data_array[counter].posArrive[2] = std::stof(kv_pair->second);
        }

        // Arrival Velocity
        kv_pair = row.find(" Arrival Velocity");
        if (kv_pair != row.end()) {
            arrival_data_array[counter].velArrive = std::stof(kv_pair->second);
        }

        // Arrival Angle
        kv_pair = row.find(" Arrival Angle");
        if (kv_pair != row.end()) {
            arrival_data_array[counter].facingArrive = std::stoi(kv_pair->second);
        }

        // Solution ID
        kv_pair = row.find(" solution ID");
        if (kv_pair != row.end()) {
            arrival_data_array[counter].solutionID = std::stoi(kv_pair->second);
        }

        counter++;
    }

    std::cout << "CSV Parsed.\n";

    return nApproaches;
}




int main(int argc, char* argv[]) {
    std::cout << "Starting Bully Impact Bruteforcer...\n";
    
    // initialize the prototype bully data. The X and Z are absolutely pinned down but the Y, angle, and velocity, may be messed with.
    struct BullyData bullyCentral;
    bullyCentral.posBully[0] = -3120.0f;
    bullyCentral.posBully[1] = -2976.0f;
    bullyCentral.posBully[2] = -896.0f;
    bullyCentral.angle = 11732;
    bullyCentral.velBully = 30000.0f;

    
    // initialize the float granularity for positions, and threads, and output file
    float granularity = 0.0078125f;
    int nThreads = 256;
    std::string outFile = "goodImpacts.csv";
    std::string inFile = "bloomVersion2.csv";
    bool verbose = false;

    
    // take input. Warning, I might have fucked up the file input reading.
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "-h") || !strcmp(argv[i], "--help")) {
            printf("BitFS Bully Impact Brute Forcer.\n");
            printf("This program accepts the following options:\n\n");
            printf("-an <angle>: Angle the bully has at time of impact. \n");
            printf("             Default: %d\n", bullyCentral.angle);
            printf("-vel <speed>: Velocity the bully has at time of impact. \n");
            printf("             Default: %f\n", bullyCentral.velBully);
            printf("-i: Path to the input file.\n");
            printf("    Default: %s\n", inFile.c_str());
            printf("-o: Path to the output file.\n");
            printf("    Default: %s\n", outFile.c_str());
            printf("-v: Verbose mode. Prints all parameters used in brute force.\n");
            printf("    Default: off\n");
            printf("-h --help: Prints this text.\n");
            exit(0);
        }
        else if (!strcmp(argv[i], "-an")) {
            bullyCentral.angle = std::stoi(argv[i + 1]);
            i += 1;
        }
        else if (!strcmp(argv[i], "-vel")) {
            bullyCentral.velBully = std::stoi(argv[i + 1]);
            i += 1;
        }
        else if (!strcmp(argv[i], "-o")) {
            outFile = argv[i + 1];
            i += 1;
        }
        else if (!strcmp(argv[i], "-i")) {
            inFile = argv[i + 1];
            i += 1;
        }
        else if (!strcmp(argv[i], "-v")) {
            verbose = true;
        }
        if (verbose) {
            printf("Bully Starting Angle: %d\n", bullyCentral.angle);
            printf("Bully Starting Speed: %f\n", bullyCentral.velBully);
        }
    }

    
    // load up our array of ways to arrive. Very high chance that something here is botched.
    struct ApproachData* arrivalList;
    int nApproaches = load_arrival_structs_from_csv(inFile, arrivalList);

    std::cout << "Cuda Mallocs Starting...\n";

    // initialize the memory for the bloom filter, unique initial impacts, good impacts (those which persist a long time)
    // and the final impact log.
    bool* filterGPU;
    cudaMalloc((void**)&filterGPU, 65536 * sizeof(bool));
    BullyData* initialImpactsGPU;
    cudaMalloc((void**)&initialImpactsGPU, MAX_INITIALS_PER_ARRIVAL * sizeof(BullyData));
    SecondaryData* goodImpactsGPU;
    cudaMalloc((void**)&goodImpactsGPU, MAX_GOOD_IMPACTS_PER_ARRIVAL * sizeof(SecondaryData));
    ImpactData* finalImpactsGPU;
    cudaMalloc((void**)&finalImpactsGPU, MAX_IMPACTS * sizeof(ImpactData));

    
    // and get those pointers onto the GPU.
    copy_pointers_to_gpu << <1, 1 >> > (filterGPU, initialImpactsGPU, goodImpactsGPU, finalImpactsGPU);
    
    // we'll be taking the overlap of the boxes "stuff within an Linfinity distance of 63 from Mario" and "stuff within an
    // Linfinity distance of 16 from the bully" to get our candidates for perturbed bully positions. Initialize a bunch of floats
    // pertaining to that. The first argument is 0 for min and 1 for max, and the second argument is 0 for x and 1 for z.
    float bpos[2][2];
    float mpos[2][2];
    float zone[2][2];
    bpos[0][0] = bullyCentral.posBully[0] - 16.0f;
    bpos[0][1] = bullyCentral.posBully[2] - 16.0f;
    bpos[1][0] = bullyCentral.posBully[0] + 16.0f;
    bpos[1][1] = bullyCentral.posBully[2] + 16.0f;

    // Now, iterate over possible ways that Mario could arrive near the bully.
    for (int i = 0; i < nApproaches; i++) {
        
        mpos[0][0] = arrivalList[i].posArrive[0] - 63.0f;
        mpos[0][1] = arrivalList[i].posArrive[2] - 63.0f;
        mpos[1][0] = arrivalList[i].posArrive[0] + 63.0f;
        mpos[1][1] = arrivalList[i].posArrive[2] + 63.0f;

        // compute intersection of boxes.
        zone[0][0] = fmaxf(bpos[0][0], mpos[0][0]);
        zone[0][1] = fmaxf(bpos[0][1], mpos[0][1]);
        zone[1][0] = fminf(bpos[1][0], mpos[1][0]);
        zone[1][1] = fminf(bpos[1][1], mpos[1][1]);
        
        // nx and nz are "if we take our rectangle of possible bully starting positions and discretize it according to the
        // float granularity we're likely to have, how many positions are there in the X and Z coordinates?". Maybe the
        // rectangles of viable bully positions don't intersect, in which case we'd expect some of the deltas to be negative
        // so we automatically move onto the next loop if that happens because no starting bully positions would work anyways.
        float deltax = zone[1][0] - zone[0][0];
        float deltaz = zone[1][1] - zone[0][1];
        int nx = (int)(deltax / granularity) + 1;
        int nz = (int)(deltaz / granularity) + 1;
        if (nx <= 0 || nz <= 0) {
            continue;
        }

        // start off by washing the bloom filter clean so it's filled with all 0's.
        int nBloomSlots = (65536 + nThreads - 1) / nThreads;
        purify_filter << <nBloomSlots, nThreads >> > ();

        // we'll be testing nx * nz possible bully positions which *might* snap to the FST position after an impact.
        // so we set up the blocks, initialize to 0 solutions, push the solution counter to the GPU, test the bully positions
        // with the initial_assessment function, and pull the solution counter back out of the GPU. Bloom filtering to eliminate
        // duplicate solutions is done implicitly in the initial_assessment function.
        int nFirstBlocks = (nx * nz + nThreads - 1) / nThreads;
        int nInitialsCPU = 0;
        cudaMemcpyToSymbol (nInitials, &nInitialsCPU, sizeof(int), 0, cudaMemcpyHostToDevice);
        initial_assessment << <nFirstBlocks, nThreads >> > (arrivalList[i], bullyCentral, nx, nz, zone[0][0], zone[0][1], granularity);
        cudaMemcpyFromSymbol (&nInitialsCPU, sizeof(int), 0, cudaMemcpyDeviceToHost);

        
        // if no solutions or too many solutions, continue to the next loop or clip off the number of solutions.
        if (nInitialsCPU > MAX_INITIALS_PER_ARRIVAL) {
            fprintf(stderr, "Warning: The number of initial impacts has been exceeded. No more will be recorded. Increase the internal maximum to prevent this from happening.\n");
            nInitialsCPU = MAX_INITIALS_PER_ARRIVAL;
        }
        if (nInitialsCPU == 0) {
            continue;
        }

        std::cout << "Initial Solutions Found.\n";

        // we'll be testing (number of solutions to the first stage) bully positions which have promise, to see
        // whether they indeed snap to the FST position after impact, and how long they remain stable there. Same thing.
        // Set up the blocks, initialize to 0 solutions, push the solution counter to GPU, time-evolve the bully, and
        // pull the solution counter back out. Bloom filtering isn't done because there's negligible probability of having
        // two distinct impacts produce identical end states.
        int nSecondBlocks = (nInitialsCPU + nThreads - 1) / nThreads;
        int nImpactsCPU = 0;
        cudaMemcpyToSymbol (nImpacts, &nImpactsCPU, sizeof(int), 0, cudaMemcpyHostToDevice);
        time_evolution << <nSecondBlocks, nThreads >> > (nInitialsCPU);
        cudaMemcpyFromSymbol (&nImpactsCPU, sizeof(int), 0, cudaMemcpyDeviceToHost);

        
        // again, clip if too many solutions and continue if no solutions.
        if (nImpactsCPU > MAX_GOOD_IMPACTS_PER_ARRIVAL) {
            fprintf(stderr, "Warning: The number of good impacts has been exceeded. No more will be recorded. Increase the internal maximum to prevent this from happening.\n");
            nImpactsCPU = MAX_GOOD_IMPACTS_PER_ARRIVAL;
        }
        if (nImpactsCPU == 0) {
            continue;
        }

        
        // append the data to the master log of solutions.
        append_info << <1, 1 >> > (arrivalList[i], nImpactsCPU);
    }
    
    
    // free up memory. Note that we don't free up our final log of solutions.
    cudaFree(filterGPU);
    cudaFree(initialImpactsGPU);
    cudaFree(goodImpactsGPU);

    
    // figure out how many solutions we have.
    printf("writing to file...\n");
    int nSolutionsCPU = 0;
    cudaMemcpyFromSymbol(&nSolutionsCPU, nSolutions, sizeof(int), 0, cudaMemcpyDeviceToHost);
    printf("%d solutions found!\n", nSolutionsCPU);

    
    // get the solutions from the GPU to the CPU.
    struct ImpactData* finalImpactLog = (struct ImpactData*)std::malloc(nSolutionsCPU * sizeof(struct ImpactData));
    cudaMemcpy(finalImpactLog, finalImpactsGPU, nSolutionsCPU * sizeof(struct ImpactData), cudaMemcpyDeviceToHost);

    
    // ok, at this point all our GPU shit is over and we've got a bunch of solutions in a table. It's time
    // to start writing this shit into a file.
    std::ofstream wf(outFile);
    wf << std::fixed;
    wf << "Bully Position X, Bully Position Z, ";
    wf << "Bully Angle, Bully Velocity, ";
    wf << "solution ID, Frames of Stability" << std::endl;
    for (int m = 0; m < nSolutionsCPU; m++) {
        wf << finalImpactLog[m].bully.posBully[0] << ", " << finalImpactLog[m].bully.posBully[2] << ", ";
        wf << finalImpactLog[m].bully.angle << ", " << finalImpactLog[m].bully.velBully << ", ";
        wf << finalImpactLog[m].routeID << ", " << finalImpactLog[m].frames << std::endl;
    }
    wf.close();

    
    // free up memory.
    std::free(arrivalList);
    std::free(finalImpactLog);
    cudaFree(finalImpactsGPU);
    
    
    // end
    printf("Complete!");
    return 0;
}
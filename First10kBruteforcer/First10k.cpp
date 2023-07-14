#include <fstream>
#include <cstring>
#include <string>
#include <unordered_map>
#include <iostream>
#include <cmath>
#include <ostream>
#include <iomanip>
#include <cstdlib>

#include "../Common/BruteforcerStructs.hpp"
#include "../Common/Floor.cuh"
#include "../Common/Movement.cuh"
#include "../Common/Stick.cuh"
#include "../Common/Surface.cuh"
#include "../Common/Trig.cuh"   

using namespace BITFS;

std::ofstream out_stream;

bool validCameraAngles[65536];

void init_camera_angles() {
    for (int i = 0; i < 65536; i++) {
        validCameraAngles[i] = false;
    }
    for (int hau = 0; hau < 4096; hau++) {
        validCameraAngles[fix(atan2s(-gCosineTableG[hau], -gSineTableG[hau]))] = true;
    }
}


int fine_camera_yaw(float* currentPosition, float* lakituPosition, short faceAngle) {
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

    float cameraPos[3] = { currentPosition[0] + baseCameraDist * gCosineTableG[fix((int)baseCameraPitch) >> 4] * gSineTableG[fix((int)baseCameraYaw) >> 4],
                       currentPosition[1] + 125.0f - 10.0f + baseCameraDist * gSineTableG[fix((int)baseCameraPitch) >> 4],
                       currentPosition[2] + baseCameraDist * gCosineTableG[fix((int)baseCameraPitch) >> 4] * gCosineTableG[fix((int)baseCameraYaw) >> 4]
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
    pan[2] = gSineTableG[0xC0] * cameraDist;

    temp[0] = pan[0];
    temp[1] = pan[1];
    temp[2] = pan[2];

    pan[0] = temp[2] * gSineTableG[fix((int)faceAngle) >> 4] + temp[0] * gCosineTableG[fix((int)faceAngle) >> 4];
    pan[2] = temp[2] * gCosineTableG[fix((int)faceAngle) >> 4] - temp[0] * gSineTableG[fix((int)faceAngle) >> 4];

    // rotate in the opposite direction
    cameraYaw = -cameraYaw;

    temp[0] = pan[0];
    temp[1] = pan[1];
    temp[2] = pan[2];

    pan[0] = temp[2] * gSineTableG[fix((int)cameraYaw) >> 4] + temp[0] * gCosineTableG[fix((int)cameraYaw) >> 4];
    pan[2] = temp[2] * gCosineTableG[fix((int)cameraYaw) >> 4] - temp[0] * gSineTableG[fix((int)cameraYaw) >> 4];

    // Only pan left or right
    pan[2] = 0.f;

    cameraYaw = -cameraYaw;

    temp[0] = pan[0];
    temp[1] = pan[1];
    temp[2] = pan[2];

    pan[0] = temp[2] * gSineTableG[fix((int)cameraYaw) >> 4] + temp[0] * gCosineTableG[fix((int)cameraYaw) >> 4];
    pan[2] = temp[2] * gCosineTableG[fix((int)cameraYaw) >> 4] - temp[0] * gSineTableG[fix((int)cameraYaw) >> 4];

    float cameraFocus[3] = { currentPosition[0] + pan[0], currentPosition[1] + 125.0f + pan[1] - 90.0f, currentPosition[2] + pan[2] };
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

    cameraFocus[0] = lakituPosition[0] + cameraDist * gCosineTableG[fix((int)cameraPitch) >> 4] * gSineTableG[fix((int)cameraYaw) >> 4];
    cameraFocus[1] = lakituPosition[1] + cameraDist * gSineTableG[fix((int)cameraPitch) >> 4];
    cameraFocus[2] = lakituPosition[2] + cameraDist * gCosineTableG[fix((int)cameraPitch) >> 4] * gCosineTableG[fix((int)cameraYaw) >> 4];

    return atan2s(lakituPosition[2] - cameraFocus[2], lakituPosition[0] - cameraFocus[0]);
}

// simple tester for being on the appropriate side of the one up platform.
bool onOneUp(float* position) {
    if (assess_floor(position) == 3) {
        Surface* floor;
        float floorheight;
        int floorIdx = find_floor(position, &floor, floorheight, floorsG, total_floors);
        
        if(position[0] > 0.0f && (floorIdx == 2 || floorIdx == 3)) {
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


//simulates 1QF of HAU-aligned travel in the air, including floor snap up for recalculating things and being precise about it.
AirInfo sim_airstep(float* initialPos, float initialSpeed, int initialAngle, bool first) {
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

    float velX = speed * gSineTableG[angle >> 4];
    float velZ = speed * gCosineTableG[angle >> 4];
        
    nextPos[0] += velX / 4.0f;
    nextPos[2] += velZ / 4.0f;
    
    Surface* floor;
    float floorheight;
    int floorIdx = find_floor(nextPos, &floor, floorheight, floorsG, total_floors);
    nextPos[1] = fmaxf(floorheight, nextPos[1]);
    
    struct AirInfo solution;
    solution.endSpeed = speed;
    solution.endPos[0] = nextPos[0];
    solution.endPos[1] = nextPos[1];
    solution.endPos[2] = nextPos[2];
    
    return solution;
}

int main(int argc, char* argv[]) {
    
    // the assumed starting position that Mario is launching from.
    float startX = -1700.0f;
    float startZ = -350.0f;
    
    float cameraPosition[3];
    cameraPosition[0] = -1700.0f;
    cameraPosition[1] = -2300.0f;
    cameraPosition[2] = -2000.0f; // for our iteration over 40 camera positiosn, this scans -2000 to 2000
    
    
    // the assumed bounds on the de-facto speed multiplier for the pyramid platform.
    float lowerNY = 0.88f;
    float upperNY = 0.96f;
    
    float tenKPosition[3];
    tenKPosition[0] = -4561.0f + (2.0f * 65536.0f);// by far the best results with post-10k speed were attained from 10king
    // off the 1-up platform in PU X = 2. -4038 is the coordinate to add for -X travel, -4561 for +X.
    tenKPosition[1] = -2948.0f;
    tenKPosition[2] = startZ;
    
    float lowerSpeed = fabs(tenKPosition[0] - startX) * (4.0f / (1.0f + upperNY)); 
    // this assumes 1QF of air movement
    // and takes the distance between the 10k position and your starting position, along with the range of de-facto speed
    // multipliers possible, to find the slowest you can possibly hit the 1-up platform with. Change if looking for other routes.
    float upperSpeed = fabs(tenKPosition[0] - startX) * (4.0f / (1.0f + lowerNY)); 
    // this assumes 1QF of air movement and does a similar thing, finding the fastest you could possibly hit the 1-up platform with.
    printf("floats(%f,%f)\n", lowerSpeed, upperSpeed);
    
    initialise_floors();
    init_stick_tables();
    init_camera_angles();
    
    // this for loop checks once every 100 X coordinates, from -2000X to +2000X. Feel free to tamper to scan other cam coords.
    for (int delta = 0; delta <= 40; delta++) {
        cameraPosition[2] += 100.0f;
        
        // try once every 10 speeds from slowest to fastest to see if you can get a setup.

        for (int s = (int)(lowerSpeed / 10.0f); s <= (int)(upperSpeed / 10.0f); s++) {
            
            float vel = (float)(s * 10);
            
            // infer back to what the platform slope must be to hit the 10k with the indicated speed.
            // if tampering with the PU you're going to, be very careful with all the stuff to avoid sign errors in the speed.
            // assumes 1QF of travel.
            float nY = ((4.0f * fabs(tenKPosition[0] - startX)) / vel) - 1.0f;
            
            // at the time of 10k impact, the camera will be focusing on an 80-20 mix of where you were last frame, and where
            // the camera was focusing at 2 frames ago. So it's an 80/20 mix of your frame 1 position and your starting position.
            // note that it's assumed we're going straight sideways.
            float frame1Focus[3];
            frame1Focus[0] = 0.8f * (vel * (nY / 4.0f) + startX) + 0.2f * startX;
            frame1Focus[1] = -2970.0f;
            frame1Focus[2] = startZ;
            
            // this is the camera yaw at time of 10k.
            int c = fine_camera_yaw(frame1Focus, cameraPosition, 16384);
            
            // we're iterating over fewer stick positions, only the ones that pull back.
            for (int j = 0; j < 6236; j++) {
                
                // for going in the -X direction, the angles should both be 16384 + 32768, and the speeds should be vel -vel.
                FancySlideInfo tenkslide = sim_slide(stickTab[j], tenKPosition, vel, vel, 0.0f, 16384, 16384, c);
                
                // must be going backwards.
                if(tenkslide.endSpeed > 0.0f) {
                    continue;
                }
                
                // must end up in air.
                if (assess_floor(tenkslide.endPos) != 1) {
                    continue;
                }
                
                // simulate air movement. Must land on ground.
                AirInfo tenkair = sim_airstep(tenkslide.endPos, tenkslide.endSpeed, tenkslide.endFacingAngle, true);
                if (assess_floor(tenkair.endPos) != 2) {
                    continue;
                }
                
                // are we stable?
                bool stable = stability_check(tenkair.endPos, tenkair.endSpeed, tenkslide.endFacingAngle);
                if (!stable) {             
                    continue;         
                }
                
                // the best ones I've ever found wound up in PU X = -29, so filter for those for the highest-speed post-10k results.
                // the print statement tells you the camera Z coordinate, the speed, the RAW stick position (I forgot to correct it),
                // the platform de-facto multiplier/slope that lets you hit the 1-up with that speed, and your X/Z coordinates, and
                // ending speed.
                
                // it is advised to not feed this data straight into the next bruteforcer, rather, use it as a guide to TAS empirically
                // and feed your empirical end state into the next bruteforcer.
                if (round(tenkair.endPos[0] / 65536.0f) == -29) {
                    printf("hit(%f,%d,(%d,%d),%f,(%f,%f),%f)\n", cameraPosition[2], s * 10, stickTab[j].stickX, stickTab[j].stickY, nY, tenkair.endPos[0] + 29.0f * 65536.0f, tenkair.endPos[2], tenkair.endSpeed);
                }
            }
        }
    }
    return 0;
}
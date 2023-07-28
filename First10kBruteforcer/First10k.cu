#include <fstream>
#include <cstring>
#include <string>
#include <unordered_map>
#include <iostream>
#include <cmath>
#include <ostream>
#include <iomanip>
#include <cstdlib>

#include "../Common/CommonBruteforcerStructs.hpp"
#include "../Common/Camera.cuh"
#include "../Common/Floors.cuh"
#include "../Common/Movement.cuh"
#include "../Common/Stick.cuh"
#include "../Common/Surface.cuh"
#include "../Common/Trig.cuh"   

using namespace BITFS;

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

    std::string outFile = "firstData.csv";
    
    float lowerSpeed = fabs(tenKPosition[0] - startX) * (4.0f / (1.0f + upperNY)); 
    // this assumes 1QF of air movement
    // and takes the distance between the 10k position and your starting position, along with the range of de-facto speed
    // multipliers possible, to find the slowest you can possibly hit the 1-up platform with. Change if looking for other routes.
    float upperSpeed = fabs(tenKPosition[0] - startX) * (4.0f / (1.0f + lowerNY)); 
    // this assumes 1QF of air movement and does a similar thing, finding the fastest you could possibly hit the 1-up platform with.
    printf("floats(%f,%f)\n", lowerSpeed, upperSpeed);
    
    initialise_floors();
    init_stick_tables(true);
    init_camera_angles();

    std::ofstream wf(outFile);
    wf << std::fixed;
    wf << "Camera X,Camera Y,Camera Z,";
    wf << "Velocity,Stick X,Stick Y,nY,10k Camera Yaw,";
    wf << "10k End Pos X,10k End Pos Y,10k End Pos Z,";
    wf << "10k End Speed,10k End Facing Angle,10k End Sliding Angle,";
    wf << "Air End Pos X,Air End Pos Y,Air End Pos Z,Air End Speed\n";
    
    // this for loop checks once every 100 X coordinates, from -2000X to +2000X. Feel free to tamper to scan other cam coords.
    for (int delta = 0; delta <= 40; delta++) {
        cameraPosition[2] += 100.0f;
        if(delta > 0)
            printf("\r");
        printf("Progress: %d/40", delta);
        
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
                FancySlideInfo tenkslide;

                //must get non-Null result
                if (!sim_slide(stickTab[j], tenKPosition, vel, vel, 0.0f, 16384, 16384, c, false, tenkslide)) {
                    continue;
                }

                // must be going backwards.
                if(tenkslide.endSpeed > 0.0f) {
                    continue;
                }
                
                // must end up in air.
                if (assess_floor(tenkslide.endPos) != 1) {
                    continue;
                }
                
                // simulate air movement. Must land on ground.
                AirInfo tenkair;
                if(!sim_airstep(tenkslide.endPos, tenkslide.endSpeed, tenkslide.endFacingAngle, true, tenkair) || assess_floor(tenkair.endPos) != 2) {
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
                    wf << cameraPosition[0] << "," << cameraPosition[1] << "," << cameraPosition[2] << ",";
                    wf << s * 10 << "," << stickTab[j].stickX << "," << stickTab[j].stickY << "," << nY << "," << c << ",";
                    wf << tenkslide.endPos[0] << "," << tenkslide.endPos[1] << "," << tenkslide.endPos[2] << ",";
                    wf << tenkslide.endSpeed << "," << tenkslide.endFacingAngle << "," << tenkslide.endSlidingAngle << ",";
                    wf << tenkair.endPos[0] << "," << tenkair.endPos[1] << "," << tenkair.endPos[2] << ",";
                    wf << tenkair.endSpeed << std::endl;
                }
            }
        }
    }

    wf.close();

    return 0;
}

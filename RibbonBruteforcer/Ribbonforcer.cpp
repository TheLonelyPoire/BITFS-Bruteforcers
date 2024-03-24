// This barfs out a stream of (velocity, camera angle, max-speed solution) tuples to be assembled into a ribbon-like image.
#include <fstream>
#include <cstring>
#include <string>
#include <unordered_map>
#include <iostream>
#include <cmath>
#include <ostream>
#include <iomanip>
#include <cstdlib>
#include <unordered_set>

#include "../Common/CommonBruteforcerStructs.hpp"
#include "../Common/Camera.cuh"
#include "../Common/Floors.cuh"
#include "../Common/Movement.cuh"
#include "../Common/Stick.cuh"
#include "../Common/Surface.cuh"
#include "../Common/Trig.cuh"

using namespace BITFS;

int main(int argc, char* argv[]) {
    
    // how hard the pyramid platform can be tilted
    float nymax = 1.0f;
    float nymin = 0.85f;

    // where we're aiming for (the 1up platform in PU (2,0), given our start position and travel angle. True arrival locations
    // have some wiggle room on the Y and X coordinates but this is right smack in the middle of our small zone)
    float tenkpos[3];
    tenkpos[0] = -4559.0f + (65536.0f * 2);
    tenkpos[1] = -2944.0f;
    tenkpos[2] = -414.0f;

    // the start position assumes we get a full bully push from the bully being right at the FST location.
    float startpos[3];
    startpos[0] = -3120.0f;
    startpos[1] = -2970.0f;
    startpos[2] = -1011.0f;

    // 1021 HAU is the angle at which we travel to get to where we're going
    short angle = 16 * 1021;

    // to get our range of camera yaws, we assume we can get the camera 4000ish units off the centerline and in-bounds
    // and these are the two most extreme camera positions.
    float camposhi[3];
    camposhi[0] = 8192.0f;
    camposhi[1] = -2300.0f;
    camposhi[2] = -4096.0f;
    float camposlo[3];
    camposlo[0] = 8192.0f;
    camposlo[1] = -2300.0f;
    camposlo[2] = 4096.0f;
    
    // and from this we derive other important parameters.
    float deltax = tenkpos[0] - startpos[0];
    float deltaz = tenkpos[2] - startpos[2];
    float distance = sqrtf(deltax * deltax + deltaz * deltaz);
    
    std::cout << "Distance: " << distance << "\n";

    // such as the range of velocities to check, ie, velocities that get you to the target with attainable pyramid tilts.
    float velmin = (4.0f * distance) / (1.0f + nymax);
    float velmax = (4.0f * distance) / (1.0f + nymin);

    std::cout << "Computed Min/Max Velocities:\n";
    std::cout << "  Min Vel: " << velmin << "\n";
    std::cout << "  Max Vel: " << velmax << "\n";

    // we find the closest that we could have been 1QF before 10k happens
    float closeoneagopos[3];
    closeoneagopos[0] = startpos[0] + ((velmax * nymin / 4.0f) * sm64_sins(angle));
    closeoneagopos[1] = -2970.0f;
    closeoneagopos[2] = startpos[2] + ((velmax * nymin / 4.0f) * sm64_coss(angle));

    // use fine camera yaw to populate some data to accurately compute 10k camera yaws.
    float focusmin[3];
    float focusmax[3];
    float panmin[1];
    float panmax[1];
    float camposmin[3];
    float camposmax[3];

    std::cout << "Loading camera positions...\n";

    fine_camera_yaw(startpos, camposhi, angle, focusmin, panmin, camposmin, false);
    fine_camera_yaw(startpos, camposlo, angle, focusmax, panmax, camposmax, false);

    std::cout << "Loading Complete.\n";
    std::cout << "  Cam Pos Min: " << camposmin[0] << "," << camposmin[1] << "," << camposmin[2] << "\n";
    std::cout << "  Cam Pos Max : " << camposmax[0] << "," << camposmax[1] << "," << camposmax[2] << "\n";

    std::cout << "Loading camera thetas...\n";

    int thetamin = fix(tenk_camera_yaw(startpos, closeoneagopos, camposhi, angle, angle, focusmin, panmin, camposmin));
    int thetamax = fix(tenk_camera_yaw(startpos, closeoneagopos, camposlo, angle, angle, focusmax, panmax, camposmax));

    std::cout << "Loading Complete.\n";
    std::cout << "  Theta Min : " << thetamin << "\n";
    std::cout << "  Theta Max : " << thetamax << "\n";

    std::cout << "Initializing Bruteforcer Components...\n";

    initialise_floors();
    init_stick_tables(true);
    init_camera_angles();

    std::cout << "Initialization Complete.\n";
    std::cout << "Running Ribbonforcer...\n";

    std::string outResultsFile = "RibbonResults.csv";
    std::string outStickInfoFile = "HighSpeedSticks.csv";
    
    std::ofstream wf(outResultsFile);
    std::ofstream wfstick(outStickInfoFile);

    wf << std::fixed;
    wf << "Start Velocity,Cam Yaw,End Velocity" << std::endl;

    wfstick << std::fixed;
    wfstick << "x,y\n";

    int outer_loop_iters = (int)(velmax - velmin) / 10;
    int loop_counter = 0;
    for (int v = ((int)velmin / 10); v <= ((int)velmax / 10); v++) {
    //for (int v = 27663; v <= 27689; v++) { // STRIP OF INTEREST
        
        if (loop_counter % 10 == 0) {
            std::cout << "\rIteration " << loop_counter << "/" << outer_loop_iters;
        }

        float vel = v * 10.0f;
        
        for (int yaw = thetamin; yaw <= thetamax; yaw++) {

            if (!validCameraAngles[yaw]) {
                continue;
            }

            float bestspeed = 0.0f;
            StickTableData* beststick;
            for (int i = 0; i < NUM_STICK_TABLE_ENTRIES_BACKWARDS; i++) {
                
                FancySlideInfo tenkslide;
                if (!sim_slide(stickTab[i], tenkpos, vel, vel * sm64_sins(angle), vel * sm64_coss(angle), (int)angle, (int)angle, yaw, false, tenkslide)){
                    continue;
                }
                
                // junk everything that doesn't get negative speed.
                if (tenkslide.endSpeed >= 0.0f){
                    continue;
                }
                // the 10k must end up in the air.
                if (assess_floor(tenkslide.endPos) != 1) {
                    continue;
                }

                // simulate the air movement then, and make sure it ends up on the ground.
                AirInfo tenkair;
                bool land = false;
                for (int qf = 1; qf <= 12; qf++) {
                    if (!sim_airstep(((qf == 1) ? tenkslide.endPos : tenkair.endPos), ((qf == 1) ? tenkslide.endSpeed : tenkair.endSpeed), tenkslide.endFacingAngle, (qf % 4 == 1), tenkair)) {
                        break;
                    }
                    
                    if(assess_floor(tenkair.endPos) == 0) {
                        break;
                    }
                    else if(assess_floor(tenkair.endPos) == 2 || assess_floor(tenkair.endPos) == 3) {
                        land = true;
                        break;
                    }
                }

                if (!land) {
                    continue;
                }

                Surface* floor;
                float floorheight;
                int floorIdx = find_floor(tenkair.endPos, &floor, floorheight, floors, total_floors);

                if (floorIdx < 27 || floorIdx > 32) {
                    continue;
                }
                // and that we're stably walking against OOB.
                bool stable = stability_check(tenkair.endPos, tenkair.endSpeed, tenkslide.endFacingAngle);
                if (!stable) {
                    continue;
                }

                // sweet we got a solution, if the speed is better than our best, make it our best speed.
                if (tenkair.endSpeed < bestspeed) {
                    bestspeed = tenkair.endSpeed;
                    beststick = &stickTab[i];
                }   
                
            }

            // now that we know the best speed associated with the given ingoing speed and camera yaw, log it.
            wf << vel << ", " << yaw << ", " << bestspeed << std::endl;

            if(bestspeed <= -7.0e-6)
                wfstick << beststick->stickX << "," << beststick->stickY << "\n";
        }

        loop_counter++;
    }
    wf.close();
    wfstick.close();
    std::cout << "\nRibbonforcer Complete.\n";
    return 0;
}
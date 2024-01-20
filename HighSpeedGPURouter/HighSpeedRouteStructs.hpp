#pragma once
#include "../Common/CommonBruteforcerStructs.hpp"

namespace BITFS {

    struct MotionData13 {
        float nextPos[3];
        float nextVel;
        int camAngle;
        int facingAngle;
        int stickX;
        int stickY;
        int waitingFrames;
    };

    struct MotionData2 {
        float nextPos[3];
        float nextVel;
        int camAngle;
        int facingAngle;
        int stickX;
        int stickY;
        int waitingFrames;
        DonutData donut;
    };

    struct MotionData4 {
        int identifier;
        int camAngle;
        int facingAngle;
        int stickX;
        int stickY;
    };

    struct PositionLog {
        float posCam[3];
        float pos21[3];
        float pos22[3];
        float pos23[3];
        float pos24[3];
    };

    struct VelocityLog {
        float vel21;
        float vel22;
        float vel23;
        float vel24;
    };

    struct StickLog {
        int stick21X;
        int stick21Y;
        int stick22X;
        int stick22Y;
        int stick23X;
        int stick23Y;
        int stick24X;
        int stick24Y;
    };

    struct AngleLog {
        int cam21;
        int facing21;
        int cam22;
        int facing22;
        int cam23;
        int facing23;
        int cam24;
        int facing24;
    };

    struct WaitingLog {
        int waiting21;
        int waiting22;
        int waiting23;
    };

    struct TargetLog {
        float minSpeed;
        float posBully[3];
        float bullySpeed;
        short bullyMovingYaw;
        float posCam[3];
    };

    struct BullyData {
        float posBully[3];
        int angle;
        float velBully;
    };

    struct AllData {
        PositionLog positions;
        VelocityLog velocities;
        StickLog sticks;
        AngleLog angles;
        WaitingLog waits;
        BullyData bully;
    };

}
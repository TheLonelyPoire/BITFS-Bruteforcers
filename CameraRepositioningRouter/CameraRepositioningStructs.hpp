#pragma once
#include "../Common/CommonBruteforcerStructs.hpp"

namespace BITFS {

    struct PositionLog {
        float posCam1[3];
        float posCam2[3];
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
    };

    struct AngleLog {
        int cam21;
        int facing21;
        int cam22;
        int facing22;
        int cam23;
        int facing23;
    };

    struct TargetLog {
        float point1[2];
        float point2[2];
        float point3[2];
        float point4[2];
        float benchmark;
    };

    struct AllData {
        PositionLog positions;
        VelocityLog velocities;
        StickLog sticks;
        AngleLog angles;
        DonutData donuts;
        TargetLog targets;
    };

}

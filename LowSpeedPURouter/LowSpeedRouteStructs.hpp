#pragma once
#include "../Common/CommonBruteforcerStructs.hpp"

namespace BITFS {

    struct PositionLog {
        float posCam1[3];
        float pos11[3];
        float pos12[3];
        float pos13[3];
        float pos14[3];
        float posPole[3];
    };

    struct VelocityLog {
        float vel11;
        float vel12;
        float vel13;
        float vel14Arrive;
        float vel14;
        float poleVelX;
        float poleVelZ;
    };

    struct StickLog {
        int stick11X;
        int stick11Y;
        int stick12X;
        int stick12Y;
        int stick13X;
        int stick13Y;
        int stick14X;
        int stick14Y;
    };

    struct AngleLog {
        int cam11;
        int facing11;
        int cam12;
        int facing12;
        int cam13;
        int facing13;
        int cam14;
        int facing14;
        int slidePole;
    };

    struct WaitingLog {
        int waiting11;
        int waiting12;
        int waiting13;
        int delayFrames;
    };

    struct TargetLog {
        int platKey;
        int inPUX;
        int inPUZ;
        int speed;
        int hau;
    };

    struct AllData {
        PositionLog positions;
        VelocityLog velocities;
        StickLog sticks;
        AngleLog angles;
        WaitingLog waits;
        DonutData donuts;
        TargetLog targets;
    };

}
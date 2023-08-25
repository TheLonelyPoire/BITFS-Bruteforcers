#pragma once

namespace BITFS {

    struct StrainInfo {
        int index;
        float speed;
        float vX;
        float vZ;
    };

    struct PositionLog {
        float posCam1[3];
        float posPole[3];
        float posTenK2Air[3];
        float posTenK2[3];
        float pos21[3];
    };

    struct VelocityLog {
        float poleVelX;
        float poleVelZ;
        float velTenK2X;
        float velTenK2Z;
        float velTenK2;
        float vel21;
    };

    struct StickLog {
        int stickPoleX;
        int stickPoleY;
        int stickStrainX;
        int stickStrainY;
        int stickTenK2X;
        int stickTenK2Y;
    };

    struct AngleLog {
        int slidePole;
        int camPole;
        int camTenK2;
        int facingTenK2;
        int slideTenK2;
    };

    struct TargetLog {
        int platKey;
        int inPUX;
        int inPUZ;
        int speed;
        int hau;
        int i;
        int storedAngle;
        int cam;
    };

    struct AllData {
        PositionLog positions;
        VelocityLog velocities;
        StickLog sticks;
        AngleLog angles;
        TargetLog targets;
    };

}
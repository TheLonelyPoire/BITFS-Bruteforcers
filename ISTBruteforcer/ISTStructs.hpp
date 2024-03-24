#pragma once

namespace BITFS {

    struct PhaseOneInfo {
        float startpos[3];
        int vel;
        int hau;
    };

    struct PhaseTwoInfo {
        float f1pos[3];
        float tenkpos[3];
        float tenkvelin;
        int camyaw;
        int id;
    };

    struct PhaseThreeInfo {
        float landpos[3];
        int pux;
        int puz;
        float landvel;
        int platid;
        int stickX;
        int stickY;
        int id;
    };

    struct BullyData {
        float posBully[3];
        int angle;
        float velBully;
    };

}
#pragma once

namespace BITFS {

    struct ApproachData {
        float posArrive[3];
        float velArrive;
        int facingArrive;
        int solutionID;
    };

    struct BullyData {
        float posBully[3];
        int angle;
        float velBully;
    };

    struct SecondaryData {
        int tag;
        int frames;
    };

    struct ImpactData {
        BullyData bully;
        int routeID;
        int frames;
    };

}

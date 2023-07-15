#pragma once

namespace BITFS {

    struct SlideInfo {
        int endAngle;
        float endSpeed;
        float endPos[3];
    };

    struct FancySlideInfo {
        int endFacingAngle;
        int endSlidingAngle;
        float endSpeed;
        float endPos[3];
    };

    struct AirInfo {
        float endSpeed;
        float endPos[3];
    };

    struct StickTableData {
        int stickX;
        int stickY;
        float magnitude;
        int angle;
    };

    struct DonutData {
        float overlapArea;
        int hauBand[2][2];
    };

}
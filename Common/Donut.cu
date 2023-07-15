#include "Donut.cuh"

#include "math.h"

#include "Trig.cuh"
#include "VMath.cuh"

namespace BITFS {

    //this takes two circles, their radii and distance, and computes the area of their overlap.
    __host__ __device__ float circle_compute(float sRad, float tRad, float dis) {

        // first order of business is the degenerate triangle inequality cases. For the first one, if the other circle is big enough
        // then it'll just engulf the starting circle, so the area of overlap is the starting circle.
        if (tRad >= dis + sRad) {
            return M_PI * sRad * sRad;
        }
        // if starting circle is big enough to engulf the other circle, other circle sets the area.
        else if (sRad >= dis + tRad) {
            return M_PI * tRad * tRad;
        }
        // and this is for the circles not overlapping. 0 overlap area.
        else if (dis >= sRad + tRad) {
            return 0.0f;
        }
        // as for the usual partial-overlap case, we use a formula from wikipedia, and solve for the subtended angles via law of cosines.
        else {
            float tRadAngle = acos(lawofCosines(sRad, dis, tRad));
            float sRadAngle = acos(lawofCosines(tRad, dis, sRad));
            return (sRad * sRad * (sRadAngle - 0.5f * sin(2.0f * sRadAngle)) + tRad * tRad * (tRadAngle - 0.5f * sin(2.0f * tRadAngle)));
        }
    }


    //this is a black magic one. Given two donuts, find the angles where they overlap and the area of their overlap.
    //input is the four coordinates of the donut centers and the four donut radii.
    __host__ __device__ DonutData donut_compute(float* marioPos, float sRadInner, float sRadOuter, float* targetPos, float tRadInner, float tRadOuter) {

        float dis = find_dis(marioPos, targetPos);
        //displacement angles are like, the angles where the donuts would overlap if they were on a flat line.
        // It's really just a gigantic proof-by-cases from playing around with desmos and the law of cosines.
        // the rough idea is that you can go "I have a triangle where two of the sides have wiggle room and one is fixed and I want 
        //to maximize/minimize the angle. This is equivalent to minimizing/maximizing the cosine of the angle. Drawing out the law of 
        // cosines, minimizing the cosine is done by making the opposing radius as large as possible, and maximizing it is done by making 
        // the opposing radius as small as possible. So that's why tRadInner shows up for the upper cosines and tRadOuter for the lower
        // cosines. Then you realize that, if the radius of the opposing side is above the distance, the cosine function is monotonic in
        // radius of your side, so maximizing cosine involves picking the maximum distance on your side ie the outer radius
        // and minimizing cosine involves picking the minimum distance on your side ie the inner radius.
        // By contrast, if the radius of the opposing side is below the distance, you get a convex function. Maximizing over an interval
        // for a convex function is simple, just pick the two extreme values, compute law of cosines, pick the bigger one.
        // Minimizing, however, depends on whether the interval is on one side of, or contains the true minimum, so there's a triple
        // proof by cases there.
        float upperCosine;
        float lowerCosine;
        if (tRadInner >= dis) {
            upperCosine = intervalClip(-1.0f, 1.0f, lawofCosines(tRadInner, dis, sRadOuter));
        }
        else if (tRadInner < dis) {
            upperCosine = intervalClip(-1.0f, 1.0f, fmaxf(lawofCosines(tRadInner, dis, sRadOuter), lawofCosines(tRadInner, dis, sRadInner)));
        }
        // now to set the lower cosine value, which is a much nastier proof-by-cases. The first if is the monotonic case
        // the second and third ifs are where our interval  for our side length is on either side of the true minimum cosine value
        // and the fourth is where the interval contains the minimum, which is attained at the side length that makes a right triangle.
        if (tRadOuter >= dis) {
            lowerCosine = intervalClip(-1.0f, 1.0f, lawofCosines(tRadOuter, dis, sRadInner));
        }
        else if (sRadInner >= sqrtf(dis * dis - tRadOuter * tRadOuter)) {
            lowerCosine = intervalClip(-1.0f, 1.0f, lawofCosines(tRadOuter, dis, sRadInner));
        }
        else if (sRadOuter <= sqrtf(dis * dis - tRadOuter * tRadOuter)) {
            lowerCosine = intervalClip(-1.0f, 1.0f, lawofCosines(tRadOuter, dis, sRadOuter));
        }
        else {
            lowerCosine = intervalClip(-1.0f, 1.0f, lawofCosines(tRadOuter, dis, sqrtf(dis * dis - tRadOuter * tRadOuter)));
        }
        // now, a higher cosine actually corresponds to a lower angle!
        float displacementAngleUpper = acos(lowerCosine);
        float displacementAngleLower = acos(upperCosine);
        // this is converting the displacement angles to AU's
        int displacementAUAngleUpper = atan2s(cos(displacementAngleUpper), sin(displacementAngleUpper));
        int displacementAUAngleLower = atan2s(cos(displacementAngleLower), sin(displacementAngleLower));
        // this is finding the line from the target donut center to the source donut center, as that's the general way Mario will be facing.
        int centerLine = atan2s(marioPos[2] - targetPos[2], marioPos[0] - targetPos[0]);
        // now we take the center line, and add or subtract the upper and lower displacement angles. 32 HAU's are added on as wiggle room
        // and then the result in AU's is converted to HAU's.
        int hauAngleUpperHi = (centerLine + displacementAUAngleUpper + 32 * 16) / 16;
        int hauAngleUpperLo = (centerLine + displacementAUAngleLower - 32 * 16) / 16;
        int hauAngleLowerLo = (centerLine - displacementAUAngleUpper - 32 * 16) / 16;
        int hauAngleLowerHi = (centerLine - displacementAUAngleLower + 32 * 16) / 16;

        struct DonutData result;
        result.hauBand[0][0] = hauAngleUpperLo;
        result.hauBand[0][1] = hauAngleUpperHi;
        result.hauBand[1][0] = hauAngleLowerLo;
        result.hauBand[1][1] = hauAngleLowerHi;
        // the area of overlap can be computed from the area of overlap for the various inner and outer circles, just draw it out
        result.overlapArea = circle_compute(sRadOuter, tRadOuter, dis) + (-circle_compute(sRadInner, tRadOuter, dis)) + (-circle_compute(sRadOuter, tRadInner, dis)) + circle_compute(sRadInner, tRadInner, dis);
        return result;
    }


}
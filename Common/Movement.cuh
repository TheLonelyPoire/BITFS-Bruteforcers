#pragma once

#include "cuda.h"
#include "cuda_runtime.h"

#include "CommonBruteforcerStructs.hpp"


namespace BITFS {

	// computes 1QF of crouchslide, given stick position, starting position, starting speed, starting facing angle, camera angle.
	// might be inaccurate, takes some shortcuts.
	// note that it takes the floor normal as a parameter, this is so we don't have to keep recomputing that shit.
	__host__ __device__ SlideInfo crude_sim_slide(StickTableData stick, float* startPos, float startSpeed, int startAngle, int camera, float* slope);

	__host__ __device__ bool sim_slide(StickTableData stick, float* startPos, float forward_speed, float vX, float vZ, int faceAngle, int slideYaw, int camera, bool usePolePlatform, FancySlideInfo& output);

	__host__ __device__ bool sim_airstep(float* initialPos, float initialSpeed, int initialAngle, bool first, AirInfo& output);

	// After a backwards 1QF crouchslide from start to end, will you end up at the target HAU?
	__host__ __device__ bool angle_match(float* startPos, float* endPos, int targetHau);

	__host__ __device__ float speed_burn(float speed, int frames);

}
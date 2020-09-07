#pragma once
#include <inttypes.h>

struct CollisionInfo {
	CollisionInfo(uint16_t collID, double mag);
	uint16_t otherBodyID;
	double collisionMagnitude;
};
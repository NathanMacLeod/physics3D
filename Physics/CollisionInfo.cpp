#include "CollisionInfo.h"

CollisionInfo::CollisionInfo(uint16_t collID, double mag) {
	this->otherBodyID = collID;
	this->collisionMagnitude = mag;
}
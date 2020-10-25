#pragma once
#include "PhysicsObject.h"
#include "Expireable.h"
#include "CooldownTimer.h"

class Debris : public PhysicsObject, public Expireable {
private:
	CooldownTimer life = CooldownTimer(25);
public:
	void performDeathActions(SpaceMinerGame* game);
	Debris(RigidBody* body, PolyModel* model);
	bool isExpired();
};
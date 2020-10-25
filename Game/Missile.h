#pragma once
#include "Projectile.h"
#include "MovingObject.h"
#include "CooldownTimer.h"

class Missile : public MovingObject, public Projectile {
private:
	CooldownTimer life = CooldownTimer(10);
	bool playerMissile;
	Vector3D targetPrevVel;
	double maxAngle = cos(3.14159 / 2.0);
	PhysicsObject* target;
	uint16_t targetID;
	bool collided;
	double liveTime = 0;
	double maxTurnT = 4;

	void aimAtTarget(float fElapsedTime);
public:
	void performDeathActions(SpaceMinerGame* game);
	Missile(Vector3D pos, Rotor orientation, Vector3D velocity, PhysicsObject* target, uint16_t sourceID, bool playerMissile);
	void giveTarget(PhysicsObject* target);
	void update(SpaceMinerGame* game, float fElapsedTime);
	RigidBody* getBody();
	void draw(PixelEngine3D* g, Vector3D cameraPos, Rotor cameraDir, double FOV);
	void dampenMotion(double timePassed);
	bool isExpired();
};
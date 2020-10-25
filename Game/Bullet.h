#pragma once
#include "MovingObject.h"
#include "Projectile.h"
#include "CooldownTimer.h"

class Bullet : public PhysicsObject, public Projectile {
private:
	CooldownTimer life = CooldownTimer(2.0);
	bool collided = false;
	olc::Pixel color;
	olc::Pixel lineColor;
public:
	void performDeathActions(SpaceMinerGame* game);
	Bullet(Vector3D position, Rotor dir, Vector3D velocity, double density, uint16_t sourceID, olc::Pixel lineColor, olc::Pixel color, double lifeTime = 2.0);
	~Bullet();
	void update(SpaceMinerGame* game, float fElapsedTime);
	bool isExpired();
	void draw(PixelEngine3D* g, Vector3D cameraPos, Rotor cameraDir, double FOV);
	RigidBody* getBody();
};
#pragma once
#include "Drawable.h"
#include "Updatable.h"
#include "Expireable.h"
#include "../Physics/Rigidbody.h"

class Projectile : public Drawable, public Updatable, public Expireable {
public:
	virtual ~Projectile();
	virtual void update(SpaceMinerGame* game, float fElapsedTime) = 0;
	virtual void draw(PixelEngine3D* g, Vector3D cameraPos, Rotor cameraDir, double FOV) = 0;
	virtual RigidBody* getBody() = 0;

	static Vector3D calculateLeadPoint(Vector3D shooterPos, Vector3D targetPos, Vector3D shooterVel, Vector3D targetVel, double bulletVel, bool guessAccel = false, double distOffset = 0, Vector3D targetPrevVel = Vector3D(0, 0, 0), float fElapsedTime = 1);
};
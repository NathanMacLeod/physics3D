#pragma once
#include "../OLCEngine/PixelEngine3D.h"
#include "PhysicsObject.h"
#include "../Physics/PhysicsEngine.h"

class SpaceMinerGame : public PixelEngine3D {
private:
	std::vector<PhysicsObject*> objects;

	PhysicsEngine pEngine = PhysicsEngine(0.01667);
	Vector3D cameraPos;
	Rotor cameraOrientation;

	void readInput(float fElapsedTime);
	void gameUpdate(float fElapsedTime);
	void gameRender();

	bool OnUserCreate();
	bool OnUserUpdate(float fElapsedTime);
};
#pragma once
#include "../OLCEngine/PixelEngine3D.h"
#include "PhysicsObject.h"
#include "../Physics/PhysicsEngine.h"

class SpaceMinerGame : public PixelEngine3D {
private:
	std::vector<PhysicsObject*> objects;

	PhysicsEngine pEngine = PhysicsEngine(0.00167);
	Vector3D cameraPos;
	Rotor cameraOrientation;

	void readInput(float fElapsedTime);
	void gameUpdate(float fElapsedTime);
	void gameRender();

	bool OnUserCreate();
	bool OnUserUpdate(float fElapsedTime);
	void createShipMesh(bool playerShip, double size, olc::Pixel color, olc::Pixel lineColor, olc::Pixel highlight,
		olc::Pixel highlightLine, olc::Pixel cockpit, olc::Pixel cockpitLine, RigidBody** bodyOut, PolyModel** meshOut);
};
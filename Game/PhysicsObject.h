#pragma once
#include "PolyModel.h"
#include "../Physics/RigidBody.h"
#include "../Physics/PhysicsEngine.h";
#include "Drawable.h"

class PhysicsObject : public Drawable {
protected: //reset to protected
	PolyModel* model;
	RigidBody* body;  

public:
	virtual ~PhysicsObject();
	PhysicsObject(const PhysicsObject& object);
	PhysicsObject();
	PhysicsObject(PolyModel* model, RigidBody* body);
	void draw(PixelEngine3D* g, Vector3D cameraPos, Rotor cameraDir, double FOV);
	void debugDraw(PixelEngine3D* g, Vector3D cameraPos, Rotor cameraDir, double FOV);
	Vector3D getDir();
	Vector3D getPos();
	RigidBody* getRigidBody();
	PolyModel* getModel();
	static void createSurfacesFromPolygons(std::vector<Polygon3D>* polygons, std::vector<RigidSurface*>* out);
};
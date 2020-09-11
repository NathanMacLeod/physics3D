#pragma once
#include "Polygon3D.h"
#include "../Math/Rotor.h"
#include "../Math/Vector3D.h"
#include <vector>

class PolyModel {
private:
	std::vector<Polygon3D> polygonsOG;
	std::vector<Polygon3D> worldPolygons;
	double radius;
	Vector3D position;
	Rotor orientation;
public:
	PolyModel(std::vector<Polygon3D>* polygons, Vector3D center);
	PolyModel();

	void rotate(Rotor rotation);
	void setOrientation(Rotor orientation);
	void translate(Vector3D translation);
	void setPosition(Vector3D pos);
	void setPosAndOrientation(Rotor orientation, Vector3D pos);
	Vector3D getPos();
	Rotor getOrientation();
	std::vector<Polygon3D>* getPolygons();
	bool outOfView(Rotor cameraDir, Vector3D cameraPos, double fov, int screenWidth, int screenHeight);
};
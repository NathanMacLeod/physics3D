#pragma once
#include "../OLCEngine/Polygon3D.h"
#include "../OLCEngine/PixelEngine3D.h"
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
	~PolyModel();
	PolyModel();

	void rotate(Rotor rotation);
	void setOrientation(Rotor orientation);
	void translate(Vector3D translation);
	void setPosition(Vector3D pos);
	void setPosAndOrientation(Rotor orientation, Vector3D pos);
	Vector3D getPos();
	Rotor getOrientation();
	bool outOfView(Vector3D cameraPos, Rotor cameraDir, double fov, int screenWidth, int screenHeight);
	void draw(PixelEngine3D* g, Vector3D cameraPos, Rotor cameraDir, double FOV);
};
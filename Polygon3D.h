#ifndef POLYGON3D_H
#define POLYGON3D_H

#include "olcPixelGameEngine.h"
#include "Vector3D.h"

class Polygon3D {
public:
	Vector3D* p1;
	Vector3D* p2;
	Vector3D* p3;
	Vector3D* tempP1;
	Vector3D* tempP2;
	Vector3D* tempP3;
	olc::Pixel p1p2Color;
	olc::Pixel p2p3Color;
	olc::Pixel p3p1Color;
	olc::Pixel color;
	Polygon3D(Vector3D* p1, Vector3D* p2, Vector3D* p3, olc::Pixel lineColor, olc::Pixel color);
	~Polygon3D();
	void copyPointsToTemp();
};

#endif
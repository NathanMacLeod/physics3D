#ifndef POLYGON3D_H
#define POLYGON3D_H

#include "olcPixelGameEngine.h"
#include "../Math/Vector3D.h"

class Polygon3D {
public:
	Vector3D* p1;
	Vector3D* p2;
	Vector3D* p3;
	olc::Pixel lineColor;
	olc::Pixel color;
	Polygon3D(Vector3D* p1, Vector3D* p2, Vector3D* p3, olc::Pixel lineColor, olc::Pixel color);
};

#endif
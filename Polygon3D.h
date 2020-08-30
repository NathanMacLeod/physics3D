#ifndef POLYGON3D_H
#define POLYGON3D_H

#include "Color.h"
#include "Vector3D.h"

class Polygon3D {
public:
	Vector3D* p1;
	Vector3D* p2;
	Vector3D* p3;
	Vector3D* tempP1;
	Vector3D* tempP2;
	Vector3D* tempP3;
	Color p1p2Color;
	Color p2p3Color;
	Color p3p1Color;
	Color color;
	Polygon3D(Vector3D* p1, Vector3D* p2, Vector3D* p3, Color lineColor, Color color);
	~Polygon3D();
	void copyPointsToTemp();
};

#endif
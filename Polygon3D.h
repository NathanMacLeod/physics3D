#ifndef POLYGON3D_H
#define POLYGON3D_H

#include "Point.h"
#include "Color.h"
#include "Vector3D.h"

class Polygon3D {
public:
	Point3D* p1;
	Point3D* p2;
	Point3D* p3;
	Point3D* tempP1;
	Point3D* tempP2;
	Point3D* tempP3;
	Color p1p2Color;
	Color p2p3Color;
	Color p3p1Color;
	Color color;
	Polygon3D(Point3D* p1, Point3D* p2, Point3D* p3, Color lineColor, Color color);
	~Polygon3D();
	void copyPointsToTemp();
};

#endif
#pragma once
#ifndef RIGIDSURFACE_H
#define RIGIDSURFACE_H

#include <vector>
#include "Point.h"
#include "Vector3D.h"

class RigidSurface {
	std::vector<Point3D*> points;
	std::vector<double> inverseSegmentMagnitudes;
	Point3D* normalVectorPoint; //sits over p1 to define normal vector

	void caclulateInverseSegmentMagnitudes();
public:
	RigidSurface(const std::vector<Point3D*>& points, const Vector3D normalVector);
	RigidSurface(const RigidSurface& surface);
	std::vector<Point3D*>* getPoints();
	Point3D* getNormalVectorPoint();
	double getInverseSegmentMagnitude(int pointIndex); //returns magntiude of line from point i to point i + 1 in points
};

#endif
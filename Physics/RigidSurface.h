#pragma once
#ifndef RIGIDSURFACE_H
#define RIGIDSURFACE_H

#include <vector>
#include "../Math/Vector3D.h"

class RigidSurface {
	std::vector<Vector3D*> points;
	std::vector<double> inverseSegmentMagnitudes;
	double nVInverseMag;
	bool interiorSurface;
	Vector3D getNCross();

	void caclulateInverseSegmentMagnitudes();
	void reverseOrder();
public:

	RigidSurface(const std::vector<Vector3D>* points, const Vector3D normalVector, bool interiorSurface = false);
	RigidSurface(const RigidSurface& surface);
	std::vector<Vector3D*>* getPoints();
	Vector3D getUnitNorm();
	double getInverseSegmentMagnitude(int pointIndex); //returns magntiude of line from point i to point i + 1 in points
	bool isInteriorSurface();
};

#endif
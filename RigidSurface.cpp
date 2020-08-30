#include "RigidSurface.h"

RigidSurface::RigidSurface(const std::vector<Vector3D*>& points, const Vector3D normalVector) {
	//normalVector should have unit length
	this->points = points;
	Vector3D cross = Vector3D(*points.at(0), *points.at(1)).crossProduct(Vector3D(*points.at(1), *points.at(2)));
	nVInverseMag = ((cross.dotProduct(normalVector) < 0) ? -1 : 1) / cross.getMagnitude();
	caclulateInverseSegmentMagnitudes();
}

RigidSurface::RigidSurface(const RigidSurface& surface) {
	for (Vector3D* p : surface.points) {
		points.push_back(new Vector3D(p->x, p->y, p->z));
	}
	nVInverseMag = surface.nVInverseMag;
	caclulateInverseSegmentMagnitudes();
}

void RigidSurface::caclulateInverseSegmentMagnitudes() {
	for (int i = 0; i < points.size(); i++) {
		Vector3D* p1 = points.at(i);
		Vector3D* p2 = (i == points.size() - 1) ? points.at(0) : points.at(i + 1);
		Vector3D p1p2(*p1, *p2);
		inverseSegmentMagnitudes.push_back(1.0/p1p2.getMagnitude());
	}
}

double RigidSurface::getInverseSegmentMagnitude(int i) {
	if (i < inverseSegmentMagnitudes.size())
		return inverseSegmentMagnitudes.at(i);
	else
		return -1;
}

Vector3D RigidSurface::getUnitNorm() {
	return Vector3D(*points.at(0), *points.at(1)).crossProduct(Vector3D(*points.at(1), *points.at(2)))
		.multiply(nVInverseMag);
}

std::vector<Vector3D*>* RigidSurface::getPoints() {
	return &points;
}
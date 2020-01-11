#include "RigidSurface.h"

RigidSurface::RigidSurface(const std::vector<Point3D*>& points, const Vector3D normalVector) {
	//normalVector should have unit length
	this->points = points;
	Point3D* p1 = points.at(0);
	normalVectorPoint = new Point3D(p1->x + normalVector.x, p1->y + normalVector.y, p1->z + normalVector.z);
	caclulateInverseSegmentMagnitudes();
}

RigidSurface::RigidSurface(const RigidSurface& surface) {
	for (Point3D* p : surface.points) {
		points.push_back(new Point3D(p->x, p->y, p->z));
	}
	normalVectorPoint = new Point3D(surface.normalVectorPoint->x, surface.normalVectorPoint->y, surface.normalVectorPoint->z);
	caclulateInverseSegmentMagnitudes();
}

void RigidSurface::caclulateInverseSegmentMagnitudes() {
	for (int i = 0; i < points.size(); i++) {
		Point3D* p1 = points.at(i);
		Point3D* p2 = (i == points.size() - 1) ? points.at(0) : points.at(i + 1);
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

std::vector<Point3D*>* RigidSurface::getPoints() {
	return &points;
}

Point3D* RigidSurface::getNormalVectorPoint() {
	return normalVectorPoint;
}
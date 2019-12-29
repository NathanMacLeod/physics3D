#include "RigidSurface.h"

RigidSurface::RigidSurface(const std::vector<Point3D*>& points, const Vector3D normalVector) {
	//normalVector should have unit length
	this->points = points;
	Point3D* p1 = points.at(0);
	normalVectorPoint = new Point3D(p1->x + normalVector.x, p1->y + normalVector.y, p1->z + normalVector.z);
}

RigidSurface::RigidSurface(const RigidSurface& surface) {
	for (Point3D* p : surface.points) {
		points.push_back(new Point3D(p->x, p->y, p->z));
	}
	normalVectorPoint = new Point3D(surface.normalVectorPoint->x, surface.normalVectorPoint->y, surface.normalVectorPoint->z);
}

std::vector<Point3D*>* RigidSurface::getPoints() {
	return &points;
}

Point3D* RigidSurface::getNormalVectorPoint() {
	return normalVectorPoint;
}

#ifndef POINT_H
#define POINT_H

class Point3D {
public:
	double x;
	double y;
	double z;

	Point3D() {
		x = 0;
		y = 0;
		z = 0;
	}

	Point3D(double x, double y, double z=0) {
		this->x = x;
		this->y = y;
		this->z = z;
	}
};

#endif
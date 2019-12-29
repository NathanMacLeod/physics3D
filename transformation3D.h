#pragma once
#ifndef TRANSFORMATION3D_H
#define TRANSFORMATION3D_H

#include <vector>
#include "Point.h"
#include "Vector3D.h"
#include <iostream>

class transformation3D {
public:
	static void translatePoints(std::vector<Point3D*>* points, const Vector3D translation) {
		translatePoints(points, translation.x, translation.y, translation.z);
	}
	static void translatePoint(Point3D* p, const Vector3D translation) {
		translatePoint(p, translation.x, translation.y, translation.z);
	}

	static void translatePoint(Point3D* p, double x, double y, double z) {
		p->x += x;
		p->y += y;
		p->z += z;
	}

	static void translatePoints(std::vector<Point3D*>* points, double x, double y, double z) {
		for (Point3D* p : *points) {
			translatePoint(p, x, y, z);
		}
	}

	static void rotatePointAroundZParallelAxis(Point3D* p, double theta, double xCoord, double yCoord, double cosTheta = -1, double sinTheta = -1) {
		if (cosTheta == sinTheta) {
			cosTheta = cos(theta);
			sinTheta = sin(theta);
		}
		//x' = xcos(a) - ysin(a)
		//y' = xsin(a) + ycos(a)
		double relativeX = p->x - xCoord;
		double relativeY = p->y - yCoord;
		p->x = xCoord + relativeX * cosTheta - relativeY * sinTheta;
		p->y = yCoord + relativeX * sinTheta + relativeY * cosTheta;
		
	}

	static void rotatePointsAroundZParallelAxis(std::vector<Point3D*>* points, double theta, double xCoord, double yCoord, double cosTheta=-1, double sinTheta=-1) {
		if (cosTheta == sinTheta) {
			cosTheta = cos(theta);
			sinTheta = sin(theta);
		}
		for (Point3D* p : *points) {
			rotatePointAroundZParallelAxis(p, theta, xCoord, yCoord, cosTheta, sinTheta);
		}
	}

	static void rotatePointAroundXParralelAxis(Point3D* p, double theta, double yCoord, double zCoord, double cosTheta = -1, double sinTheta = -1) {
		if (cosTheta == sinTheta) {
			cosTheta = cos(theta);
			sinTheta = sin(theta);
		}
		//x' = xcos(a) - ysin(a)
		//y' = xsin(a) + ycos(a)
		double relativeY = p->y - yCoord;
		double relativeZ = p->z - zCoord;
		p->y = yCoord + relativeY * cosTheta - relativeZ * sinTheta;
		p->z = zCoord + relativeY * sinTheta + relativeZ * cosTheta;
	}

	static void rotatePointsAroundXParralelAxis(std::vector<Point3D*>* points, double theta, double yCoord, double zCoord, double cosTheta = -1, double sinTheta = -1) {
		if(cosTheta == sinTheta) {
			cosTheta = cos(theta);
			sinTheta = sin(theta);
		}
		for (Point3D* p : *points) {
			rotatePointAroundXParralelAxis(p, theta, yCoord, zCoord, cosTheta, sinTheta);
		}
	}

	static void rotatePointAroundYParralelAxis(Point3D* p, double theta, double xCoord, double zCoord, double cosTheta = -1, double sinTheta = -1) {
		if (cosTheta == sinTheta) {
			cosTheta = cos(theta);
			sinTheta = sin(theta);
		}
		//x' = xcos(a) - ysin(a)
		//y' = xsin(a) + ycos(a)
		double relativeX = p->x - xCoord;
		double relativeZ = p->z - zCoord;
		p->x = xCoord + relativeX * cosTheta + relativeZ * sinTheta;
		p->z = zCoord - relativeX * sinTheta + relativeZ * cosTheta;
	}

	static void rotatePointsAroundYParralelAxis(std::vector<Point3D*>* points, double theta, double xCoord, double zCoord, double cosTheta=-1, double sinTheta=-1) {
		if (cosTheta == sinTheta) {
			cosTheta = cos(theta);
			sinTheta = sin(theta);
		}
		for (Point3D* p : *points) {
			rotatePointAroundYParralelAxis(p, theta, xCoord, zCoord, cosTheta, sinTheta);
		}
	}

	static void rotatePointAroundArbitraryAxis(Point3D* p, Point3D* output, const Vector3D axisUnitVector, double xPos, double yPos, double zPos, double theta, double cosTheta=-1, double sinTheta=-1) {
		if (cosTheta == sinTheta) {
			cosTheta = cos(theta);
			sinTheta = sin(theta);
		}
		//std::cout << "axisVector x:" << axisUnitVector.x << " y:" << axisUnitVector.y << " z:" << axisUnitVector.z << "\n";
		Vector3D pointVector(p->x - xPos, p->y - yPos, p->z - zPos);
		//std::cout << "PointVector x:" << (double) pointVector.x << " y:" << (double) pointVector.y << " z:" << (double) pointVector.z << "\n";
		Vector3D vParralel;
		axisUnitVector.multiply(pointVector.dotProduct(axisUnitVector), &vParralel);
		//std::cout << "vParralel x:" << vParralel.x << " y:" << vParralel.y << " z:" << vParralel.z << "\n";
		Vector3D vPerpendicular;
		pointVector.sub(vParralel, &vPerpendicular);
		//std::cout << "vPerpendicular x:" << vPerpendicular.x << " y:" << vPerpendicular.y << " z:" << vPerpendicular.z << "\n";
		Vector3D b;
		Vector3D* crossedVector = axisUnitVector.crossProduct(pointVector, &b);
		//std::cout << "crossVector x:" << b.x << " y:" << b.y << " z:" << b.z << "\n";
		Vector3D c;
		Vector3D* comp1 = vPerpendicular.multiply(cosTheta, &c);
		Vector3D d;
		Vector3D* comp2 = crossedVector->multiply(sinTheta, &d);
		Vector3D e;
		Vector3D* newPointVector = comp1->add(*comp2, &e);
		newPointVector->add(vParralel, &e);
		//std::cout << "newPointVector x:" << (double) e.x << " y:" << (double) e.y << " z:" << (double) e.z << "\n" << "\n";
		output->x = newPointVector->x + xPos;
		output->y = newPointVector->y + yPos;
		output->z = newPointVector->z + zPos;
	}

	static void rotatePointsAroundArbitraryAxis(std::vector<Point3D*>* points, const Vector3D axisUnitVector, double xPos, double yPos, double zPos, double theta, double cosTheta=-1, double sinTheta=-1) {
		//axis vector should be unit length
		if (cosTheta == sinTheta) {
			cosTheta = cos(theta);
			sinTheta = sin(theta);
		}
		for (Point3D* p : *points) {
			rotatePointAroundArbitraryAxis(p, p, axisUnitVector, xPos, yPos, zPos, theta, cosTheta, sinTheta);
		}
	}

};

#endif
#pragma once
#include <vector>;
#include "Vector3D.h";
#include "Point.h";
#include "RigidSurface.h"

class ConvexHull {

public:

	class ColPointInfo {
	public:
		Point3D point;
		double penDepth;

		ColPointInfo(Point3D p, double penDepth);
	};

	class Edge {
	public:
		Point3D* p1;
		Point3D* p2;
		double inverseMagnitude;

		Edge(Point3D* p1, Point3D* p2);
	};

private:
	std::vector<RigidSurface*> surfaces;
	std::vector<Point3D*> colPoints;
	std::vector<Edge*> colEdges;
	Point3D centerOfMass;
	double collisionRadius;
	double collisionRadiusSquared;
	double mass;
	double* inertiaTensor;

	void findBodyMassAndInertia(double density);
	void findCollisionRadius();
	void findColPointsEdges();
	void findMaxMin(Vector3D n, double* max, double* min, Point3D* maxP, Point3D* minP);
public:
	ConvexHull(const std::vector<RigidSurface*>& surfaces, double density);
	~ConvexHull();

	Point3D getCenterOfMass();
	std::vector<Point3D*>* getColPoints();
	double getCollisionRadius();
	double* getInertia();
	double getMass();
	bool hullsInCollisionRange(ConvexHull* hull);
	bool SATColliderDetect(ConvexHull* potCollider, std::vector<ColPointInfo>* colSupPoints, Point3D* colPoint, Vector3D* nVect, double* colDepth, bool* separatingAxis);
	bool SATEdgeCol(ConvexHull* b, Point3D* collisionPoint, Vector3D* normal, double* colDepth, bool* separatingAxis);
	bool getPointInsideBody(const Point3D point);
};
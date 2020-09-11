#pragma once
#include <vector>;
#include "../Math/Vector3D.h";
#include "RigidSurface.h"

class ConvexHull {

public:

	class ColPointInfo {
	public:
		Vector3D point;
		double penDepth;

		ColPointInfo(Vector3D p, double penDepth);
	};

	class Edge {
	public:
		Vector3D* p1;
		Vector3D* p2;
		double inverseMagnitude;

		Edge(Vector3D* p1, Vector3D* p2);
	};

private:
	std::vector<RigidSurface*> surfaces;
	std::vector<Vector3D*> colPoints;
	std::vector<Edge*> colEdges;
	Vector3D centerOfMass;
	double collisionRadius;
	double collisionRadiusSquared;
	double mass;
	double* inertiaTensor;

	void findBodyMassAndInertia(double density);
	void findCollisionRadius();
	void findColPointsEdges();
	void findMaxMin(Vector3D n, double* max, double* min, Vector3D* maxP, Vector3D* minP);
public:
	ConvexHull(const std::vector<RigidSurface*>* surfaces, double density);
	~ConvexHull();

	Vector3D getCenterOfMass();
	Vector3D* getCOMPointer();
	std::vector<Vector3D*>* getColPoints();
	double getCollisionRadius();
	double* getInertia();
	double getMass();
	bool hullsInCollisionRange(ConvexHull* hull);
	bool SATColliderDetect(ConvexHull* potCollider, std::vector<ColPointInfo>* colSupPoints, Vector3D* colPoint, Vector3D* nVect, double* colDepth, bool* separatingAxis);
	bool SATEdgeCol(ConvexHull* b, Vector3D* collisionPoint, Vector3D* normal, double* colDepth, bool* separatingAxis);
	bool getPointInsideBody(const Vector3D point);
};
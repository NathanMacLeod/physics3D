#pragma once
#ifndef RIGIDBODY_H
#define RIGIDBODY_H

#include <vector>;
#include "ConvexHull.h"

class RigidBody {

public:
	/*class ColPointInfo {
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
	};*/

private:
	/*struct SimplexFace {
		Vector3D points[3];
		Vector3D norm;
		double dist;

		SimplexFace(Vector3D a, Vector3D b, Vector3D c);
	};*/

	//std::vector<RigidSurface*> surfacesReferenceCopy;
	//std::vector<RigidSurface*> surfaces;
	std::vector<Point3D*> pointsToTransform;
	std::vector<ConvexHull*> hulls;
	//std::vector<Point3D*> colPoints;
	//std::vector<Edge*> colEdges;
	Point3D centerOfMass;
	Vector3D velocity;
	Vector3D angularVelocity;
	Point3D orientationPoint1;
	Point3D orientationPoint2;
	double collisionRadius;
	double collisionRadiusSquared;
	double mass;
	double inverseMass;
	double friction;
	double restitution;
	double* inertiaTensor;
	bool fixed;

	void findBodyMassAndInertia(double density);
	void findCollisionRadius();
	//void findColPointsEdges();
	//void createReferenceCopies();
	//bool getPointInsideBody(const Point3D point, const std::vector<Point3D*> vectorPositions, const std::vector<Vector3D*> normalVectors);
	Vector3D findVectorRelativeToBodyFrame(const Vector3D vector);
	//Vector3D solve2Simplex(Vector3D a, Vector3D b, std::vector<Vector3D>* closestFeature);
	//Vector3D solve3Simplex(Vector3D a, Vector3D b, Vector3D c, std::vector<Vector3D>* closestFeature);
	//bool iterateSimplex(std::vector<Vector3D>& currSimplex, Vector3D* nextDir, int* removeIndex, std::vector<Vector3D>* closestFeature);
	//double findInteriorDist(std::vector<Vector3D>& simplex, RigidBody& otherBody, Vector3D* collNorm, Point3D* colPoint);
	//std::vector<Point3D> getClosestFeature(Vector3D dir);
public:

	RigidBody(const std::vector<ConvexHull*>& hulls, double density, double friction, double resistution, bool fixed); //not done
	//void recalibrateFromReference();
	bool bodiesInCollisionRange(RigidBody& body);
	//Vector3D findSupportPoint(const Vector3D direction);
	//double GJK(RigidBody& otherBody, Vector3D* collNorm, Point3D* colPoint);
	//bool SATColliderDetect(RigidBody* potCollider, std::vector<ColPointInfo>* colSupPoints, Point3D* colPoint, Vector3D* nVect, double* colDepth, bool* separatingAxis);
	//bool SATEdgeCol(RigidBody* b, Point3D* collisionPoint, Vector3D* normal, double* colDepth, bool* separatingAxis);
	double getCollisionRadius() const;
	double getCollisionRadiusSquared() const;
	Vector3D getAngularVelocity();
	bool checkForColls(RigidBody* b, std::vector<ConvexHull::ColPointInfo>* colSupPoints, Point3D* collisionPoint, Vector3D* nVect, double* colDepth);
	//void findCollisionInformationAsCollider(std::vector<ColPointInfo*>* colOutputs, RigidBody& body, double timestep);
	//bool getPointInsideBody(const Point3D point);
	//std::vector<RigidSurface*>* getSurfaces();
	std::vector<ConvexHull*>* getHulls();
	Point3D getCenterOfMass();
	Vector3D getVelocityOfPointDueToAngularVelocity(const Point3D point) const;
	Vector3D getVelocityOfPoint(const Point3D point) const;
	Vector3D getVelocity();
	bool verifyCollisionPointNotExiting(const RigidBody body, const Vector3D normalVector, const Point3D point);
	void applyImpulseAtPosition(const Vector3D impulse, const Point3D position);
	double findInverseInertiaOfAxis(const Vector3D axis);
	double getRadialDistanceOfPoint(const Point3D point);
	double getMass() const;
	double getInverseMass() const;
	double getFriction() const;
	double getRestitution() const;
	void translate(const Vector3D translation);
	void acclerateLineraly(const Vector3D changeInVelocity);
	void moveInTime(double time);
	bool getFixed();
};

#endif
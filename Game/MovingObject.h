#pragma once
#include "PhysicsObject.h"

class MovingObject : public PhysicsObject {
protected:
	double rollI;
	double pitchI;
	double yawI;

	double linearDampFactor;
	double angularDampFactor;

	double pitchRate;
	double rollRate;
	double yawRate;
	double forwardThrust;
	double sideThrust;
	double verticleThrust;
	
	void findIVals();
	Vector3D accel;
public:
	virtual ~MovingObject();
	virtual void dampenMotion(double timePassed);

	Vector3D getAccel();
	void roll(double mult, double timePassed);
	void pitch(double mult, double timePassed);
	void yaw(double mult, double timePassed);
	void accelForward(double mult, double timePassed);
	void accelDrift(double mult, double timePassed);
	void accelLift(double mult, double timePassed);


	enum DebrisType { HeadDebris, WingDebris, BodyDebris };
	static int nDebrisTypes;
	static std::vector<Vector3D> shipPoints;
	static void initShipPoints();
	static void createShipMesh(bool playerShip, double size, olc::Pixel mainColor, olc::Pixel mainLine, olc::Pixel highlight, 
		olc::Pixel highlightLine, olc::Pixel cockpit, olc::Pixel cockpitLine, RigidBody** bodyOut, PolyModel** meshOut);
	static void getShipDebris(DebrisType debrisType, double size, olc::Pixel mainColor, olc::Pixel mainLine, olc::Pixel highlight,
		olc::Pixel highlightLine, olc::Pixel cockpit, olc::Pixel cockpitLine, RigidBody** bodyOut, PolyModel** meshOut);
};
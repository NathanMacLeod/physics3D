#include "MovingObject.h"
#include "stdio.h"

std::vector<Vector3D> MovingObject::shipPoints;
int MovingObject::nDebrisTypes = 2;

MovingObject::~MovingObject() {

}

void MovingObject::findIVals() {
	rollI = 1.0 / body->findInverseInertiaOfAxis(Vector3D(1, 0, 0));
	pitchI = 1.0 / body->findInverseInertiaOfAxis(Vector3D(0, 0, 1));
	yawI = 1.0 / body->findInverseInertiaOfAxis(Vector3D(0, 1, 0));
}

void MovingObject::dampenMotion(double timePassed) {
	float t = std::fmin(1, timePassed);
	Vector3D vel = body->getVelocity();
	Vector3D aVel = body->getAngularVelocity();

	body->setVelocity(vel.sub(vel.multiply(t * linearDampFactor)));
	body->setAngVelocity(aVel.sub(aVel.multiply(t * angularDampFactor)));
}

void MovingObject::roll(double mult, double timePassed) {
	Vector3D axis = body->getOrientation().rotate(Vector3D(0, 1, 0));
	Vector3D impulse = body->getOrientation().rotate(Vector3D(0, 0, mult * rollRate * timePassed * yawI / 2.0));
	body->applyImpulseAtPosition(impulse, body->getCenterOfMass().add(axis));
	body->applyImpulseAtPosition(impulse.getInverse(), body->getCenterOfMass().sub(axis));
}

void MovingObject::pitch(double mult, double timePassed) {
	Vector3D axis = body->getOrientation().rotate(Vector3D(0, 1, 0));
	Vector3D impulse = body->getOrientation().rotate(Vector3D(mult * pitchRate * timePassed * pitchI / 2.0, 0, 0));
	body->applyImpulseAtPosition(impulse, body->getCenterOfMass().add(axis));
	body->applyImpulseAtPosition(impulse.getInverse(), body->getCenterOfMass().sub(axis));
}

void MovingObject::yaw(double mult, double timePassed) {
	Vector3D axis = body->getOrientation().rotate(Vector3D(0, 0, -1));
	Vector3D impulse = body->getOrientation().rotate(Vector3D(mult * yawRate * timePassed *yawI / 2.0, 0, 0));
	body->applyImpulseAtPosition(impulse, body->getCenterOfMass().add(axis));
	body->applyImpulseAtPosition(impulse.getInverse(), body->getCenterOfMass().sub(axis));
}

void MovingObject::accelForward(double mult, double timePassed) {
	Vector3D axis = body->getOrientation().rotate(Vector3D(1, 0, 0));
	double impulse = mult * forwardThrust * timePassed * body->getMass();
	body->applyImpulseAtPosition(axis.multiply(impulse), body->getCenterOfMass());
}

void MovingObject::accelDrift(double mult, double timePassed) {
	Vector3D axis = body->getOrientation().rotate(Vector3D(0, 0, -1));
	double impulse = mult * sideThrust * timePassed * body->getMass();
	body->applyImpulseAtPosition(axis.multiply(impulse), body->getCenterOfMass());
}

void MovingObject::accelLift(double mult, double timePassed) {
	Vector3D axis = body->getOrientation().rotate(Vector3D(0, -1, 0));
	double impulse = mult * verticleThrust * timePassed * body->getMass();
	body->applyImpulseAtPosition(axis.multiply(impulse), body->getCenterOfMass());
}

void MovingObject::initShipPoints() {
	//wings
	shipPoints.push_back(Vector3D(0, 0, 0));
	shipPoints.push_back(Vector3D(1, 4, 5));
	shipPoints.push_back(Vector3D(1, 4, -5));
	shipPoints.push_back(Vector3D(8, 3, 4));
	shipPoints.push_back(Vector3D(8, 3, -4));
	shipPoints.push_back(Vector3D(4, 2, 2.5));
	shipPoints.push_back(Vector3D(4, 2, -2.5));
	shipPoints.push_back(Vector3D(4.5, 1, 1));
	shipPoints.push_back(Vector3D(4.5, 1, -1));

	//head + body
	//9
	shipPoints.push_back(Vector3D(8.5, 1.5, 0));
	shipPoints.push_back(Vector3D(6.5, 1, 1.5));
	shipPoints.push_back(Vector3D(6.5, 1, -1.5));
	shipPoints.push_back(Vector3D(4, 1, 1));
	shipPoints.push_back(Vector3D(4, 1, -1));

	//14
	shipPoints.push_back(Vector3D(7, 2, 0));
	shipPoints.push_back(Vector3D(4.25, 1.5, 0));
	shipPoints.push_back(Vector3D(8, 1, 0));
	shipPoints.push_back(Vector3D(6, 1.0 / 3, 1));
	shipPoints.push_back(Vector3D(6, 1.0 / 3, -1));
	shipPoints.push_back(Vector3D(6, -0.5, 0));
	shipPoints.push_back(Vector3D(5, 0, 0));

	//21
	shipPoints.push_back(Vector3D(5, 1, 1));
	shipPoints.push_back(Vector3D(5, 1, -1));

	//23
	shipPoints.push_back(Vector3D(2, 2, 1.75));
	shipPoints.push_back(Vector3D(2, 2, -1.75));
	shipPoints.push_back(Vector3D(-0.5, 2.5, 2));
	shipPoints.push_back(Vector3D(-0.5, 2.5, -2));
	shipPoints.push_back(Vector3D(-0.5, 2.5, 0.5));
	shipPoints.push_back(Vector3D(-0.5, 2.5, -0.5));
	shipPoints.push_back(Vector3D(-0.5, 1, 1.25));
	shipPoints.push_back(Vector3D(-0.5, 1, -1.25));

	//31
	shipPoints.push_back(Vector3D(2, 2.25, 0));
	shipPoints.push_back(Vector3D(-0.5, 1, 0.25));
	shipPoints.push_back(Vector3D(-0.5, 1, -0.25));
	shipPoints.push_back(Vector3D(1.5, 0.75, 0));
	shipPoints.push_back(Vector3D(1, 1, 1.25));
	shipPoints.push_back(Vector3D(1, 1, -1.25));

	//37
	shipPoints.push_back(Vector3D(1, 3.7, 5.3));
	shipPoints.push_back(Vector3D(1, 3.7, -5.3));
	shipPoints.push_back(Vector3D(4, 1.8, 2.7));
	shipPoints.push_back(Vector3D(4, 1.8, -2.7));
}

void MovingObject::createShipMesh(bool playerShip, double size, olc::Pixel color, olc::Pixel lineColor, olc::Pixel highlight,
	olc::Pixel highlightLine, olc::Pixel cockpit, olc::Pixel cockpitLine, RigidBody** bodyOut, PolyModel** meshOut) {

	if (shipPoints.size() == 0) {
		initShipPoints();
	}

	std::vector<Polygon3D> polygons;
	std::vector<Vector3D> points = shipPoints;
	std::vector<ConvexHull*> hulls;

	for (int i = 0; i < points.size(); i++) {
		points[i] = points[i].multiply(size);
	}

	olc::Pixel thruster = olc::DARK_RED;
	olc::Pixel thrusterLine = olc::RED;
	polygons.push_back(Polygon3D(points.at(0), points.at(8), points.at(20), lineColor, color));
	polygons.push_back(Polygon3D(points.at(0), points.at(40), points.at(8), lineColor, color));
	polygons.push_back(Polygon3D(points.at(0), points.at(38), points.at(40), lineColor, color));
	polygons.push_back(Polygon3D(points.at(40), points.at(38), points.at(4), lineColor, color));
	polygons.push_back(Polygon3D(points.at(4), points.at(6), points.at(40), lineColor, color));
	polygons.push_back(Polygon3D(points.at(40), points.at(6), points.at(8), lineColor, color));
	polygons.push_back(Polygon3D(points.at(38), points.at(2), points.at(4), lineColor, color));
	polygons.push_back(Polygon3D(points.at(0), points.at(2), points.at(38), lineColor, color));
	polygons.push_back(Polygon3D(points.at(2), points.at(6), points.at(4), lineColor, color));
	polygons.push_back(Polygon3D(points.at(6), points.at(2), points.at(0), lineColor, color));
	polygons.push_back(Polygon3D(points.at(8), points.at(6), points.at(0), lineColor, color));
	if (!playerShip) {
		polygons.push_back(Polygon3D(points.at(20), points.at(8), points.at(22), lineColor, color));
		polygons.push_back(Polygon3D(points.at(20), points.at(22), points.at(11), lineColor, color));
		polygons.push_back(Polygon3D(points.at(20), points.at(11), points.at(18), lineColor, color));
		polygons.push_back(Polygon3D(points.at(16), points.at(18), points.at(11), lineColor, color));
		polygons.push_back(Polygon3D(points.at(11), points.at(9), points.at(16), lineColor, color));
		polygons.push_back(Polygon3D(points.at(14), points.at(9), points.at(11), lineColor, color));
		polygons.push_back(Polygon3D(points.at(14), points.at(11), points.at(22), lineColor, color));
		polygons.push_back(Polygon3D(points.at(14), points.at(22), points.at(13), lineColor, color));
		polygons.push_back(Polygon3D(points.at(14), points.at(13), points.at(15), lineColor, color));
		polygons.push_back(Polygon3D(points.at(15), points.at(13), points.at(24), lineColor, color));
		polygons.push_back(Polygon3D(points.at(15), points.at(24), points.at(31), lineColor, color));
		polygons.push_back(Polygon3D(points.at(31), points.at(24), points.at(26), lineColor, color));
		polygons.push_back(Polygon3D(points.at(31), points.at(26), points.at(28), lineColor, color));
		polygons.push_back(Polygon3D(points.at(28), points.at(26), points.at(30), thrusterLine, thruster));//thruster
		polygons.push_back(Polygon3D(points.at(28), points.at(30), points.at(33), thrusterLine, thruster));//thruster
		polygons.push_back(Polygon3D(points.at(15), points.at(28), points.at(33), lineColor, color));
		polygons.push_back(Polygon3D(points.at(15), points.at(33), points.at(34), lineColor, color));
		polygons.push_back(Polygon3D(points.at(31), points.at(28), points.at(15), lineColor, color));
		polygons.push_back(Polygon3D(points.at(26), points.at(24), points.at(30), lineColor, color));
		polygons.push_back(Polygon3D(points.at(30), points.at(24), points.at(13), lineColor, color));
		polygons.push_back(Polygon3D(points.at(33), points.at(30), points.at(36), lineColor, color));
		polygons.push_back(Polygon3D(points.at(33), points.at(36), points.at(34), lineColor, color));
		polygons.push_back(Polygon3D(points.at(34), points.at(36), points.at(0), lineColor, color));


		polygons.push_back(Polygon3D(points.at(20), points.at(18), points.at(19), cockpitLine, cockpit));//cockpit
		polygons.push_back(Polygon3D(points.at(18), points.at(16), points.at(19), cockpitLine, cockpit));//cockpit
	}


	///////////////////////////////
	polygons.push_back(Polygon3D(points.at(7), points.at(0), points.at(20), lineColor, color));
	polygons.push_back(Polygon3D(points.at(39), points.at(0), points.at(7), lineColor, color));
	polygons.push_back(Polygon3D(points.at(37), points.at(0), points.at(39), lineColor, color));
	polygons.push_back(Polygon3D(points.at(37), points.at(39), points.at(3), lineColor, color));
	polygons.push_back(Polygon3D(points.at(3), points.at(39), points.at(5), lineColor, color));
	polygons.push_back(Polygon3D(points.at(5), points.at(39), points.at(7), lineColor, color));
	polygons.push_back(Polygon3D(points.at(1), points.at(37), points.at(3), lineColor, color));
	polygons.push_back(Polygon3D(points.at(1), points.at(0), points.at(37), lineColor, color));
	polygons.push_back(Polygon3D(points.at(5), points.at(1), points.at(3), lineColor, color));
	polygons.push_back(Polygon3D(points.at(1), points.at(5), points.at(0), lineColor, color));
	polygons.push_back(Polygon3D(points.at(5), points.at(7), points.at(0), lineColor, color));
	if (!playerShip) {
		polygons.push_back(Polygon3D(points.at(7), points.at(20), points.at(21), lineColor, color));
		polygons.push_back(Polygon3D(points.at(21), points.at(20), points.at(10), lineColor, color));
		polygons.push_back(Polygon3D(points.at(10), points.at(20), points.at(17), lineColor, color));
		polygons.push_back(Polygon3D(points.at(17), points.at(16), points.at(10), lineColor, color));
		polygons.push_back(Polygon3D(points.at(9), points.at(10), points.at(16), lineColor, color));
		polygons.push_back(Polygon3D(points.at(9), points.at(14), points.at(10), lineColor, color));
		polygons.push_back(Polygon3D(points.at(10), points.at(14), points.at(21), lineColor, color));
		polygons.push_back(Polygon3D(points.at(21), points.at(14), points.at(12), lineColor, color));
		polygons.push_back(Polygon3D(points.at(12), points.at(14), points.at(15), lineColor, color));
		polygons.push_back(Polygon3D(points.at(12), points.at(15), points.at(23), lineColor, color));
		polygons.push_back(Polygon3D(points.at(23), points.at(15), points.at(31), lineColor, color));
		polygons.push_back(Polygon3D(points.at(23), points.at(31), points.at(25), lineColor, color));
		polygons.push_back(Polygon3D(points.at(25), points.at(31), points.at(27), lineColor, color));
		polygons.push_back(Polygon3D(points.at(25), points.at(27), points.at(29), thrusterLine, thruster));//thruster
		polygons.push_back(Polygon3D(points.at(29), points.at(27), points.at(32), thrusterLine, thruster));//thruster
		polygons.push_back(Polygon3D(points.at(27), points.at(15), points.at(32), lineColor, color));
		polygons.push_back(Polygon3D(points.at(32), points.at(15), points.at(34), lineColor, color));
		polygons.push_back(Polygon3D(points.at(27), points.at(31), points.at(15), lineColor, color));
		polygons.push_back(Polygon3D(points.at(23), points.at(25), points.at(29), lineColor, color));
		polygons.push_back(Polygon3D(points.at(23), points.at(29), points.at(12), lineColor, color));
		polygons.push_back(Polygon3D(points.at(29), points.at(32), points.at(35), lineColor, color));
		polygons.push_back(Polygon3D(points.at(35), points.at(32), points.at(34), lineColor, color));
		polygons.push_back(Polygon3D(points.at(35), points.at(34), points.at(0), lineColor, color));


		polygons.push_back(Polygon3D(points.at(17), points.at(20), points.at(19), cockpitLine, cockpit));//cockpit
		polygons.push_back(Polygon3D(points.at(16), points.at(17), points.at(19), cockpitLine, cockpit));
	}

	//surfaces

	std::vector<RigidSurface*> wing1;

	std::vector<Vector3D> w1s1;
	w1s1.push_back(points.at(3));
	w1s1.push_back(points.at(37));
	w1s1.push_back(points.at(39));
	wing1.push_back(new RigidSurface(&w1s1, Vector3D(0, -1, 0)));

	std::vector<Vector3D> w1s2;
	w1s2.push_back(points.at(3));
	w1s2.push_back(points.at(1));
	w1s2.push_back(points.at(5));
	wing1.push_back(new RigidSurface(&w1s2, Vector3D(0, 1, 0)));

	std::vector<Vector3D> w1s3;
	w1s3.push_back(points.at(3));
	w1s3.push_back(points.at(1));
	w1s3.push_back(points.at(37));
	wing1.push_back(new RigidSurface(&w1s3, Vector3D(0, 0, 1)));

	std::vector<Vector3D> w1s4;
	w1s4.push_back(points.at(39));
	w1s4.push_back(points.at(37));
	w1s4.push_back(points.at(1));
	w1s4.push_back(points.at(5));
	wing1.push_back(new RigidSurface(&w1s4, Vector3D(-1, 0, 0), true));

	std::vector<Vector3D> w1s5;
	w1s5.push_back(points.at(3));
	w1s5.push_back(points.at(39));
	w1s5.push_back(points.at(5));
	wing1.push_back(new RigidSurface(&w1s5, Vector3D(1, 0, 0)));

	hulls.push_back(new ConvexHull(&wing1, 1));

	std::vector<RigidSurface*> wing2;

	std::vector<Vector3D> w2s1;
	w2s1.push_back(points.at(39));
	w2s1.push_back(points.at(37));
	w2s1.push_back(points.at(1));
	w2s1.push_back(points.at(5));
	wing2.push_back(new RigidSurface(&w2s1, Vector3D(1, 0, 0), true));

	std::vector<Vector3D> w2s2;
	w2s2.push_back(points.at(39));
	w2s2.push_back(points.at(37));
	w2s2.push_back(points.at(0));
	wing2.push_back(new RigidSurface(&w2s2, Vector3D(0, -1, 0)));

	std::vector<Vector3D> w2s3;
	w2s3.push_back(points.at(5));
	w2s3.push_back(points.at(39));
	w2s3.push_back(points.at(7));
	wing2.push_back(new RigidSurface(&w2s3, Vector3D(1, 0, 0)));

	std::vector<Vector3D> w2s4;
	w2s4.push_back(points.at(0));
	w2s4.push_back(points.at(39));
	w2s4.push_back(points.at(7));
	wing2.push_back(new RigidSurface(&w2s4, Vector3D(0, -1, 0)));

	std::vector<Vector3D> w2s5;
	w2s5.push_back(points.at(37));
	w2s5.push_back(points.at(1));
	w2s5.push_back(points.at(0));
	wing2.push_back(new RigidSurface(&w2s5, Vector3D(-1, 0, 0)));

	std::vector<Vector3D> w2s6;
	w2s6.push_back(points.at(5));
	w2s6.push_back(points.at(1));
	w2s6.push_back(points.at(0));
	wing2.push_back(new RigidSurface(&w2s6, Vector3D(0, 1, 0)));

	std::vector<Vector3D> w2s7;
	w2s7.push_back(points.at(0));
	w2s7.push_back(points.at(5));
	w2s7.push_back(points.at(7));
	wing2.push_back(new RigidSurface(&w2s7, Vector3D(0, 1, 0)));

	hulls.push_back(new ConvexHull(&wing2, 1));

	std::vector<RigidSurface*> wing3;

	std::vector<Vector3D> w3s1;
	w3s1.push_back(points.at(4));
	w3s1.push_back(points.at(38));
	w3s1.push_back(points.at(40));
	wing3.push_back(new RigidSurface(&w3s1, Vector3D(0, -1, 0)));

	std::vector<Vector3D> w3s2;
	w3s2.push_back(points.at(4));
	w3s2.push_back(points.at(2));
	w3s2.push_back(points.at(6));
	wing3.push_back(new RigidSurface(&w3s2, Vector3D(0, 1, 0)));

	std::vector<Vector3D> w3s3;
	w3s3.push_back(points.at(4));
	w3s3.push_back(points.at(2));
	w3s3.push_back(points.at(38));
	wing3.push_back(new RigidSurface(&w3s3, Vector3D(0, 0, -1)));

	std::vector<Vector3D> w3s4;
	w3s4.push_back(points.at(40));
	w3s4.push_back(points.at(38));
	w3s4.push_back(points.at(2));
	w3s4.push_back(points.at(6));
	wing3.push_back(new RigidSurface(&w3s4, Vector3D(-1, 0, 0), true));

	std::vector<Vector3D> w3s5;
	w3s5.push_back(points.at(4));
	w3s5.push_back(points.at(40));
	w3s5.push_back(points.at(6));
	wing3.push_back(new RigidSurface(&w3s5, Vector3D(1, 0, 0)));

	hulls.push_back(new ConvexHull(&wing3, 1));

	std::vector<RigidSurface*> wing4;

	std::vector<Vector3D> w4s1;
	w4s1.push_back(points.at(40));
	w4s1.push_back(points.at(38));
	w4s1.push_back(points.at(2));
	w4s1.push_back(points.at(6));
	wing4.push_back(new RigidSurface(&w4s1, Vector3D(1, 0, 0), true));

	std::vector<Vector3D> w4s2;
	w4s2.push_back(points.at(40));
	w4s2.push_back(points.at(38));
	w4s2.push_back(points.at(0));
	wing4.push_back(new RigidSurface(&w4s2, Vector3D(0, -1, 0)));

	std::vector<Vector3D> w4s3;
	w4s3.push_back(points.at(6));
	w4s3.push_back(points.at(40));
	w4s3.push_back(points.at(8));
	wing4.push_back(new RigidSurface(&w4s3, Vector3D(1, 0, 0)));

	std::vector<Vector3D> w4s4;
	w4s4.push_back(points.at(0));
	w4s4.push_back(points.at(40));
	w4s4.push_back(points.at(8));
	wing4.push_back(new RigidSurface(&w4s4, Vector3D(0, -1, 0)));

	std::vector<Vector3D> w4s5;
	w4s5.push_back(points.at(38));
	w4s5.push_back(points.at(2));
	w4s5.push_back(points.at(0));
	wing4.push_back(new RigidSurface(&w4s5, Vector3D(-1, 0, 0)));

	std::vector<Vector3D> w4s6;
	w4s6.push_back(points.at(6));
	w4s6.push_back(points.at(2));
	w4s6.push_back(points.at(0));
	wing4.push_back(new RigidSurface(&w4s6, Vector3D(0, 1, 0)));

	std::vector<Vector3D> w4s7;
	w4s7.push_back(points.at(0));
	w4s7.push_back(points.at(6));
	w4s7.push_back(points.at(8));
	wing4.push_back(new RigidSurface(&w4s7, Vector3D(0, 1, 0)));

	hulls.push_back(new ConvexHull(&wing4, 1));

	std::vector<RigidSurface*> bodyRear;

	std::vector<Vector3D> brs1;
	brs1.push_back(points.at(30));
	brs1.push_back(points.at(29));
	brs1.push_back(points.at(25));
	brs1.push_back(points.at(26));
	bodyRear.push_back(new RigidSurface((&brs1), Vector3D(-1, 0, 0)));

	std::vector<Vector3D> brs2;
	brs2.push_back(points.at(29));
	brs2.push_back(points.at(30));
	brs2.push_back(points.at(0));
	bodyRear.push_back(new RigidSurface((&brs2), Vector3D(-1, 0, 0)));

	std::vector<Vector3D> brs3;
	brs3.push_back(points.at(30));
	brs3.push_back(points.at(0));
	brs3.push_back(points.at(20));
	brs3.push_back(points.at(8));
	bodyRear.push_back(new RigidSurface((&brs3), Vector3D(0, -1, 0)));

	std::vector<Vector3D> brs4;
	brs4.push_back(points.at(29));
	brs4.push_back(points.at(0));
	brs4.push_back(points.at(20));
	brs4.push_back(points.at(7));
	bodyRear.push_back(new RigidSurface((&brs4), Vector3D(0, -1, 0)));

	std::vector<Vector3D> brs5;
	brs5.push_back(points.at(25));
	brs5.push_back(points.at(26));
	brs5.push_back(points.at(24));
	brs5.push_back(points.at(23));
	bodyRear.push_back(new RigidSurface((&brs5), Vector3D(0, 1, 0)));

	std::vector<Vector3D> brs6;
	brs6.push_back(points.at(23));
	brs6.push_back(points.at(24));
	brs6.push_back(points.at(15));
	bodyRear.push_back(new RigidSurface((&brs6), Vector3D(0, 1, 0)));

	std::vector<Vector3D> brs7;
	brs7.push_back(points.at(24));
	brs7.push_back(points.at(15));
	brs7.push_back(points.at(8));
	bodyRear.push_back(new RigidSurface((&brs7), Vector3D(0, 1, 0)));

	std::vector<Vector3D> brs8;
	brs8.push_back(points.at(23));
	brs8.push_back(points.at(15));
	brs8.push_back(points.at(7));
	bodyRear.push_back(new RigidSurface((&brs8), Vector3D(0, 1, 0)));

	std::vector<Vector3D> brs9;
	brs9.push_back(points.at(7));
	brs9.push_back(points.at(15));
	brs9.push_back(points.at(8));
	brs9.push_back(points.at(20));
	bodyRear.push_back(new RigidSurface((&brs9), Vector3D(1, 0, 0), !playerShip));

	std::vector<Vector3D> brs10;
	brs10.push_back(points.at(30));
	brs10.push_back(points.at(8));
	brs10.push_back(points.at(24));
	brs10.push_back(points.at(26));
	bodyRear.push_back(new RigidSurface((&brs10), Vector3D(0, 0, -1)));

	std::vector<Vector3D> brs11;
	brs11.push_back(points.at(29));
	brs11.push_back(points.at(7));
	brs11.push_back(points.at(23));
	brs11.push_back(points.at(25));
	bodyRear.push_back(new RigidSurface((&brs11), Vector3D(0, 0, 1)));

	hulls.push_back(new ConvexHull(&bodyRear, 1));

	if (!playerShip) {
		std::vector<RigidSurface*> head;

		std::vector<Vector3D> hs1;
		hs1.push_back(points.at(20));
		hs1.push_back(points.at(8));
		hs1.push_back(points.at(18));
		head.push_back(new RigidSurface(&hs1, Vector3D(0, -1, 0)));

		std::vector<Vector3D> hs2;
		hs2.push_back(points.at(11));
		hs2.push_back(points.at(8));
		hs2.push_back(points.at(18));
		head.push_back(new RigidSurface(&hs2, Vector3D(0, 0, -1)));

		std::vector<Vector3D> hs3;
		hs3.push_back(points.at(11));
		hs3.push_back(points.at(8));
		hs3.push_back(points.at(15));
		hs3.push_back(points.at(14));
		head.push_back(new RigidSurface(&hs3, Vector3D(0, 1, 0)));

		std::vector<Vector3D> hs4;
		hs4.push_back(points.at(18));
		hs4.push_back(points.at(11));
		hs4.push_back(points.at(9));
		hs4.push_back(points.at(16));
		head.push_back(new RigidSurface(&hs4, Vector3D(0, -1, 0)));

		std::vector<Vector3D> hs5;
		hs5.push_back(points.at(11));
		hs5.push_back(points.at(9));
		hs5.push_back(points.at(14));
		head.push_back(new RigidSurface(&hs5, Vector3D(0, 0, -1)));

		std::vector<Vector3D> hs6;
		hs6.push_back(points.at(20));
		hs6.push_back(points.at(19));
		hs6.push_back(points.at(18));
		head.push_back(new RigidSurface(&hs6, Vector3D(0, 0, -1)));

		std::vector<Vector3D> hs7;
		hs7.push_back(points.at(19));
		hs7.push_back(points.at(18));
		hs7.push_back(points.at(16));
		head.push_back(new RigidSurface(&hs7, Vector3D(0, 0, 0)));

		//////

		std::vector<Vector3D> hs8;
		hs8.push_back(points.at(20));
		hs8.push_back(points.at(7));
		hs8.push_back(points.at(17));
		head.push_back(new RigidSurface(&hs8, Vector3D(0, 0, 1)));

		std::vector<Vector3D> hs9;
		hs9.push_back(points.at(10));
		hs9.push_back(points.at(7));
		hs9.push_back(points.at(17));
		head.push_back(new RigidSurface(&hs9, Vector3D(0, 0, 1)));

		std::vector<Vector3D> hs10;
		hs10.push_back(points.at(17));
		hs10.push_back(points.at(10));
		hs10.push_back(points.at(9));
		hs10.push_back(points.at(16));
		head.push_back(new RigidSurface(&hs10, Vector3D(0, 0, 1)));

		std::vector<Vector3D> hs11;
		hs11.push_back(points.at(10));
		hs11.push_back(points.at(9));
		hs11.push_back(points.at(14));
		head.push_back(new RigidSurface(&hs11, Vector3D(0, 0, 1)));

		std::vector<Vector3D> hs12;
		hs12.push_back(points.at(10));
		hs12.push_back(points.at(14));
		hs12.push_back(points.at(15));
		hs12.push_back(points.at(7));
		head.push_back(new RigidSurface(&hs12, Vector3D(0, 0, 1)));

		std::vector<Vector3D> hs13;
		hs13.push_back(points.at(20));
		hs13.push_back(points.at(19));
		hs13.push_back(points.at(17));
		head.push_back(new RigidSurface(&hs13, Vector3D(0, 0, 1)));

		std::vector<Vector3D> hs14;
		hs14.push_back(points.at(19));
		hs14.push_back(points.at(17));
		hs14.push_back(points.at(16));
		head.push_back(new RigidSurface(&hs14, Vector3D(0, 0, 1)));

		std::vector<Vector3D> hs15;
		hs15.push_back(points.at(8));
		hs15.push_back(points.at(15));
		hs15.push_back(points.at(7));
		hs15.push_back(points.at(20));
		head.push_back(new RigidSurface(&hs15, Vector3D(-1, 0, 0), true));

		hulls.push_back(new ConvexHull(&head, 1));
	}

	*bodyOut = new RigidBody(hulls, 1, 1, 0.3, false);
	*meshOut = new PolyModel(&polygons, (*bodyOut)->getCenterOfMass());
}

void MovingObject::getShipDebris(DebrisType debrisType, double size, olc::Pixel color, olc::Pixel lineColor, olc::Pixel highlight,
	olc::Pixel highlightLine, olc::Pixel cockpit, olc::Pixel cockpitLine, RigidBody** bodyOut, PolyModel** meshOut) {

	if (shipPoints.size() == 0) {
		initShipPoints();
	}

	std::vector<Polygon3D> polygons;
	std::vector<Vector3D> points = shipPoints;
	std::vector<ConvexHull*> hulls;

	for (int i = 0; i < points.size(); i++) {
		points[i] = points[i].multiply(size);
	}

	switch (debrisType) {
	case WingDebris:
	{
		polygons.push_back(Polygon3D(points.at(7), points.at(0), points.at(20), lineColor, color));
		polygons.push_back(Polygon3D(points.at(39), points.at(0), points.at(7), lineColor, color));
		polygons.push_back(Polygon3D(points.at(37), points.at(0), points.at(39), lineColor, color));
		polygons.push_back(Polygon3D(points.at(37), points.at(39), points.at(3), lineColor, color));
		polygons.push_back(Polygon3D(points.at(3), points.at(39), points.at(5), lineColor, color));
		polygons.push_back(Polygon3D(points.at(5), points.at(39), points.at(7), lineColor, color));
		polygons.push_back(Polygon3D(points.at(1), points.at(37), points.at(3), lineColor, color));
		polygons.push_back(Polygon3D(points.at(1), points.at(0), points.at(37), lineColor, color));
		polygons.push_back(Polygon3D(points.at(5), points.at(1), points.at(3), lineColor, color));
		polygons.push_back(Polygon3D(points.at(1), points.at(5), points.at(0), lineColor, color));
		polygons.push_back(Polygon3D(points.at(5), points.at(7), points.at(0), lineColor, color));

		std::vector<RigidSurface*> wing1;

		std::vector<Vector3D> w1s1;
		w1s1.push_back(points.at(3));
		w1s1.push_back(points.at(37));
		w1s1.push_back(points.at(39));
		wing1.push_back(new RigidSurface(&w1s1, Vector3D(0, -1, 0)));

		std::vector<Vector3D> w1s2;
		w1s2.push_back(points.at(3));
		w1s2.push_back(points.at(1));
		w1s2.push_back(points.at(5));
		wing1.push_back(new RigidSurface(&w1s2, Vector3D(0, 1, 0)));

		std::vector<Vector3D> w1s3;
		w1s3.push_back(points.at(3));
		w1s3.push_back(points.at(1));
		w1s3.push_back(points.at(37));
		wing1.push_back(new RigidSurface(&w1s3, Vector3D(0, 0, 1)));

		std::vector<Vector3D> w1s4;
		w1s4.push_back(points.at(39));
		w1s4.push_back(points.at(37));
		w1s4.push_back(points.at(1));
		w1s4.push_back(points.at(5));
		wing1.push_back(new RigidSurface(&w1s4, Vector3D(-1, 0, 0), true));

		std::vector<Vector3D> w1s5;
		w1s5.push_back(points.at(3));
		w1s5.push_back(points.at(39));
		w1s5.push_back(points.at(5));
		wing1.push_back(new RigidSurface(&w1s5, Vector3D(1, 0, 0)));

		hulls.push_back(new ConvexHull(&wing1, 1));

		std::vector<RigidSurface*> wing2;

		std::vector<Vector3D> w2s1;
		w2s1.push_back(points.at(39));
		w2s1.push_back(points.at(37));
		w2s1.push_back(points.at(1));
		w2s1.push_back(points.at(5));
		wing2.push_back(new RigidSurface(&w2s1, Vector3D(1, 0, 0), true));

		std::vector<Vector3D> w2s2;
		w2s2.push_back(points.at(39));
		w2s2.push_back(points.at(37));
		w2s2.push_back(points.at(0));
		wing2.push_back(new RigidSurface(&w2s2, Vector3D(0, -1, 0)));

		std::vector<Vector3D> w2s3;
		w2s3.push_back(points.at(5));
		w2s3.push_back(points.at(39));
		w2s3.push_back(points.at(7));
		wing2.push_back(new RigidSurface(&w2s3, Vector3D(1, 0, 0)));

		std::vector<Vector3D> w2s4;
		w2s4.push_back(points.at(0));
		w2s4.push_back(points.at(39));
		w2s4.push_back(points.at(7));
		wing2.push_back(new RigidSurface(&w2s4, Vector3D(0, -1, 0)));

		std::vector<Vector3D> w2s5;
		w2s5.push_back(points.at(37));
		w2s5.push_back(points.at(1));
		w2s5.push_back(points.at(0));
		wing2.push_back(new RigidSurface(&w2s5, Vector3D(-1, 0, 0)));

		std::vector<Vector3D> w2s6;
		w2s6.push_back(points.at(5));
		w2s6.push_back(points.at(1));
		w2s6.push_back(points.at(0));
		wing2.push_back(new RigidSurface(&w2s6, Vector3D(0, 1, 0)));

		std::vector<Vector3D> w2s7;
		w2s7.push_back(points.at(0));
		w2s7.push_back(points.at(5));
		w2s7.push_back(points.at(7));
		wing2.push_back(new RigidSurface(&w2s7, Vector3D(0, 1, 0)));

		hulls.push_back(new ConvexHull(&wing2, 1));
	}
		break;
	case HeadDebris:
	{

		polygons.push_back(Polygon3D(points.at(20), points.at(8), points.at(22), lineColor, color));
		polygons.push_back(Polygon3D(points.at(20), points.at(22), points.at(11), lineColor, color));
		polygons.push_back(Polygon3D(points.at(20), points.at(11), points.at(18), lineColor, color));
		polygons.push_back(Polygon3D(points.at(16), points.at(18), points.at(11), lineColor, color));
		polygons.push_back(Polygon3D(points.at(11), points.at(9), points.at(16), lineColor, color));
		polygons.push_back(Polygon3D(points.at(14), points.at(9), points.at(11), lineColor, color));
		polygons.push_back(Polygon3D(points.at(14), points.at(11), points.at(22), lineColor, color));
		polygons.push_back(Polygon3D(points.at(14), points.at(22), points.at(8), lineColor, color));
		polygons.push_back(Polygon3D(points.at(14), points.at(8), points.at(15), lineColor, color));
		polygons.push_back(Polygon3D(points.at(15), points.at(8), points.at(20), olc::Pixel(20, 7, 7), olc::Pixel(15, 5, 5)));

		polygons.push_back(Polygon3D(points.at(7), points.at(20), points.at(21), lineColor, color));
		polygons.push_back(Polygon3D(points.at(21), points.at(20), points.at(10), lineColor, color));
		polygons.push_back(Polygon3D(points.at(10), points.at(20), points.at(17), lineColor, color));
		polygons.push_back(Polygon3D(points.at(17), points.at(16), points.at(10), lineColor, color));
		polygons.push_back(Polygon3D(points.at(9), points.at(10), points.at(16), lineColor, color));
		polygons.push_back(Polygon3D(points.at(9), points.at(14), points.at(10), lineColor, color));
		polygons.push_back(Polygon3D(points.at(10), points.at(14), points.at(21), lineColor, color));
		polygons.push_back(Polygon3D(points.at(21), points.at(14), points.at(7), lineColor, color));
		polygons.push_back(Polygon3D(points.at(7), points.at(14), points.at(15), lineColor, color));
		polygons.push_back(Polygon3D(points.at(7), points.at(15), points.at(20), olc::Pixel(20, 7, 7), olc::Pixel(15, 5, 5)));

		polygons.push_back(Polygon3D(points.at(17), points.at(20), points.at(19), cockpitLine, cockpit));//cockpit
		polygons.push_back(Polygon3D(points.at(16), points.at(17), points.at(19), cockpitLine, cockpit));
		polygons.push_back(Polygon3D(points.at(20), points.at(18), points.at(19), cockpitLine, cockpit));//cockpit
		polygons.push_back(Polygon3D(points.at(18), points.at(16), points.at(19), cockpitLine, cockpit));//cockpit

		std::vector<RigidSurface*> head;

		std::vector<Vector3D> hs1;
		hs1.push_back(points.at(20));
		hs1.push_back(points.at(8));
		hs1.push_back(points.at(18));
		head.push_back(new RigidSurface(&hs1, Vector3D(0, -1, 0)));

		std::vector<Vector3D> hs2;
		hs2.push_back(points.at(11));
		hs2.push_back(points.at(8));
		hs2.push_back(points.at(18));
		head.push_back(new RigidSurface(&hs2, Vector3D(0, 0, -1)));

		std::vector<Vector3D> hs3;
		hs3.push_back(points.at(11));
		hs3.push_back(points.at(8));
		hs3.push_back(points.at(15));
		hs3.push_back(points.at(14));
		head.push_back(new RigidSurface(&hs3, Vector3D(0, 1, 0)));

		std::vector<Vector3D> hs4;
		hs4.push_back(points.at(18));
		hs4.push_back(points.at(11));
		hs4.push_back(points.at(9));
		hs4.push_back(points.at(16));
		head.push_back(new RigidSurface(&hs4, Vector3D(0, -1, 0)));

		std::vector<Vector3D> hs5;
		hs5.push_back(points.at(11));
		hs5.push_back(points.at(9));
		hs5.push_back(points.at(14));
		head.push_back(new RigidSurface(&hs5, Vector3D(0, 0, -1)));

		std::vector<Vector3D> hs6;
		hs6.push_back(points.at(20));
		hs6.push_back(points.at(19));
		hs6.push_back(points.at(18));
		head.push_back(new RigidSurface(&hs6, Vector3D(0, 0, -1)));

		std::vector<Vector3D> hs7;
		hs7.push_back(points.at(19));
		hs7.push_back(points.at(18));
		hs7.push_back(points.at(16));
		head.push_back(new RigidSurface(&hs7, Vector3D(0, 0, 0)));

		//////

		std::vector<Vector3D> hs8;
		hs8.push_back(points.at(20));
		hs8.push_back(points.at(7));
		hs8.push_back(points.at(17));
		head.push_back(new RigidSurface(&hs8, Vector3D(0, 0, 1)));

		std::vector<Vector3D> hs9;
		hs9.push_back(points.at(10));
		hs9.push_back(points.at(7));
		hs9.push_back(points.at(17));
		head.push_back(new RigidSurface(&hs9, Vector3D(0, 0, 1)));

		std::vector<Vector3D> hs10;
		hs10.push_back(points.at(17));
		hs10.push_back(points.at(10));
		hs10.push_back(points.at(9));
		hs10.push_back(points.at(16));
		head.push_back(new RigidSurface(&hs10, Vector3D(0, 0, 1)));

		std::vector<Vector3D> hs11;
		hs11.push_back(points.at(10));
		hs11.push_back(points.at(9));
		hs11.push_back(points.at(14));
		head.push_back(new RigidSurface(&hs11, Vector3D(0, 0, 1)));

		std::vector<Vector3D> hs12;
		hs12.push_back(points.at(10));
		hs12.push_back(points.at(14));
		hs12.push_back(points.at(15));
		hs12.push_back(points.at(7));
		head.push_back(new RigidSurface(&hs12, Vector3D(0, 0, 1)));

		std::vector<Vector3D> hs13;
		hs13.push_back(points.at(20));
		hs13.push_back(points.at(19));
		hs13.push_back(points.at(17));
		head.push_back(new RigidSurface(&hs13, Vector3D(0, 0, 1)));

		std::vector<Vector3D> hs14;
		hs14.push_back(points.at(19));
		hs14.push_back(points.at(17));
		hs14.push_back(points.at(16));
		head.push_back(new RigidSurface(&hs14, Vector3D(0, 0, 1)));

		std::vector<Vector3D> hs15;
		hs15.push_back(points.at(8));
		hs15.push_back(points.at(15));
		hs15.push_back(points.at(7));
		hs15.push_back(points.at(20));
		head.push_back(new RigidSurface(&hs15, Vector3D(-1, 0, 0), true));

		hulls.push_back(new ConvexHull(&head, 1));
	}
	}

	*bodyOut = new RigidBody(hulls, 1, 1, 0.3, false);
	*meshOut = new PolyModel(&polygons, (*bodyOut)->getCenterOfMass());
}
#include "Missile.h"
#include "Asteroid.h"
#include "SpaceMinerGame.h"

Missile::Missile(Vector3D pos, Rotor orientation, Vector3D vel, PhysicsObject* target, uint16_t sourceID, bool playerMissile) {

	this->playerMissile = playerMissile;
	olc::Pixel lineColor;
	olc::Pixel color;
	olc::Pixel highlight;

	if (playerMissile) {
		angularDampFactor = 25.0;
		linearDampFactor = 15;
		forwardThrust = 4000;
		pitchRate = 35;
		yawRate = 35;

		lineColor = olc::BLACK;
		color = olc::DARK_RED;
		highlight = olc::GREY;

	}
	else {
		angularDampFactor = 12.0;
		linearDampFactor = 3;
		forwardThrust = 2000;
		pitchRate = 12;
		yawRate = 12;

		lineColor = olc::BLACK;
		color = olc::DARK_MAGENTA;
		highlight = olc::GREEN;

	}

	double length = 150;
	double width = length/4.0;
	double finS = width * 1.2;

	std::vector<Vector3D> points;
	points.push_back(Vector3D(0, 0, 0));
	points.push_back(Vector3D(0, sqrt(3) * width / 2.0, -width / 2.0));
	points.push_back(Vector3D(0, sqrt(3) * width / 2.0, width / 2.0));

	points.push_back(Vector3D(length, 0, 0));
	points.push_back(Vector3D(length, sqrt(3) * width / 2.0, -width / 2.0));
	points.push_back(Vector3D(length, sqrt(3) * width / 2.0, width / 2.0));

	points.push_back(Vector3D(length * 5/4, width / sqrt(3), 0));

	//fins
	//7
	points.push_back(Vector3D(0, sqrt(3) * width / 6.0, -width / 6.0));
	points.push_back(Vector3D(0, sqrt(3) * width / 3.0, -width / 3.0));
	points.push_back(Vector3D(0, sqrt(3) * width / 4.0 - finS / 2.0, -width / 4.0 - finS * sqrt(3) / 2.0));
	points.push_back(Vector3D(length / 4.0, sqrt(3) * width / 4.0, -width / 4.0));

	//11
	points.push_back(Vector3D(0, sqrt(3) * width / 2.0, -width / 6.0));
	points.push_back(Vector3D(0, sqrt(3) * width / 2.0, width / 6.0));
	points.push_back(Vector3D(0, sqrt(3) * width / 2.0 + finS, 0));
	points.push_back(Vector3D(length / 4.0, sqrt(3) * width / 2.0, 0));

	//15
	points.push_back(Vector3D(0, sqrt(3) * width / 6.0, width / 6.0));
	points.push_back(Vector3D(0, sqrt(3) * width / 3.0, width / 3.0));
	points.push_back(Vector3D(0, sqrt(3) * width / 4.0 - finS / 2.0, width / 4.0 + finS * sqrt(3) / 2.0));
	points.push_back(Vector3D(length / 4.0, sqrt(3) * width / 4.0, width / 4.0));


	std::vector<Polygon3D> polygons;
	polygons.push_back(Polygon3D(points.at(3), points.at(0), points.at(1), lineColor, color));
	polygons.push_back(Polygon3D(points.at(3), points.at(1), points.at(4), lineColor, color));
	polygons.push_back(Polygon3D(points.at(4), points.at(1), points.at(5), lineColor, color));
	polygons.push_back(Polygon3D(points.at(5), points.at(1), points.at(2), lineColor, color));
	polygons.push_back(Polygon3D(points.at(3), points.at(0), points.at(1), lineColor, color));
	polygons.push_back(Polygon3D(points.at(5), points.at(2), points.at(0), lineColor, color));
	polygons.push_back(Polygon3D(points.at(5), points.at(0), points.at(3), lineColor, color));

	polygons.push_back(Polygon3D(points.at(0), points.at(2), points.at(1), lineColor, color));
	//cap
	polygons.push_back(Polygon3D(points.at(3), points.at(4), points.at(6), lineColor, highlight));
	polygons.push_back(Polygon3D(points.at(4), points.at(5), points.at(6), lineColor, highlight));
	polygons.push_back(Polygon3D(points.at(5), points.at(3), points.at(6), lineColor, highlight));
	//fins
	polygons.push_back(Polygon3D(points.at(8), points.at(9), points.at(7), lineColor, highlight));
	polygons.push_back(Polygon3D(points.at(7), points.at(9), points.at(10), lineColor, highlight));
	polygons.push_back(Polygon3D(points.at(8), points.at(10), points.at(9), lineColor, highlight));

	polygons.push_back(Polygon3D(points.at(11), points.at(12), points.at(13), lineColor, highlight));
	polygons.push_back(Polygon3D(points.at(12), points.at(14), points.at(13), lineColor, highlight));
	polygons.push_back(Polygon3D(points.at(13), points.at(14), points.at(11), lineColor, highlight));

	polygons.push_back(Polygon3D(points.at(15), points.at(17), points.at(16), lineColor, highlight));
	polygons.push_back(Polygon3D(points.at(15), points.at(18), points.at(17), lineColor, highlight));
	polygons.push_back(Polygon3D(points.at(17), points.at(18), points.at(16), lineColor, highlight));

	std::vector<RigidSurface*> surfaces;

	std::vector<Vector3D> s1;
	s1.push_back(points.at(0));
	s1.push_back(points.at(1));
	s1.push_back(points.at(2));
	surfaces.push_back(new RigidSurface(&s1, Vector3D(-1, 0, 0)));

	std::vector<Vector3D> s2;
	s2.push_back(points.at(0));
	s2.push_back(points.at(3));
	s2.push_back(points.at(4));
	s2.push_back(points.at(1));
	surfaces.push_back(new RigidSurface(&s2, Vector3D(0, 0, -1)));

	std::vector<Vector3D> s3;
	s3.push_back(points.at(1));
	s3.push_back(points.at(4));
	s3.push_back(points.at(5));
	s3.push_back(points.at(2));
	surfaces.push_back(new RigidSurface(&s3, Vector3D(0, 1, 0)));

	std::vector<Vector3D> s4;
	s4.push_back(points.at(2));
	s4.push_back(points.at(5));
	s4.push_back(points.at(3));
	s4.push_back(points.at(0));
	surfaces.push_back(new RigidSurface(&s4, Vector3D(0, 0, 1)));

	std::vector<Vector3D> s5;
	s5.push_back(points.at(3));
	s5.push_back(points.at(4));
	s5.push_back(points.at(6));
	surfaces.push_back(new RigidSurface(&s5, Vector3D(1, 0, 0)));

	std::vector<Vector3D> s6;
	s6.push_back(points.at(4));
	s6.push_back(points.at(5));
	s6.push_back(points.at(6));
	surfaces.push_back(new RigidSurface(&s6, Vector3D(1, 0, 0)));

	std::vector<Vector3D> s7;
	s7.push_back(points.at(5));
	s7.push_back(points.at(3));
	s7.push_back(points.at(6));
	surfaces.push_back(new RigidSurface(&s7, Vector3D(1, 0, 0)));

	std::vector<ConvexHull*> hulls;
	hulls.push_back(new ConvexHull(&surfaces, 1));
	body = new RigidBody(hulls, 1, 1, 0.3, false);
	model = new PolyModel(&polygons, body->getCenterOfMass());

	body->translate(pos);
	body->setToOrientation(orientation);

    findIVals();
	
	body->setVelocity(vel);
	this->target = target;
	body->addNoCol(sourceID);
	body->setTrackCollHistory(true);
	life.reset();

	if (target != nullptr) {
		targetID = target->getRigidBody()->getID();
	}
}

void Missile::giveTarget(PhysicsObject* target) {
	this->target = target;
	if (target != nullptr) {
		targetID = target->getRigidBody()->getID();
	}
}

RigidBody* Missile::getBody() {
	return body;
}

void Missile::performDeathActions(SpaceMinerGame* game) {
	game->getPhysicsEngine()->removeRigidBody(body);
}

void Missile::aimAtTarget(float fElapsedTime) {

	if (target == nullptr) {
		return;
	}

	Vector3D toTarget = target->getRigidBody()->getCenterOfMass().sub(body->getCenterOfMass());

	if (toTarget.getUnitVector().dotProduct(getDir()) < maxAngle) {
		target = nullptr;
		return;
	}

	Vector3D targetRel = Projectile::calculateLeadPoint(getPos(), target->getPos(), body->getVelocity(), target->getRigidBody()->getVelocity(), body->getVelocity().getMagnitude(),
		true, 0, targetPrevVel, fElapsedTime);

	targetPrevVel = target->getRigidBody()->getVelocity();

	Vector3D pitchDir = body->getOrientation().rotate(Vector3D(0, 1, 0));
	Vector3D yawDir = body->getOrientation().rotate(Vector3D(0, 0, -1));
	
	if (targetRel.dotProduct(yawDir) > 0) {
		yaw(-1, fElapsedTime);
	}
	else {
		yaw(1, fElapsedTime);
	}

	if (targetRel.dotProduct(pitchDir) > 0) {
		pitch(-1, fElapsedTime);
	}
	else {
		pitch(1, fElapsedTime);
	}
	
}

void Missile::dampenMotion(double timePassed) {
	float t = std::fmin(1, timePassed);
	Vector3D vel = body->getVelocity();
	Vector3D aVel = body->getAngularVelocity();

	double min = 0.05;
	Vector3D dir = getDir();
	Vector3D velocParr = dir.multiply(body->getVelocity().dotProduct(dir));
	Vector3D velocNorm = body->getVelocity().sub(velocParr);
	
	double stabFactor = std::fmin(1, liveTime / maxTurnT);
	body->setVelocity(body->getVelocity().sub(velocNorm.multiply(t * stabFactor * linearDampFactor).add(velocParr.multiply(min * t * stabFactor * linearDampFactor))));
	body->setAngVelocity(aVel.sub(aVel.multiply(t * angularDampFactor)));
}

void Missile::update(SpaceMinerGame* game, float fElapsedTime) {

	liveTime += fElapsedTime;
	dampenMotion(fElapsedTime);
	


	if (target == nullptr) {
		life.updateTimer(fElapsedTime);
	}
	else if (!game->getPhysicsEngine()->bodyInUse(targetID)) {
		target = nullptr;
	}

	aimAtTarget(fElapsedTime);

	std::vector<CollisionInfo>* colls = body->getCollHistory();
	if (colls->size() > 0) {
		int collID = colls->at(0).otherBodyID;

		for (Enemy* e : *game->getEnemies()) {
			if (e->getRigidBody()->getID() == collID) {
				e->damage(100);
				break;
			}
		}

		for (Asteroid* a : *game->getAsteroids()) {
			if (a->getRigidBody()->getID() == collID) {
				a->damage(100);
				break;
			}
		}

		Particle::generateExplosion(game, body->getCenterOfMass(), 0.6, 1250, 300, olc::WHITE);
		Particle::generateExplosion(game, body->getCenterOfMass(), 0.7, 750, 150, olc::WHITE);
		collided = true;
		return;
	}

	accelForward(1, fElapsedTime);
	if (playerMissile) {
		Particle::generateExplosion(game, getPos(), 0.7, 50, 3, olc::RED);
		Particle::generateExplosion(game, getPos(), 0.9, 100, 1, olc::YELLOW);
		Particle::generateExplosion(game, getPos(), 1.5, 150, 3, olc::DARK_GREY);
	}
	else {
		Particle::generateExplosion(game, getPos(), 0.7, 50, 3, olc::WHITE);
		Particle::generateExplosion(game, getPos(), 0.9, 100, 3, olc::CYAN);
		Particle::generateExplosion(game, getPos(), 1.5, 150, 1, olc::WHITE);
	}
	
}

void Missile::draw(PixelEngine3D* g, Vector3D cameraPos, Rotor cameraDir, double FOV) {
	MovingObject::draw(g, cameraPos, cameraDir, FOV);
	//debugDraw(g, cameraPos, cameraDir, FOV);
}

bool Missile::isExpired() {
	return collided || life.isReady();
}
#include "SpaceStation.h"
#include "SpaceMinerGame.h"
#include "Bullet.h"

double SpaceStation::Turret::size;

SpaceStation::SpaceStation(SpaceMinerGame* game, Vector3D pos) {
	createStructure();
	mainStructure->getRigidBody()->translate(pos);
	turret1->body->getRigidBody()->translate(pos);
	turret1->gun->getRigidBody()->translate(pos);
	turret2->body->getRigidBody()->translate(pos);
	turret2->gun->getRigidBody()->translate(pos);

	PhysicsEngine* pEngine = game->getPhysicsEngine();
	pEngine->addRigidBody(mainStructure->getRigidBody());
	pEngine->addRigidBody(turret1->body->getRigidBody());
	pEngine->addRigidBody(turret1->gun->getRigidBody());
	pEngine->addRigidBody(turret2->body->getRigidBody());
	pEngine->addRigidBody(turret2->gun->getRigidBody());
	
}

void SpaceStation::update(SpaceMinerGame* game, float fElapsedTime) {
	turret1->update(game, fElapsedTime);
	turret2->update(game, fElapsedTime);
}

void SpaceStation::draw(PixelEngine3D* g, Vector3D cameraPos, Rotor cameraDir, double FOV) {
	mainStructure->draw(g, cameraPos, cameraDir, FOV);
	turret1->body->draw(g, cameraPos, cameraDir, FOV);
	turret1->gun->draw(g, cameraPos, cameraDir, FOV);
	turret2->body->draw(g, cameraPos, cameraDir, FOV);
	turret2->gun->draw(g, cameraPos, cameraDir, FOV);

	if (turret1->target != nullptr) {
		Vector3D leadTarget = Projectile::calculateLeadPoint(turret1->gun->getPos(), turret1->target->getPos(), Vector3D(0, 0, 0), turret1->target->getRigidBody()->getVelocity(), turret1->bulletVel);
		//g->draw3DPoint(leadTarget, cameraPos, cameraDir, FOV, olc::RED);
	}
}

SpaceStation::Turret::Turret(PhysicsObject* body, PhysicsObject* gun, Vector3D upDir) {
	this->body = body;
	this->gun = gun;
	this->upDir = upDir;
}

void SpaceStation::Turret::aimAndShoot(SpaceMinerGame* game, float fElapsedTime) {
	Vector3D toTarget = target->getPos().sub(body->getPos());
	Vector3D toTargetUnit = toTarget.getUnitVector();

	Vector3D leadTarget = Projectile::calculateLeadPoint(gun->getPos(), target->getPos(), Vector3D(0, 0, 0), target->getRigidBody()->getVelocity(), bulletVel,
		true, size, prevTargetVel, fElapsedTime);

	prevTargetVel = target->getRigidBody()->getVelocity();

	Vector3D turnDir = body->getRigidBody()->getOrientation().rotate(Vector3D(1, 0, 0));
	if (turnDir.dotProduct(leadTarget) > 0) {
		body->getRigidBody()->setAngVelocity(upDir.multiply(-turnRate));
	}
	else {
		body->getRigidBody()->setAngVelocity(upDir.multiply(turnRate));
	}

	double targetPitch = 3.14159 / 2.0 - acos(upDir.dotProduct(toTarget.getUnitVector()));

	int pitchdir = 1;
	if (currPitch < targetPitch) {
		currPitch += fElapsedTime * pitchRate;
		currPitch = fmin(maxPitch, currPitch);
	}
	else {
		pitchdir = -1;
		currPitch -= fElapsedTime * pitchRate;
		currPitch = fmax(minPitch, currPitch);
	}

	if (getGunDir().rotate(Vector3D(1, 0, 0)).dotProduct(leadTarget.getUnitVector()) > cos(3.14159 / 9.0)) {
		shoot(game);
	}

	Vector3D pitchAxis = body->getRigidBody()->getOrientation().rotate(Vector3D(1, 0, 0));

	gun->getRigidBody()->setAngVelocity(body->getRigidBody()->getAngularVelocity().add(pitchAxis.multiply(pitchdir * pitchRate)));
}

bool SpaceStation::Turret::inRange(MovingObject* o) {
	return o->getPos().sub(body->getPos()).getMagnitudeSquared() <= maxRange * maxRange;
}

void SpaceStation::Turret::update(SpaceMinerGame* game, float fElapsedTime) {
	fireRate.updateTimer(fElapsedTime);

	if (target != nullptr) {
		if (!game->getPhysicsEngine()->bodyInUse(targetID)
			|| !inRange(target)) {
			target = nullptr;
			return;
		}
		aimAndShoot(game, fElapsedTime);
	}
	else {
		for (Enemy* e : *game->getEnemies()) {
			if (inRange(e)) {
				target = e;
				targetID = e->getRigidBody()->getID();
				break;
			}
		}
	}

	Vector3D pitchAxis = body->getRigidBody()->getOrientation().rotate(Vector3D(1, 0, 0));

	gun->getRigidBody()->setToOrientation(body->getRigidBody()->getOrientation().applyRotor(Rotor(pitchAxis, currPitch)));
	
}

Rotor SpaceStation::Turret::getGunDir() {
	return Rotor(Vector3D(0, 1, 0), -3.14159 / 2.0).applyRotor(gun->getRigidBody()->getOrientation());
}

void SpaceStation::Turret::shoot(SpaceMinerGame* game) {
	if (fireRate.isReady()) {
		fireRate.reset();

		Vector3D dir = gun->getRigidBody()->getOrientation().rotate(Vector3D(0, 0, 1));
		double spawnDist = size;

		Vector3D spawnPos = body->getRigidBody()->getCenterOfMass().add(dir.multiply(spawnDist));

		Rotor orientation = getGunDir();

		Vector3D variation = Vector3D(0.5 - (float)rand() / RAND_MAX, 0.5 - (float)rand() / RAND_MAX, 0.5 - (float)rand() / RAND_MAX).getUnitVector().multiply(spread);
		Bullet* bullet = new Bullet(gun->getRigidBody()->getCenterOfMass().add(dir.multiply(spawnDist)), orientation,
			dir.add(variation).multiply(bulletVel), 150, body->getRigidBody()->getID(), olc::RED, olc::WHITE, 3.0);
		bullet->getBody()->addNoCol(gun->getRigidBody()->getID());

		game->addProjectile(bullet);
	}
}

void SpaceStation::createStructure() {

	double s = 2700;
	Turret::size = 5 * s / 9.0;

	double r1 = 2 * s / 9.0;
	points.push_back(Vector3D(2 * r1 / sqrt(3), 0, 0));
	points.push_back(Vector3D(r1 / sqrt(3), 0, -r1));
	points.push_back(Vector3D(-r1 / sqrt(3), 0, -r1));
	points.push_back(Vector3D(-2 * r1 / sqrt(3), 0, 0));
	points.push_back(Vector3D(-r1 / sqrt(3), 0, r1));
	points.push_back(Vector3D(r1 / sqrt(3), 0, r1));

	//6
	points.push_back(Vector3D(2 * s / sqrt(3), 2 * s / 9.0, 0));
	points.push_back(Vector3D(s / sqrt(3), 2 * s / 9.0, -s));
	points.push_back(Vector3D(-s / sqrt(3), 2 * s / 9.0, -s));
	points.push_back(Vector3D(-2 * s / sqrt(3), 2 * s / 9.0, 0));
	points.push_back(Vector3D(-s / sqrt(3), 2 * s / 9.0, s));
	points.push_back(Vector3D(s / sqrt(3), 2 * s / 9.0, s));


	double ws = 0.6 * s / 9.0;
	double wd = 0.25 * s / 9.0;

	//12
	points.push_back(points.at(6).add(Vector3D(-ws / 2.0 - sqrt(3) * wd / 2.0, ws, sqrt(3) * ws / 2.0 - wd / 2.0)));
	points.push_back(points.at(6).add(Vector3D(-ws / 2.0 - sqrt(3) * wd / 2.0, ws, -sqrt(3) * ws / 2.0 + wd / 2.0)));
	//
	points.push_back(points.at(7).add(Vector3D(ws / 2.0 - sqrt(3) * wd / 2.0, ws, sqrt(3) * ws / 2.0 + wd / 2.0)));
	points.push_back(points.at(7).add(Vector3D(-ws, ws, wd)));
	//
	points.push_back(points.at(8).add(Vector3D(ws, ws, wd)));
	points.push_back(points.at(8).add(Vector3D(-ws / 2.0 + sqrt(3) * wd / 2.0, ws, sqrt(3) * ws / 2.0 + wd / 2.0)));
	//
	points.push_back(points.at(9).add(Vector3D(ws / 2.0 + sqrt(3) * wd / 2.0, ws, -sqrt(3) * ws / 2.0 + wd / 2.0)));
	points.push_back(points.at(9).add(Vector3D(ws / 2.0 + sqrt(3) * wd / 2.0, ws, sqrt(3) * ws / 2.0 - wd / 2.0)));
	//
	points.push_back(points.at(10).add(Vector3D(-ws / 2.0 + sqrt(3) * wd / 2.0, ws, -sqrt(3) * ws / 2.0 - wd / 2.0)));
	points.push_back(points.at(10).add(Vector3D(ws, ws, -wd)));
	//
	points.push_back(points.at(11).add(Vector3D(-ws, ws, -wd)));
	points.push_back(points.at(11).add(Vector3D(ws / 2.0 - sqrt(3) * wd / 2.0, ws, -sqrt(3) * ws / 2.0 - wd / 2.0)));

	for (int i = 12; i < 24; i++) {
		points.push_back(points.at(i).add(Vector3D(0, 6 * s / 9.0 - 2 * ws, 0)));
	}

	//36
	points.push_back(Vector3D(2 * s / sqrt(3), 8 * s / 9.0, 0));
	points.push_back(Vector3D(s / sqrt(3), 8 * s / 9.0, -s));
	points.push_back(Vector3D(-s / sqrt(3), 8 * s / 9.0, -s));
	points.push_back(Vector3D(-2 * s / sqrt(3), 8 * s / 9.0, 0));
	points.push_back(Vector3D(-s / sqrt(3), 8 * s / 9.0, s));
	points.push_back(Vector3D(s / sqrt(3), 8 * s / 9.0, s));

	double r3 = 4 * s / 9.0;

	//42
	points.push_back(Vector3D(2 * r3 / sqrt(3), 10 * s / 9.0, 0));
	points.push_back(Vector3D(r3 / sqrt(3), 10 * s / 9.0, -r3));
	points.push_back(Vector3D(-r3 / sqrt(3), 10 * s / 9.0, -r3));
	points.push_back(Vector3D(-2 * r3 / sqrt(3), 10 * s / 9.0, 0));
	points.push_back(Vector3D(-r3 / sqrt(3), 10 * s / 9.0, r3));
	points.push_back(Vector3D(r3 / sqrt(3), 10 * s / 9.0, r3));

	//48
	points.push_back(Vector3D(2 * r1 / sqrt(3), 22 * s / 9.0, 0));
	points.push_back(Vector3D(r1 / sqrt(3), 22 * s / 9.0, -r1));
	points.push_back(Vector3D(-r1 / sqrt(3), 22 * s / 9.0, -r1));
	points.push_back(Vector3D(-2 * r1 / sqrt(3), 22 * s / 9.0, 0));
	points.push_back(Vector3D(-r1 / sqrt(3), 22 * s / 9.0, r1));
	points.push_back(Vector3D(r1 / sqrt(3), 22 * s / 9.0, r1));

	//54
	points.push_back(Vector3D(0, 0, 0));
	points.push_back(Vector3D(0, 22 * s / 9.0, 0));

	double r4 = 2 * s / 9.0;
	double r5 = 1 * s / 9.0;

	//56
	points.push_back(Vector3D(2 * r4 / sqrt(3), -2 * s / 9.0, 0));
	points.push_back(Vector3D(r4 / sqrt(3), -2 * s / 9.0, -r4));
	points.push_back(Vector3D(-r4 / sqrt(3), -2 * s / 9.0, -r4));
	points.push_back(Vector3D(-2 * r4 / sqrt(3), -2 * s / 9.0, 0));
	points.push_back(Vector3D(-r4 / sqrt(3), -2 * s / 9.0, r4));
	points.push_back(Vector3D(r4 / sqrt(3), -2 * s / 9.0, r4));

	//62
	points.push_back(Vector3D(2 * r5 / sqrt(3), -3 * s / 9.0, 0));
	points.push_back(Vector3D(r5 / sqrt(3), -3 * s / 9.0, -r5));
	points.push_back(Vector3D(-r5 / sqrt(3), -3 * s / 9.0, -r5));
	points.push_back(Vector3D(-2 * r5 / sqrt(3), -3 * s / 9.0, 0));
	points.push_back(Vector3D(-r5 / sqrt(3), -3 * s / 9.0, r5));
	points.push_back(Vector3D(r5 / sqrt(3), -3 * s / 9.0, r5));

	//68
	points.push_back(Vector3D(0.5 * s / 9.0, -s / 9.0, 0));
	points.push_back(Vector3D(-0.5 * s / 9.0, -s / 9.0, 0));
	points.push_back(Vector3D(-0.5 * s / 9.0, - 2 * s / 9.0, 0));
	points.push_back(Vector3D(0.5 * s / 9.0, -2 * s / 9.0, 0));
	//72
	points.push_back(Vector3D(0, -s / 9.0, Turret::size));
	points.push_back(Vector3D(-0.5 * s / 9.0, -1.5 * s / 9.0, Turret::size));
	points.push_back(Vector3D(0 * s / 9.0, -2 * s / 9.0, Turret::size));
	points.push_back(Vector3D(0.5 * s / 9.0, -1.5 * s / 9.0, Turret::size));

	//76
	points.push_back(Vector3D(0, -3 * s / 9.0, 0));

	olc::Pixel color = olc::VERY_DARK_RED;
	olc::Pixel lineColor = olc::BLACK;
	olc::Pixel wColor = olc::DARK_CYAN;
	olc::Pixel wLineColor = olc::DARK_CYAN;

	polygons.push_back(Polygon3D(points.at(54), points.at(1), points.at(0), lineColor, color));
	polygons.push_back(Polygon3D(points.at(54), points.at(2), points.at(1), lineColor, color));
	polygons.push_back(Polygon3D(points.at(54), points.at(3), points.at(2), lineColor, color));
	polygons.push_back(Polygon3D(points.at(54), points.at(4), points.at(3), lineColor, color));
	polygons.push_back(Polygon3D(points.at(54), points.at(5), points.at(4), lineColor, color));
	polygons.push_back(Polygon3D(points.at(54), points.at(0), points.at(5), lineColor, color));

	polygons.push_back(Polygon3D(points.at(0), points.at(7), points.at(6), lineColor, color));
	polygons.push_back(Polygon3D(points.at(0), points.at(1), points.at(7), lineColor, color));
	//
	polygons.push_back(Polygon3D(points.at(1), points.at(8), points.at(7), lineColor, color));
	polygons.push_back(Polygon3D(points.at(1), points.at(2), points.at(8), lineColor, color));
	//
	polygons.push_back(Polygon3D(points.at(2), points.at(9), points.at(8), lineColor, color));
	polygons.push_back(Polygon3D(points.at(2), points.at(3), points.at(9), lineColor, color));
	//
	polygons.push_back(Polygon3D(points.at(3), points.at(10), points.at(9), lineColor, color));
	polygons.push_back(Polygon3D(points.at(3), points.at(4), points.at(10), lineColor, color));
	//
	polygons.push_back(Polygon3D(points.at(4), points.at(11), points.at(10), lineColor, color));
	polygons.push_back(Polygon3D(points.at(4), points.at(5), points.at(11), lineColor, color));
	//
	polygons.push_back(Polygon3D(points.at(5), points.at(6), points.at(11), lineColor, color));
	polygons.push_back(Polygon3D(points.at(5), points.at(0), points.at(6), lineColor, color));


	polygons.push_back(Polygon3D(points.at(6), points.at(7), points.at(14), lineColor, color));
	polygons.push_back(Polygon3D(points.at(6), points.at(14), points.at(13), lineColor, color));
	//
	polygons.push_back(Polygon3D(points.at(7), points.at(8), points.at(16), lineColor, color));
	polygons.push_back(Polygon3D(points.at(7), points.at(16), points.at(15), lineColor, color));
	//
	polygons.push_back(Polygon3D(points.at(8), points.at(9), points.at(18), lineColor, color));
	polygons.push_back(Polygon3D(points.at(8), points.at(18), points.at(17), lineColor, color));
	//
	polygons.push_back(Polygon3D(points.at(9), points.at(10), points.at(20), lineColor, color));
	polygons.push_back(Polygon3D(points.at(9), points.at(20), points.at(19), lineColor, color));
	//
	polygons.push_back(Polygon3D(points.at(10), points.at(11), points.at(22), lineColor, color));
	polygons.push_back(Polygon3D(points.at(10), points.at(22), points.at(21), lineColor, color));
	//
	polygons.push_back(Polygon3D(points.at(11), points.at(6), points.at(12), lineColor, color));
	polygons.push_back(Polygon3D(points.at(11), points.at(12), points.at(23), lineColor, color));
	//

	polygons.push_back(Polygon3D(points.at(6), points.at(13), points.at(25), lineColor, color));
	polygons.push_back(Polygon3D(points.at(6), points.at(25), points.at(36), lineColor, color));
	polygons.push_back(Polygon3D(points.at(7), points.at(37), points.at(14), lineColor, color));
	polygons.push_back(Polygon3D(points.at(14), points.at(37), points.at(26), lineColor, color));
	//
	polygons.push_back(Polygon3D(points.at(7), points.at(15), points.at(27), lineColor, color));
	polygons.push_back(Polygon3D(points.at(7), points.at(27), points.at(37), lineColor, color));
	polygons.push_back(Polygon3D(points.at(8), points.at(38), points.at(16), lineColor, color));
	polygons.push_back(Polygon3D(points.at(16), points.at(38), points.at(28), lineColor, color));
	//
	polygons.push_back(Polygon3D(points.at(8), points.at(17), points.at(29), lineColor, color));
	polygons.push_back(Polygon3D(points.at(8), points.at(29), points.at(38), lineColor, color));
	polygons.push_back(Polygon3D(points.at(9), points.at(39), points.at(18), lineColor, color));
	polygons.push_back(Polygon3D(points.at(18), points.at(39), points.at(30), lineColor, color));
	//
	polygons.push_back(Polygon3D(points.at(9), points.at(19), points.at(31), lineColor, color));
	polygons.push_back(Polygon3D(points.at(9), points.at(31), points.at(39), lineColor, color));
	polygons.push_back(Polygon3D(points.at(10), points.at(40), points.at(20), lineColor, color));
	polygons.push_back(Polygon3D(points.at(20), points.at(40), points.at(32), lineColor, color));
	//
	polygons.push_back(Polygon3D(points.at(10), points.at(21), points.at(33), lineColor, color));
	polygons.push_back(Polygon3D(points.at(10), points.at(33), points.at(40), lineColor, color));
	polygons.push_back(Polygon3D(points.at(11), points.at(41), points.at(22), lineColor, color));
	polygons.push_back(Polygon3D(points.at(22), points.at(41), points.at(34), lineColor, color));
	//
	polygons.push_back(Polygon3D(points.at(11), points.at(23), points.at(35), lineColor, color));
	polygons.push_back(Polygon3D(points.at(11), points.at(35), points.at(41), lineColor, color));
	polygons.push_back(Polygon3D(points.at(6), points.at(36), points.at(12), lineColor, color));
	polygons.push_back(Polygon3D(points.at(12), points.at(36), points.at(24), lineColor, color));


	polygons.push_back(Polygon3D(points.at(25), points.at(26), points.at(37), lineColor, color));
	polygons.push_back(Polygon3D(points.at(25), points.at(37), points.at(36), lineColor, color));
	//
	polygons.push_back(Polygon3D(points.at(27), points.at(28), points.at(38), lineColor, color));
	polygons.push_back(Polygon3D(points.at(27), points.at(38), points.at(37), lineColor, color));
	//
	polygons.push_back(Polygon3D(points.at(29), points.at(30), points.at(39), lineColor, color));
	polygons.push_back(Polygon3D(points.at(29), points.at(39), points.at(38), lineColor, color));
	//
	polygons.push_back(Polygon3D(points.at(31), points.at(32), points.at(40), lineColor, color));
	polygons.push_back(Polygon3D(points.at(31), points.at(40), points.at(39), lineColor, color));
	//
	polygons.push_back(Polygon3D(points.at(33), points.at(34), points.at(41), lineColor, color));
	polygons.push_back(Polygon3D(points.at(33), points.at(41), points.at(40), lineColor, color));
	//
	polygons.push_back(Polygon3D(points.at(35), points.at(24), points.at(36), lineColor, color));
	polygons.push_back(Polygon3D(points.at(35), points.at(36), points.at(41), lineColor, color));


	polygons.push_back(Polygon3D(points.at(13), points.at(14), points.at(25), wLineColor, wColor));
	polygons.push_back(Polygon3D(points.at(14), points.at(26), points.at(25), wLineColor, wColor));
	//
	polygons.push_back(Polygon3D(points.at(15), points.at(16), points.at(27), wLineColor, wColor));
	polygons.push_back(Polygon3D(points.at(16), points.at(28), points.at(27), wLineColor, wColor));
	//
	polygons.push_back(Polygon3D(points.at(17), points.at(18), points.at(29), wLineColor, wColor));
	polygons.push_back(Polygon3D(points.at(18), points.at(30), points.at(29), wLineColor, wColor));
	//
	polygons.push_back(Polygon3D(points.at(19), points.at(20), points.at(31), wLineColor, wColor));
	polygons.push_back(Polygon3D(points.at(20), points.at(32), points.at(31), wLineColor, wColor));
	//
	polygons.push_back(Polygon3D(points.at(21), points.at(22), points.at(33), wLineColor, wColor));
	polygons.push_back(Polygon3D(points.at(22), points.at(34), points.at(33), wLineColor, wColor));
	//
	polygons.push_back(Polygon3D(points.at(23), points.at(12), points.at(35), wLineColor, wColor));
	polygons.push_back(Polygon3D(points.at(12), points.at(24), points.at(35), wLineColor, wColor));
	//


	polygons.push_back(Polygon3D(points.at(42), points.at(36), points.at(43), lineColor, color));
	polygons.push_back(Polygon3D(points.at(43), points.at(36), points.at(37), lineColor, color));
	//
	polygons.push_back(Polygon3D(points.at(43), points.at(37), points.at(44), lineColor, color));
	polygons.push_back(Polygon3D(points.at(44), points.at(37), points.at(38), lineColor, color));
	//
	polygons.push_back(Polygon3D(points.at(44), points.at(38), points.at(45), lineColor, color));
	polygons.push_back(Polygon3D(points.at(45), points.at(38), points.at(39), lineColor, color));
	//
	polygons.push_back(Polygon3D(points.at(45), points.at(39), points.at(46), lineColor, color));
	polygons.push_back(Polygon3D(points.at(46), points.at(39), points.at(40), lineColor, color));
	//
	polygons.push_back(Polygon3D(points.at(46), points.at(40), points.at(47), lineColor, color));
	polygons.push_back(Polygon3D(points.at(47), points.at(40), points.at(41), lineColor, color));
	//
	polygons.push_back(Polygon3D(points.at(47), points.at(41), points.at(42), lineColor, color));
	polygons.push_back(Polygon3D(points.at(42), points.at(41), points.at(36), lineColor, color));


	polygons.push_back(Polygon3D(points.at(42), points.at(49), points.at(48), lineColor, color));
	polygons.push_back(Polygon3D(points.at(42), points.at(43), points.at(49), lineColor, color));
	//
	polygons.push_back(Polygon3D(points.at(43), points.at(50), points.at(49), lineColor, color));
	polygons.push_back(Polygon3D(points.at(43), points.at(44), points.at(50), lineColor, color));
	//
	polygons.push_back(Polygon3D(points.at(44), points.at(51), points.at(50), lineColor, color));
	polygons.push_back(Polygon3D(points.at(44), points.at(45), points.at(51), lineColor, color));
	//
	polygons.push_back(Polygon3D(points.at(45), points.at(52), points.at(51), lineColor, color));
	polygons.push_back(Polygon3D(points.at(45), points.at(46), points.at(52), lineColor, color));
	//
	polygons.push_back(Polygon3D(points.at(46), points.at(53), points.at(52), lineColor, color));
	polygons.push_back(Polygon3D(points.at(46), points.at(47), points.at(53), lineColor, color));
	//
	polygons.push_back(Polygon3D(points.at(47), points.at(48), points.at(53), lineColor, color));
	polygons.push_back(Polygon3D(points.at(47), points.at(42), points.at(48), lineColor, color));
	

	polygons.push_back(Polygon3D(points.at(55), points.at(48), points.at(49), lineColor, color));
	//
	polygons.push_back(Polygon3D(points.at(55), points.at(49), points.at(50), lineColor, color));
	//
	polygons.push_back(Polygon3D(points.at(55), points.at(50), points.at(51), lineColor, color));
	//
	polygons.push_back(Polygon3D(points.at(55), points.at(51), points.at(52), lineColor, color));
	//
	polygons.push_back(Polygon3D(points.at(55), points.at(52), points.at(53), lineColor, color));
	//
	polygons.push_back(Polygon3D(points.at(55), points.at(53), points.at(48), lineColor, color));
	
	
	//turret polygons
	std::vector<Polygon3D> turrpolygons;
	turrpolygons.push_back(Polygon3D(points.at(0), points.at(1), points.at(54), lineColor, color));
	turrpolygons.push_back(Polygon3D(points.at(1), points.at(2), points.at(54), lineColor, color));
	turrpolygons.push_back(Polygon3D(points.at(2), points.at(3), points.at(54), lineColor, color));
	turrpolygons.push_back(Polygon3D(points.at(3), points.at(4), points.at(54), lineColor, color));
	turrpolygons.push_back(Polygon3D(points.at(4), points.at(5), points.at(54), lineColor, color));
	turrpolygons.push_back(Polygon3D(points.at(5), points.at(0), points.at(54), lineColor, color));

	turrpolygons.push_back(Polygon3D(points.at(0), points.at(56), points.at(1), lineColor, color));
	turrpolygons.push_back(Polygon3D(points.at(1), points.at(56), points.at(57), lineColor, color));
	//
	turrpolygons.push_back(Polygon3D(points.at(1), points.at(57), points.at(2), lineColor, color));
	turrpolygons.push_back(Polygon3D(points.at(2), points.at(57), points.at(58), lineColor, color));
	//
	turrpolygons.push_back(Polygon3D(points.at(2), points.at(58), points.at(3), lineColor, color));
	turrpolygons.push_back(Polygon3D(points.at(3), points.at(58), points.at(59), lineColor, color));
	//
	turrpolygons.push_back(Polygon3D(points.at(3), points.at(59), points.at(4), lineColor, color));
	turrpolygons.push_back(Polygon3D(points.at(4), points.at(59), points.at(60), lineColor, color));
	//
	turrpolygons.push_back(Polygon3D(points.at(4), points.at(60), points.at(5), lineColor, color));
	turrpolygons.push_back(Polygon3D(points.at(5), points.at(60), points.at(61), lineColor, color));
	//
	turrpolygons.push_back(Polygon3D(points.at(5), points.at(61), points.at(0), lineColor, color));
	turrpolygons.push_back(Polygon3D(points.at(0), points.at(61), points.at(56), lineColor, color));


	turrpolygons.push_back(Polygon3D(points.at(56), points.at(62), points.at(57), lineColor, color));
	turrpolygons.push_back(Polygon3D(points.at(57), points.at(62), points.at(63), lineColor, color));
	//
	turrpolygons.push_back(Polygon3D(points.at(57), points.at(63), points.at(58), lineColor, color));
	turrpolygons.push_back(Polygon3D(points.at(58), points.at(63), points.at(64), lineColor, color));
	//
	turrpolygons.push_back(Polygon3D(points.at(58), points.at(64), points.at(59), lineColor, color));
	turrpolygons.push_back(Polygon3D(points.at(59), points.at(64), points.at(65), lineColor, color));
	//
	turrpolygons.push_back(Polygon3D(points.at(59), points.at(65), points.at(60), lineColor, color));
	turrpolygons.push_back(Polygon3D(points.at(60), points.at(65), points.at(66), lineColor, color));
	//
	turrpolygons.push_back(Polygon3D(points.at(60), points.at(66), points.at(61), lineColor, color));
	turrpolygons.push_back(Polygon3D(points.at(61), points.at(66), points.at(67), lineColor, color));
	//
	turrpolygons.push_back(Polygon3D(points.at(61), points.at(67), points.at(56), lineColor, color));
	turrpolygons.push_back(Polygon3D(points.at(56), points.at(67), points.at(62), lineColor, color));


	turrpolygons.push_back(Polygon3D(points.at(76), points.at(63), points.at(62), lineColor, color));
	turrpolygons.push_back(Polygon3D(points.at(76), points.at(64), points.at(63), lineColor, color));
	turrpolygons.push_back(Polygon3D(points.at(76), points.at(65), points.at(64), lineColor, color));
	turrpolygons.push_back(Polygon3D(points.at(76), points.at(66), points.at(65), lineColor, color));
	turrpolygons.push_back(Polygon3D(points.at(76), points.at(67), points.at(66), lineColor, color));
	turrpolygons.push_back(Polygon3D(points.at(76), points.at(62), points.at(67), lineColor, color));

	olc::Pixel gunCol = olc::Pixel(30, 30, 30);
	olc::Pixel gunLine = olc::BLACK;
	//gun
	std::vector<Polygon3D> gunpolygons;
	gunpolygons.push_back(Polygon3D(points.at(69), points.at(72), points.at(68), gunLine, gunCol));
	gunpolygons.push_back(Polygon3D(points.at(70), points.at(73), points.at(69), gunLine, gunCol));
	gunpolygons.push_back(Polygon3D(points.at(71), points.at(74), points.at(70), gunLine, gunCol));
	gunpolygons.push_back(Polygon3D(points.at(68), points.at(75), points.at(71), gunLine, gunCol));
	gunpolygons.push_back(Polygon3D(points.at(73), points.at(72), points.at(69), gunLine, gunCol));
	gunpolygons.push_back(Polygon3D(points.at(74), points.at(73), points.at(70), gunLine, gunCol));
	gunpolygons.push_back(Polygon3D(points.at(75), points.at(74), points.at(71), gunLine, gunCol));
	gunpolygons.push_back(Polygon3D(points.at(72), points.at(75), points.at(68), gunLine, gunCol));
	gunpolygons.push_back(Polygon3D(points.at(74), points.at(75), points.at(73), gunLine, olc::Pixel(10, 10, 10)));
	gunpolygons.push_back(Polygon3D(points.at(75), points.at(72), points.at(73), gunLine, olc::Pixel(10, 10, 10)));

	std::vector<ConvexHull*> turretHulls;
	std::vector<RigidSurface*> turretSurfaces;

	std::vector<Vector3D> ts1;
	ts1.push_back(points.at(0));
	ts1.push_back(points.at(1));
	ts1.push_back(points.at(2));
	ts1.push_back(points.at(3));
	ts1.push_back(points.at(4));
	ts1.push_back(points.at(5));
	turretSurfaces.push_back(new RigidSurface(&ts1, Vector3D(0, 1, 0)));

	std::vector<Vector3D> ts2;
	ts2.push_back(points.at(0));
	ts2.push_back(points.at(1));
	ts2.push_back(points.at(57));
	ts2.push_back(points.at(56));
	turretSurfaces.push_back(new RigidSurface(&ts2, Vector3D(1, 0, 0)));

	std::vector<Vector3D> ts3;
	ts3.push_back(points.at(1));
	ts3.push_back(points.at(2));
	ts3.push_back(points.at(58));
	ts3.push_back(points.at(57));
	turretSurfaces.push_back(new RigidSurface(&ts3, Vector3D(0, 0, -1)));

	std::vector<Vector3D> ts4;
	ts4.push_back(points.at(2));
	ts4.push_back(points.at(3));
	ts4.push_back(points.at(59));
	ts4.push_back(points.at(58));
	turretSurfaces.push_back(new RigidSurface(&ts4, Vector3D(-1, 0, 0)));

	std::vector<Vector3D> ts5;
	ts5.push_back(points.at(3));
	ts5.push_back(points.at(4));
	ts5.push_back(points.at(60));
	ts5.push_back(points.at(59));
	turretSurfaces.push_back(new RigidSurface(&ts5, Vector3D(-1, 0, 0)));

	std::vector<Vector3D> ts6;
	ts6.push_back(points.at(4));
	ts6.push_back(points.at(5));
	ts6.push_back(points.at(61));
	ts6.push_back(points.at(60));
	turretSurfaces.push_back(new RigidSurface(&ts6, Vector3D(0, 0, 1)));

	std::vector<Vector3D> ts7;
	ts7.push_back(points.at(5));
	ts7.push_back(points.at(0));
	ts7.push_back(points.at(56));
	ts7.push_back(points.at(61));
	turretSurfaces.push_back(new RigidSurface(&ts7, Vector3D(1, 0, 0)));

	std::vector<Vector3D> ts8;
	ts8.push_back(points.at(56));
	ts8.push_back(points.at(57));
	ts8.push_back(points.at(63));
	ts8.push_back(points.at(62));
	turretSurfaces.push_back(new RigidSurface(&ts8, Vector3D(1, 0, 0)));

	std::vector<Vector3D> ts9;
	ts9.push_back(points.at(57));
	ts9.push_back(points.at(58));
	ts9.push_back(points.at(64));
	ts9.push_back(points.at(63));
	turretSurfaces.push_back(new RigidSurface(&ts9, Vector3D(0, 0, -1)));

	std::vector<Vector3D> ts10;
	ts10.push_back(points.at(58));
	ts10.push_back(points.at(59));
	ts10.push_back(points.at(65));
	ts10.push_back(points.at(64));
	turretSurfaces.push_back(new RigidSurface(&ts10, Vector3D(-1, 0, 0)));

	std::vector<Vector3D> ts11;
	ts11.push_back(points.at(59));
	ts11.push_back(points.at(60));
	ts11.push_back(points.at(66));
	ts11.push_back(points.at(65));
	turretSurfaces.push_back(new RigidSurface(&ts11, Vector3D(-1, 0, 0)));

	std::vector<Vector3D> ts12;
	ts12.push_back(points.at(60));
	ts12.push_back(points.at(61));
	ts12.push_back(points.at(67));
	ts12.push_back(points.at(66));
	turretSurfaces.push_back(new RigidSurface(&ts12, Vector3D(0, 0, 1)));

	std::vector<Vector3D> ts13;
	ts13.push_back(points.at(61));
	ts13.push_back(points.at(56));
	ts13.push_back(points.at(62));
	ts13.push_back(points.at(67));
	turretSurfaces.push_back(new RigidSurface(&ts13, Vector3D(1, 0, 0)));

	std::vector<Vector3D> ts14;
	ts14.push_back(points.at(62));
	ts14.push_back(points.at(63));
	ts14.push_back(points.at(64));
	ts14.push_back(points.at(65));
	ts14.push_back(points.at(66));
	ts14.push_back(points.at(67));
	turretSurfaces.push_back(new RigidSurface(&ts14, Vector3D(0, -1, 0)));

	std::vector<Vector3D> gs;
	gs.push_back(points.at(68));
	gs.push_back(points.at(69));
	gs.push_back(points.at(70));
	gs.push_back(points.at(71));

	turretHulls.push_back(new ConvexHull(&turretSurfaces, 1));
	RigidBody* turretBody = new RigidBody(turretHulls, 1, 1, 0.3, true);
	PolyModel* turrPoly = new PolyModel(&turrpolygons, turretBody->getCenterOfMass());
	PhysicsObject* t1Body = new PhysicsObject(turrPoly, turretBody);

	std::vector<ConvexHull*> gunHull;
	std::vector<RigidSurface*> gunSurfaces;
	PhysicsObject::createSurfacesFromPolygons(&gunpolygons, &gunSurfaces);
	gunSurfaces.push_back(new RigidSurface(&gs, Vector3D(0, 0, -1)));
	gunHull.push_back(new ConvexHull(&gunSurfaces, 1));
	RigidBody* gunBody = new RigidBody(gunHull, 1, 1, 0.3, true, true, turretBody->getCenterOfMass());
	PolyModel* gunModel = new PolyModel(&gunpolygons, gunBody->getCenterOfMass());
	PhysicsObject* gunObj = new PhysicsObject(gunModel, gunBody);

	turret1 = new Turret(t1Body, gunObj, Vector3D(0, -1, 0));

	PhysicsObject* body2 = new PhysicsObject(*t1Body);
	PhysicsObject* gun2 = new PhysicsObject(*gunObj);

	turret2 = new Turret(body2, gun2, Vector3D(0, 1, 0));
	Vector3D translation = points.at(55).sub(points.at(54)).add(points.at(54).sub(turretBody->getCenterOfMass()).multiply(2));
	turret2->gun->getRigidBody()->translate(translation);
	turret2->body->getRigidBody()->translate(translation);
	Rotor orientation = Rotor(Vector3D(0, 0, 1), 3.141592);
	turret2->gun->getRigidBody()->setToOrientation(orientation);
	turret2->body->getRigidBody()->setToOrientation(orientation);

	std::vector<ConvexHull*> hulls;
	std::vector<RigidSurface*> surfaces;
	
	std::vector<Vector3D> s1;
	s1.push_back(points.at(0));
	s1.push_back(points.at(1));
	s1.push_back(points.at(2));
	s1.push_back(points.at(3));
	s1.push_back(points.at(4));
	s1.push_back(points.at(5));
	surfaces.push_back(new RigidSurface(&s1, Vector3D(0, -1, 0)));

	std::vector<Vector3D> s2;
	s2.push_back(points.at(0));
	s2.push_back(points.at(1));
	s2.push_back(points.at(7));
	s2.push_back(points.at(6));
	surfaces.push_back(new RigidSurface(&s2, Vector3D(0, -1, 0)));

	std::vector<Vector3D> s3;
	s3.push_back(points.at(1));
	s3.push_back(points.at(2));
	s3.push_back(points.at(8));
	s3.push_back(points.at(7));
	surfaces.push_back(new RigidSurface(&s3, Vector3D(0, -1, 0)));

	std::vector<Vector3D> s4;
	s4.push_back(points.at(2));
	s4.push_back(points.at(3));
	s4.push_back(points.at(9));
	s4.push_back(points.at(8));
	surfaces.push_back(new RigidSurface(&s4, Vector3D(0, -1, 0)));

	std::vector<Vector3D> s5;
	s5.push_back(points.at(3));
	s5.push_back(points.at(4));
	s5.push_back(points.at(10));
	s5.push_back(points.at(9));
	surfaces.push_back(new RigidSurface(&s5, Vector3D(0, -1, 0)));

	std::vector<Vector3D> s6;
	s6.push_back(points.at(4));
	s6.push_back(points.at(5));
	s6.push_back(points.at(11));
	s6.push_back(points.at(10));
	surfaces.push_back(new RigidSurface(&s6, Vector3D(0, -1, 0)));

	std::vector<Vector3D> s7;
	s7.push_back(points.at(5));
	s7.push_back(points.at(0));
	s7.push_back(points.at(6));
	s7.push_back(points.at(11));
	surfaces.push_back(new RigidSurface(&s7, Vector3D(0, -1, 0)));

	//

	std::vector<Vector3D> s8;
	s8.push_back(points.at(6));
	s8.push_back(points.at(7));
	s8.push_back(points.at(37));
	s8.push_back(points.at(36));
	surfaces.push_back(new RigidSurface(&s8, Vector3D(1, 0, 0)));

	std::vector<Vector3D> s9;
	s9.push_back(points.at(7));
	s9.push_back(points.at(8));
	s9.push_back(points.at(38));
	s9.push_back(points.at(37));
	surfaces.push_back(new RigidSurface(&s9, Vector3D(0, 0, -1)));

	std::vector<Vector3D> s10;
	s10.push_back(points.at(8));
	s10.push_back(points.at(9));
	s10.push_back(points.at(39));
	s10.push_back(points.at(38));
	surfaces.push_back(new RigidSurface(&s10, Vector3D(-1, 0, 0)));

	std::vector<Vector3D> s11;
	s11.push_back(points.at(9));
	s11.push_back(points.at(10));
	s11.push_back(points.at(40));
	s11.push_back(points.at(39));
	surfaces.push_back(new RigidSurface(&s11, Vector3D(-1, 0, 0)));

	std::vector<Vector3D> s12;
	s12.push_back(points.at(10));
	s12.push_back(points.at(11));
	s12.push_back(points.at(41));
	s12.push_back(points.at(40));
	surfaces.push_back(new RigidSurface(&s12, Vector3D(0, 0, 1)));

	std::vector<Vector3D> s13;
	s13.push_back(points.at(11));
	s13.push_back(points.at(6));
	s13.push_back(points.at(36));
	s13.push_back(points.at(41));
	surfaces.push_back(new RigidSurface(&s13, Vector3D(1, 0, 0)));

	//

	std::vector<Vector3D> s14;
	s14.push_back(points.at(36));
	s14.push_back(points.at(37));
	s14.push_back(points.at(43));
	s14.push_back(points.at(42));
	surfaces.push_back(new RigidSurface(&s14, Vector3D(0, 1, 0)));

	std::vector<Vector3D> s15;
	s15.push_back(points.at(37));
	s15.push_back(points.at(38));
	s15.push_back(points.at(44));
	s15.push_back(points.at(43));
	surfaces.push_back(new RigidSurface(&s15, Vector3D(0, 1, 0)));

	std::vector<Vector3D> s16;
	s16.push_back(points.at(38));
	s16.push_back(points.at(39));
	s16.push_back(points.at(45));
	s16.push_back(points.at(44));
	surfaces.push_back(new RigidSurface(&s16, Vector3D(0, 1, 0)));

	std::vector<Vector3D> s17;
	s17.push_back(points.at(39));
	s17.push_back(points.at(40));
	s17.push_back(points.at(46));
	s17.push_back(points.at(45));
	surfaces.push_back(new RigidSurface(&s17, Vector3D(0, 1, 0)));

	std::vector<Vector3D> s18;
	s18.push_back(points.at(40));
	s18.push_back(points.at(41));
	s18.push_back(points.at(47));
	s18.push_back(points.at(46));
	surfaces.push_back(new RigidSurface(&s18, Vector3D(0, 1, 0)));

	std::vector<Vector3D> s19;
	s19.push_back(points.at(41));
	s19.push_back(points.at(36));
	s19.push_back(points.at(42));
	s19.push_back(points.at(47));
	surfaces.push_back(new RigidSurface(&s19, Vector3D(0, 1, 0)));

	//

	std::vector<Vector3D> s20;
	s20.push_back(points.at(42));
	s20.push_back(points.at(43));
	s20.push_back(points.at(44));
	s20.push_back(points.at(45));
	s20.push_back(points.at(46));
	s20.push_back(points.at(47));
	surfaces.push_back(new RigidSurface(&s20, Vector3D(0, 1, 0), true));

	hulls.push_back(new ConvexHull(&surfaces, 1));
	//
	std::vector<RigidSurface*> botHull;

	std::vector<Vector3D> s21;
	s21.push_back(points.at(42));
	s21.push_back(points.at(43));
	s21.push_back(points.at(44));
	s21.push_back(points.at(45));
	s21.push_back(points.at(46));
	s21.push_back(points.at(47));
	botHull.push_back(new RigidSurface(&s21, Vector3D(0, -1, 0), true));

	std::vector<Vector3D> s22;
	s22.push_back(points.at(42));
	s22.push_back(points.at(43));
	s22.push_back(points.at(49));
	s22.push_back(points.at(48));
	botHull.push_back(new RigidSurface(&s22, Vector3D(1, 0, 0)));

	std::vector<Vector3D> s23;
	s23.push_back(points.at(43));
	s23.push_back(points.at(44));
	s23.push_back(points.at(50));
	s23.push_back(points.at(49));
	botHull .push_back(new RigidSurface(&s23, Vector3D(0, 0, -1)));

	std::vector<Vector3D> s24;
	s24.push_back(points.at(44));
	s24.push_back(points.at(45));
	s24.push_back(points.at(51));
	s24.push_back(points.at(50));
	botHull.push_back(new RigidSurface(&s24, Vector3D(-1, 0, 0)));

	std::vector<Vector3D> s25;
	s25.push_back(points.at(45));
	s25.push_back(points.at(46));
	s25.push_back(points.at(52));
	s25.push_back(points.at(51));
	botHull.push_back(new RigidSurface(&s25, Vector3D(-1, 0, 0)));

	std::vector<Vector3D> s26;
	s26.push_back(points.at(46));
	s26.push_back(points.at(47));
	s26.push_back(points.at(53));
	s26.push_back(points.at(52));
	botHull.push_back(new RigidSurface(&s26, Vector3D(0, 0, 1)));

	std::vector<Vector3D> s27;
	s27.push_back(points.at(47));
	s27.push_back(points.at(42));
	s27.push_back(points.at(48));
	s27.push_back(points.at(53));
	botHull.push_back(new RigidSurface(&s27, Vector3D(1, 0, 0)));

	std::vector<Vector3D> s28;
	s28.push_back(points.at(48));
	s28.push_back(points.at(49));
	s28.push_back(points.at(50));
	s28.push_back(points.at(51));
	s28.push_back(points.at(52));
	s28.push_back(points.at(53));
	botHull.push_back(new RigidSurface(&s28, Vector3D(0, 1, 0)));
	
	hulls.push_back(new ConvexHull(&botHull, 1));

	RigidBody* structure = new RigidBody(hulls, 1, 1, 0.3, true);
	PolyModel* model = new PolyModel(&polygons, structure->getCenterOfMass());
	mainStructure = new PhysicsObject(model, structure);
}
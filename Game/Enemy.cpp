#include "Enemy.h"
#include "Player.h"
#include "Bullet.h"
#include "Missile.h"
#include "SpaceMinerGame.h"

Enemy::Enemy(Vector3D pos, Rotor orientation) {
	//createStructure();
	MovingObject::createShipMesh(false, size, olc::VERY_DARK_MAGENTA, olc::BLACK, 
		olc::VERY_DARK_MAGENTA, olc::BLACK, olc::DARK_GREEN, olc::VERY_DARK_GREEN, &body, &model);
	body->setToOrientation(orientation);
	body->translate(pos);

	maxHp = 100;
	hp = maxHp;

	angularDampFactor = 0.8; 
	linearDampFactor = 0.12;

	findIVals();
	yawRate = 0.5;
	pitchRate = 1.7;
	rollRate = 0.5;
	forwardThrust = 550;


	std::vector<Vector3D>* potentialSmokePoints = body->getAllPoints();
	int index = rand() % potentialSmokePoints->size();
	smokePoint = potentialSmokePoints->at(index);
}

void Enemy::update(SpaceMinerGame* game, float fElapsedTime) {
	dampenMotion(fElapsedTime);
	shootTimer.updateTimer(fElapsedTime);
	missileTimer.updateTimer(fElapsedTime);

	Player* p = game->getPlayer();

	Vector3D dir = getDir();
	Vector3D toPlayer = p->getPos().sub(getPos());
	Vector3D toPlayerUnit = toPlayer.getUnitVector();

	accelForward(1, fElapsedTime);

	if (dir.dotProduct(toPlayerUnit) > fireCone) {
		shoot(game);
	}

	Vector3D leadTarget = Projectile::calculateLeadPoint(getPos(), p->getPos(), body->getVelocity(), p->getRigidBody()->getVelocity(), bulletVel);

	aimAtTarget(leadTarget, acos(dir.dotProduct(leadTarget.getUnitVector())), fElapsedTime);

	if (hp / maxHp < 0.5) {
		Vector3D smokePWorld = getPos().add(body->getOrientation().rotate(smokePoint));
		Vector3D originVar = Vector3D((float)rand() / RAND_MAX, (float)rand() / RAND_MAX, (float)rand() / RAND_MAX).multiply(0.75 * size);
		Particle::generateExplosion(game, smokePWorld.add(originVar), 0.7, 50, 3, olc::RED);
		Particle::generateExplosion(game, smokePWorld.add(originVar), 0.9, 100, 1, olc::GREEN);
		Particle::generateExplosion(game, smokePWorld.add(originVar), 4.5, 250, 3, olc::DARK_GREY);
	}
}

void Enemy::aimAtTarget(Vector3D relPos, double angleOff, float fElapsedTime) {
	Vector3D pitchDir = body->getOrientation().rotate(Vector3D(0, 1, 0));
	Vector3D yawDir = body->getOrientation().rotate(Vector3D(0, 0, -1));
	
	if (relPos.dotProduct(yawDir) > 0) {
		yaw(-1, fElapsedTime);
	}
	else {
		yaw(1, fElapsedTime);
	}

	if (relPos.dotProduct(pitchDir) > 0) {
		pitch(-1, fElapsedTime);
	}
	else {
		pitch(0.5, fElapsedTime);
	}

	if (angleOff > 3.14159 / 9) {
		if (relPos.dotProduct(yawDir) > 0) {
			roll(1, fElapsedTime);
		}
		else {
			roll(-1, fElapsedTime);
		}
	}

}

void Enemy::damage(double damage) {
	Damageable::damage(damage);
}

void Enemy::performDeathActions(SpaceMinerGame* game) {
	Particle::generateExplosion(game, getPos(), 0.9, 1550, 250, olc::MAGENTA);
	Particle::generateExplosion(game, getPos(), 1.0, 1250, 250, olc::GREEN);
	Particle::generateExplosion(game, getPos(), 1.1, 750, 170, olc::RED);

	for (int i = 0; i < 3; i++) {
		double speed = 350;
		Vector3D dir = Vector3D(0.5 - (float)rand() / RAND_MAX, 0.5 - (float)rand() / RAND_MAX, 0.5 - (float)rand() / RAND_MAX).getUnitVector();
		Vector3D vel = body->getVelocity().multiply(1.0 / 3).add(dir.multiply(speed));
		Vector3D rot = Vector3D(0.5 - (float)rand() / RAND_MAX, 0.5 - (float)rand() / RAND_MAX, 0.5 - (float)rand() / RAND_MAX).getUnitVector().multiply(5);

		Ore* ore = new Ore(getPos(), Ore::NormalDebris, (i == 0)? HeadDebris : WingDebris);
		ore->getRigidBody()->setVelocity(vel);
		ore->getRigidBody()->setAngVelocity(rot);
		game->addOre(ore);

	}
	game->getPhysicsEngine()->removeRigidBody(body);
}

void Enemy::shoot(SpaceMinerGame* game) {

	if (false && missileTimer.isReady()) {
		missileTimer.reset();
		shootTimer.setCustum(2.0);

		Vector3D dir = body->getOrientation().rotate(Vector3D(1, 0, 0));
		double spawnDist = 150;

		double launchVel = 900;
		Vector3D launchDir = body->getOrientation().rotate(Vector3D(0, 1, 0)).multiply(launchVel);
		Missile* missile = new Missile(body->getCenterOfMass(), body->getOrientation(), body->getVelocity().add(launchDir), game->getPlayer(), body->getID(), false);
		game->addProjectile(missile);
	}
	else if (shootTimer.isReady()) {
		shootTimer.reset();

		Vector3D dir = body->getOrientation().rotate(Vector3D(1, 0, 0));
		double spawnDist = 150;
		Bullet* bullet = new Bullet(body->getCenterOfMass().add(dir.multiply(spawnDist)), body->getOrientation(), body->getVelocity().add(dir.multiply(bulletVel)), 150, body->getID(), olc::GREEN, olc::WHITE);

		//Missile* missile = new Missile(body->getCenterOfMass(), body->getOrientation(), body->getVelocity(), game->getPlayer(), body->getID());
		game->addProjectile(bullet);
	}
	
}

/*void Enemy::draw(PixelEngine3D* g, Vector3D cameraPos, Rotor cameraDir, double FOV) {
	debugDraw(g, cameraPos, cameraDir, FOV);
}*/
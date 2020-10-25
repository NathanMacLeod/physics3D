#include "Bullet.h"
#include "Particle.h"
#include "SpaceMinerGame.h"

Bullet::Bullet(Vector3D position, Rotor dir, Vector3D velocity, double density, uint16_t sourceID, olc::Pixel lineColor, olc::Pixel color, double lifeTime) {

	life = CooldownTimer(lifeTime);

	double l = 20;
	double w = 5;

	Vector3D p1(0, -w, -w);
	Vector3D p2(0, -w, w);
	Vector3D p3(0, w, w);
	Vector3D p4(0, w, -w);
	Vector3D p5(l, 0, 0);
	Vector3D p6(-l, 0, 0);

	std::vector<Polygon3D> polygons;

	polygons.push_back(Polygon3D(p4, p5, p1, lineColor, color));
	polygons.push_back(Polygon3D(p1, p5, p2, lineColor, color));
	polygons.push_back(Polygon3D(p2, p5, p3, lineColor, color));
	polygons.push_back(Polygon3D(p3, p5, p1, lineColor, color));

	polygons.push_back(Polygon3D(p1, p6, p4, lineColor, color));
	polygons.push_back(Polygon3D(p2, p6, p1, lineColor, color));
	polygons.push_back(Polygon3D(p3, p6, p2, lineColor, color));
	polygons.push_back(Polygon3D(p1, p6, p3, lineColor, color));

	std::vector<ConvexHull*> hulls;
	std::vector<RigidSurface*> surfaces;

	std::vector<Vector3D> s1points;
	s1points.push_back(p1);
	s1points.push_back(p2);
	s1points.push_back(p5);
	surfaces.push_back(new RigidSurface(&s1points, Vector3D(0, -1, 0)));

	std::vector<Vector3D> s2points;
	s2points.push_back(p2);
	s2points.push_back(p3);
	s2points.push_back(p5);
	surfaces.push_back(new RigidSurface(&s2points, Vector3D(0, 0, 1)));

	std::vector<Vector3D> s3points;
	s3points.push_back(p3);
	s3points.push_back(p4);
	s3points.push_back(p5);
	surfaces.push_back(new RigidSurface(&s3points, Vector3D(0, 1, 0)));

	std::vector<Vector3D> s4points;
	s4points.push_back(p4);
	s4points.push_back(p1);
	s4points.push_back(p6);
	surfaces.push_back(new RigidSurface(&s4points, Vector3D(0, 0, -1)));

	std::vector<Vector3D> s5points;
	s5points.push_back(p1);
	s5points.push_back(p2);
	s5points.push_back(p6);
	surfaces.push_back(new RigidSurface(&s5points, Vector3D(0, -1, 0)));

	std::vector<Vector3D> s6points;
	s6points.push_back(p2);
	s6points.push_back(p3);
	s6points.push_back(p6);
	surfaces.push_back(new RigidSurface(&s6points, Vector3D(0, 0, 1)));

	std::vector<Vector3D> s7points;
	s7points.push_back(p3);
	s7points.push_back(p4);
	s7points.push_back(p6);
	surfaces.push_back(new RigidSurface(&s7points, Vector3D(0, 1, 0)));

	std::vector<Vector3D> s8points;
	s8points.push_back(p4);
	s8points.push_back(p1);
	s8points.push_back(p6);
	surfaces.push_back(new RigidSurface(&s8points, Vector3D(0, 0, -1)));

	hulls.push_back(new ConvexHull(&surfaces, density));

	body = new RigidBody(hulls, 1, 1, 0.3, false);
	model = new PolyModel(&polygons, body->getCenterOfMass());

	body->setToOrientation(dir);
	body->translate(position);

	body->setVelocity(velocity);
	body->setAngVelocity(dir.rotate(Vector3D(1, 0, 0)).multiply(16));

	Vector3D angVelRel = body->findVectorRelativeToBodyFrame(body->getAngularVelocity());

	body->setTrackCollHistory(true);
	body->addNoCol(sourceID);
	life.reset();
}

Bullet::~Bullet() {

}

void Bullet::draw(PixelEngine3D* g, Vector3D cameraPos, Rotor cameraDir, double FOV) {
	PhysicsObject::draw(g, cameraPos, cameraDir, FOV);
}

bool Bullet::isExpired() {
	return life.isReady() || collided;
}

RigidBody* Bullet::getBody() {
	return body;
}

void Bullet::performDeathActions(SpaceMinerGame* game) {
	game->getPhysicsEngine()->removeRigidBody(body);
}

void Bullet::update(SpaceMinerGame* game, float fElapsedTime) {
	life.updateTimer(fElapsedTime);

	std::vector<CollisionInfo>* colls = body->getCollHistory();
	if (colls->size() > 0) {
		int collID = colls->at(0).otherBodyID;

		for (Enemy* e : *game->getEnemies()) {
			if (e->getRigidBody()->getID() == collID) {
				e->damage(35);
				break;
			}
		}

		for (Asteroid* a : *game->getAsteroids()) {
			if (a->getRigidBody()->getID() == collID) {
				a->damage(35);
				break;
			}
		}

		Particle::generateExplosion(game, body->getCenterOfMass(), 0.5, 350, 50, olc::WHITE);
		collided = true;
		return;
	}
}
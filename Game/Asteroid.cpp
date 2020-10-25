#include "Asteroid.h"
#include "SpaceMinerGame.h"

Asteroid::Asteroid(Vector3D position, double size, int detail) {
	this->size = size;
	maxHp = 30 * 300 * size * sqrt(size) / (MIN_SIZE * MIN_SIZE);
	hp = maxHp;
	createRockMesh(position, size, detail, 1, 0.2, olc::WHITE, olc::BLACK, &body, &model);
}

void Asteroid::createRockMesh(Vector3D pos, double size, double detail, double density, double roughness, olc::Pixel lineColor, olc::Pixel color, RigidBody** bodyOut, PolyModel** meshOut) {
	//form cube, then 

	Vector3D p1(-1, -1, -1);
	Vector3D p2(1, -1, -1);
	Vector3D p3(1, -1, 1);
	Vector3D p4(-1, -1, 1);
	Vector3D p5(-1, 1, -1);
	Vector3D p6(1, 1, -1);
	Vector3D p7(1, 1, 1);
	Vector3D p8(-1, 1, 1);

	std::vector<Vector3D*> allP;

	std::vector<Vector3D*> f1;
	generateFace(p1, p2, p3, p4, detail, &f1, &allP);

	std::vector<Vector3D*> f2;
	generateFace(p5, p6, p7, p8, detail, &f2, &allP);

	std::vector<Vector3D*> f3;
	generateFace(p1, p2, p6, p5, detail, &f3, &allP);

	std::vector<Vector3D*> f4;
	generateFace(p2, p3, p7, p6, detail, &f4, &allP);

	std::vector<Vector3D*> f5;
	generateFace(p3, p4, p8, p7, detail, &f5, &allP);

	std::vector<Vector3D*> f6;
	generateFace(p4, p1, p5, p8, detail, &f6, &allP);

	for (Vector3D* p : allP) {
		//random var
		float randFactor = 1 - 2 * ((float)rand() / (float)RAND_MAX);
		//double r = size * abs(p->getUnitVector().dotProduct(Vector3D(0, -1, 0))) * (1 + randMag * randFactor);
		double r = size * (1 + roughness * randFactor);

		*p = pos.add(p->getUnitVector().multiply(r));
	}

	std::vector<Polygon3D> polygons;
	generatePolygons(&f1, &polygons, pos, detail, lineColor, color);
	generatePolygons(&f2, &polygons, pos, detail, lineColor, color);
	generatePolygons(&f3, &polygons, pos, detail, lineColor, color);
	generatePolygons(&f4, &polygons, pos, detail, lineColor, color);
	generatePolygons(&f5, &polygons, pos, detail, lineColor, color);
	generatePolygons(&f6, &polygons, pos, detail, lineColor, color);


	std::vector<ConvexHull*> hulls;

	//std::vector<RigidSurface*> surfaces;
	//createSurfacesFromPolygons(&polygons, &surfaces);
	//hulls->push_back(new ConvexHull(&surfaces, 1));

	for (Polygon3D p : polygons) {
		std::vector<RigidSurface*> surfaces;

		std::vector<Vector3D> s1points;
		s1points.push_back(pos);
		s1points.push_back(p.p1);
		s1points.push_back(p.p2);
		surfaces.push_back(new RigidSurface(&s1points, p.p1.sub(p.p3)));

		std::vector<Vector3D> s2points;
		s2points.push_back(pos);
		s2points.push_back(p.p2);
		s2points.push_back(p.p3);
		surfaces.push_back(new RigidSurface(&s2points, p.p2.sub(p.p1)));

		std::vector<Vector3D> s3points;
		s3points.push_back(pos);
		s3points.push_back(p.p3);
		s3points.push_back(p.p1);
		surfaces.push_back(new RigidSurface(&s3points, p.p3.sub(p.p2)));

		std::vector<Vector3D> s4points;
		s4points.push_back(p.p1);
		s4points.push_back(p.p2);
		s4points.push_back(p.p3);
		surfaces.push_back(new RigidSurface(&s4points, p.p1.sub(pos)));

		hulls.push_back(new ConvexHull(&surfaces, 1));
	}

	*bodyOut = new RigidBody(hulls, 1, 1, 0.3, false);
	*meshOut = new PolyModel(&polygons, (*bodyOut)->getCenterOfMass());

	for (Vector3D* p : allP) {
		delete p;
	}
}

void Asteroid::generatePolygons(std::vector<Vector3D*>* face, std::vector<Polygon3D>* polygons, Vector3D center, int detail, olc::Pixel lineColor, olc::Pixel color) {
	Vector3D* testp1 = face->at(0);
	Vector3D* testp2 = face->at(1);
	Vector3D* testp4 = face->at(detail + 1);

	bool reverseWinding = false;
	if (testp2->sub(*testp1).crossProduct(testp4->sub(*testp1)).dotProduct(testp1->sub(center)) > 0) {
		reverseWinding = true;
	}

	for (int i = 0; i < detail; i++) {
		for (int j = 0; j < detail; j++) {
			Vector3D* p1 = face->at(i + j * (detail + 1));
			Vector3D* p2 = face->at((i + 1) + j * (detail + 1));
			Vector3D* p3 = face->at((i + 1) + (j + 1) * (detail + 1));
			Vector3D* p4 = face->at(i + (j + 1) * (detail + 1));

			if (reverseWinding) {
				polygons->push_back(Polygon3D(*p1, *p2, *p4, lineColor, color));
				polygons->push_back(Polygon3D(*p2, *p3, *p4, lineColor, color));
			}
			else {
				polygons->push_back(Polygon3D(*p4, *p2, *p1, lineColor, color));
				polygons->push_back(Polygon3D(*p4, *p3, *p2, lineColor, color));
			}
		}
	}
}

//p1p2 p1p4 should be orthogonal
void Asteroid::generateFace(Vector3D p1, Vector3D p2, Vector3D p3, Vector3D p4, int detail, std::vector<Vector3D*>* face_out, std::vector<Vector3D*>* allP_out) {
	Vector3D p1p2 = p2.sub(p1).multiply(1.0/detail);
	Vector3D p1p4 = p4.sub(p1).multiply(1.0 / detail);
	for (int i = 0; i <= detail; i++) {
		for (int j = 0; j <= detail; j++) {
			Vector3D p = p1.add(p1p2.multiply(i)).add(p1p4.multiply(j));
			bool pAlreadyAdded = false;
			for (Vector3D* existingP : *allP_out) {
				if (*existingP == p) {
					face_out->push_back(existingP);
					pAlreadyAdded = true;
					break;
				}
			}
			if (!pAlreadyAdded) {
				Vector3D* newP = new Vector3D(p.x, p.y, p.z);
				face_out->push_back(newP);
				allP_out->push_back(newP);
			}
		}
	}
}

double Asteroid::pickAsteroidSize() {
	float val = (float)rand() / RAND_MAX;
	return val * val * val * (MAX_SIZE - MIN_SIZE) + MIN_SIZE;
}

void Asteroid::performDeathActions(SpaceMinerGame* game) {
	Particle::generateExplosion(game, getPos(), 0.9, 1550, 350, olc::WHITE);
	
	int nOre = std::max<int>(2, 30 * size * sqrt(size) / (MIN_SIZE * MIN_SIZE));

	for (int i = 0; i < nOre; i++) {
		double speed = 750 * (float)rand() / RAND_MAX;
		Vector3D dir = Vector3D(0.5 - (float)rand() / RAND_MAX, 0.5 - (float)rand() / RAND_MAX, 0.5 - (float)rand() / RAND_MAX).getUnitVector();
		Vector3D vel = dir.multiply(speed);
		Vector3D displacment = dir.multiply(size * (0.25 + 0.35 * (float)rand() / RAND_MAX));
		Vector3D rot = Vector3D(0.5 - (float)rand() / RAND_MAX, 0.5 - (float)rand() / RAND_MAX, 0.5 - (float)rand() / RAND_MAX).getUnitVector().multiply(5);

		Ore* ore = new Ore(body->getCenterOfMass().add(displacment), Ore::pickTypeRandomly());
		ore->getRigidBody()->setVelocity(vel);
		ore->getRigidBody()->setAngVelocity(rot);
		game->addOre(ore);
		
	}

	game->getPhysicsEngine()->removeRigidBody(body);
}
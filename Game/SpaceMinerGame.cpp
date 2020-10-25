#define OLC_PGE_APPLICATION
#include "SpaceMinerGame.h"
#include "Player.h"
#include "Windows.h"

//ScreenHeight()
//ScreenWidth()
//Draw(x, y, olc::Pixel(r, g, b));

std::vector<Polygon3D>* createBox(double x, double y, double z, double xPos, double yPos, double zPos, olc::Pixel color, olc::Pixel lineColor) {
	std::vector<Polygon3D>* polygons = new std::vector<Polygon3D>();
	Vector3D p1 = Vector3D(-x / 2.0, -y / 2.0, -z / 2.0);
	Vector3D p2 = Vector3D(-x / 2.0, y / 2.0, -z / 2.0);
	Vector3D p3 = Vector3D(x / 2.0, -y / 2.0, -z / 2.0);
	Vector3D p4 = Vector3D(x / 2.0, y / 2.0, -z / 2.0);
	Vector3D p5 = Vector3D(-x / 2.0, -y / 2.0, z / 2.0);
	Vector3D p6 = Vector3D(-x / 2.0, y / 2.0, z / 2.0);
	Vector3D p7 = Vector3D(x / 2.0, -y / 2.0, z / 2.0);
	Vector3D p8 = Vector3D(x / 2.0, y / 2.0, z / 2.0);

	Vector3D translation(xPos, yPos, zPos);
	std::vector<Vector3D*> points;
	points.push_back(&p1);
	points.push_back(&p2);
	points.push_back(&p3);
	points.push_back(&p4);
	points.push_back(&p5);
	points.push_back(&p6);
	points.push_back(&p7);
	points.push_back(&p8);
	transformation3D::translatePoints(&points, translation);

	polygons->push_back(Polygon3D(p1, p2, p3, lineColor, color));
	polygons->push_back(Polygon3D(p2, p4, p3, lineColor, color));

	polygons->push_back(Polygon3D(p5, p7, p6, lineColor, color));
	polygons->push_back(Polygon3D(p6, p7, p8, lineColor, color));

	polygons->push_back(Polygon3D(p3, p4, p7, lineColor, color));
	polygons->push_back(Polygon3D(p7, p4, p8, lineColor, color));

	polygons->push_back(Polygon3D(p1, p5, p2, lineColor, color));
	polygons->push_back(Polygon3D(p5, p6, p2, lineColor, color));

	polygons->push_back(Polygon3D(p2, p6, p4, lineColor, color));
	polygons->push_back(Polygon3D(p6, p8, p4, lineColor, color));

	polygons->push_back(Polygon3D(p1, p3, p5, lineColor, color));
	polygons->push_back(Polygon3D(p5, p3, p7, lineColor, color));

	return polygons;
}

std::vector<RigidSurface*>* createRigidBodyFromPolygons(std::vector<Polygon3D>* polygons) {
	std::vector<RigidSurface*>* surfaces = new std::vector<RigidSurface*>();
	for (Polygon3D polygon : *polygons) {
		Vector3D v1 = polygon.p2.sub(polygon.p1);
		Vector3D v2 = polygon.p3.sub(polygon.p1); 
		Vector3D normalVector = (v1.crossProduct(v2)).getUnitVector();
		std::vector<Vector3D> points;
		points.push_back(polygon.p1);
		points.push_back(polygon.p2);
		points.push_back(polygon.p3);
		RigidSurface* surface = new RigidSurface(&points, normalVector);
		surfaces->push_back(surface);
	}
	return surfaces;
}

bool SpaceMinerGame::OnUserCreate() {

	initZBuffer();
	
	int worldSize = 3000;

	cameraPos = Vector3D(0, -300, -250);

	{
		std::vector<Polygon3D>* floorPolygons = createBox(2500, 10, 2500, 0, -1, 0, olc::DARK_RED, olc::BLACK);
		std::vector<ConvexHull*> hulls;
		hulls.push_back(new ConvexHull(createRigidBodyFromPolygons(floorPolygons), 1));
		RigidBody* floor = new RigidBody(hulls, 1, 1, 0.3, true);
		PolyModel* model = new PolyModel(floorPolygons, floor->getCenterOfMass());

		objects.push_back(new PhysicsObject(model, floor));
		pEngine.addRigidBody(floor);
	}
	double x = 0;// 52;
	double y = -300;
	double z = 00;
	std::vector<Polygon3D>* b1polygons = createBox(40, 100, 40, x, y, z, olc::MAGENTA, olc::WHITE);
	std::vector<Polygon3D>* b2polygons = createBox(280, 60, 60, x, y - 50, z, olc::MAGENTA, olc::WHITE);
	std::vector<ConvexHull*>* hulls = new std::vector<ConvexHull*>();
	hulls->push_back(new ConvexHull(createRigidBodyFromPolygons(b1polygons), 1));
	hulls->push_back(new ConvexHull(createRigidBodyFromPolygons(b2polygons), 1));
	RigidBody* b1 = new RigidBody(*hulls, 1, 1, 0.3, false);
	int v = 20;
	b1->setToOrientation(Rotor(Vector3D(0, 0, 1), 3.14159 / 2.0));
	b1->setAngVelocity(Vector3D(5, 0.1, 0).multiply(1.5));

	std::vector<Polygon3D> polygons;

	for (Polygon3D p : *b1polygons) {
		polygons.push_back(p);
	}
	for (Polygon3D p : *b2polygons) {
		polygons.push_back(p);
	}

	objects.push_back(new PhysicsObject(new PolyModel(&polygons, b1->getCenterOfMass()), b1));

	delete b1polygons;
	delete b2polygons;

	pEngine.addRigidBody(b1);

	pEngine.setGravity(Vector3D(0, 160, 0));
	pEngine.setOctree(true, Vector3D(0, 0, 0), worldSize * 2, 300);


	return true;
}

void SpaceMinerGame::readInput(float fElapsedTime) {
	
	Vector3D forwardDir = cameraOrientation.rotate(Vector3D(0, 0, 1));
	forwardDir = Vector3D(forwardDir.x, 0, forwardDir.z).getUnitVector();
	Vector3D strafeDir = cameraOrientation.rotate(Vector3D(1, 0, 0));

	double moveSpeed = 150;
	double rotateSpeed = 0.75;

	if (GetKey(olc::Key::W).bHeld) {
		cameraPos = cameraPos.add(forwardDir.multiply(moveSpeed * fElapsedTime));
	}
	if (GetKey(olc::Key::S).bHeld) {
		cameraPos = cameraPos.add(forwardDir.multiply(moveSpeed * -fElapsedTime));
	}
	if (GetKey(olc::Key::A).bHeld) {
		cameraPos = cameraPos.add(strafeDir.multiply(moveSpeed * -fElapsedTime));
	}
	if (GetKey(olc::Key::D).bHeld) {
		cameraPos = cameraPos.add(strafeDir.multiply(moveSpeed * fElapsedTime));
	}
	if (GetKey(olc::Key::SPACE).bHeld) {
		cameraPos = cameraPos.add(Vector3D(0, 1, 0).multiply(moveSpeed * -fElapsedTime));
	}
	if (GetKey(olc::Key::CTRL).bHeld) {
		cameraPos = cameraPos.add(Vector3D(0, 1, 0).multiply(moveSpeed * fElapsedTime));
	}
	if (GetKey(olc::Key::K).bPressed) {
		std::vector<Polygon3D> polygons;

		int x = 0;
		int y = -300;
		int z = 0;

		std::vector<Polygon3D>* b1Polygons = createBox(75, 75, 75, x, y, z, olc::GREEN, olc::BLACK);
		std::vector<ConvexHull*> hulls;
		hulls.push_back(new ConvexHull(createRigidBodyFromPolygons(b1Polygons), 1));
		for (Polygon3D p: *b1Polygons) {
			polygons.push_back(p);
		}

		/*std::vector<Polygon3D>* b2Polygons = createBox(50, 20, 50, x, y - 30, z, olc::DARK_BLUE, olc::BLACK);
		//hulls.push_back(new ConvexHull(createRigidBodyFromPolygons(b2Polygons), 1));
		for (Polygon3D p : *b2Polygons) {
		//	polygons.push_back(p);
		}

		std::vector<Polygon3D>* b3Polygons = createBox(50, 20, 50, x, y + 30, z, olc::DARK_BLUE, olc::BLACK);
		//hulls.push_back(new ConvexHull(createRigidBodyFromPolygons(b3Polygons), 1));
		for (Polygon3D p : *b3Polygons) {
			//polygons.push_back(p);
		}*/

		RigidBody* body = new RigidBody(hulls, 1, 1, 0.3, false);
		PolyModel* model = new PolyModel(&polygons, body->getCenterOfMass());


		body->translate(Vector3D(0, -400, 0));

		body->setAngVelocity(Vector3D(0, 0, 1));
		objects.push_back(new PhysicsObject(model, body));
		pEngine.addRigidBody(body);
	}
	if (GetKey(olc::Key::UP).bHeld) {
		cameraOrientation = cameraOrientation.applyRotor(Rotor(strafeDir, rotateSpeed * fElapsedTime));
	}
	if (GetKey(olc::Key::LEFT).bHeld) {
		cameraOrientation = cameraOrientation.applyRotor(Rotor(Vector3D(0, 1, 0), -rotateSpeed * fElapsedTime));
	}
	if (GetKey(olc::Key::DOWN).bHeld) {
		cameraOrientation = cameraOrientation.applyRotor(Rotor(strafeDir, -rotateSpeed * fElapsedTime));
	}
	if (GetKey(olc::Key::RIGHT).bHeld) {
		cameraOrientation = cameraOrientation.applyRotor(Rotor(Vector3D(0, 1, 0), rotateSpeed * fElapsedTime));
	}
	
}

void SpaceMinerGame::gameUpdate(float fElapsedTime) {
	pEngine.iterateEngine(fElapsedTime);

	readInput(fElapsedTime);
}

void SpaceMinerGame::gameRender() {
	static double fv = 300;

	clearZBuffer();
	Clear(olc::BLACK);

	for (PhysicsObject* o : objects) {
		o->draw(this, cameraPos, cameraOrientation, fv);
	}

	//draw octree
	if (false) {
		std::vector<OctreeNode*> nodes;
		OctreeNode* root = pEngine.getOctreeRoot();
		if (root != nullptr) {
			root->getAllNodes(&nodes);
			for (OctreeNode* node : nodes) {
				double s = node->getSize();
				Vector3D p = node->getPos();
				draw3DLine(p.add(Vector3D(0, 0, 0)), p.add(Vector3D(s, 0, 0)), cameraPos, cameraOrientation, fv, olc::GREEN);
				draw3DLine(p.add(Vector3D(s, 0, 0)), p.add(Vector3D(s, 0, s)), cameraPos, cameraOrientation, fv, olc::GREEN);
				draw3DLine(p.add(Vector3D(s, 0, s)), p.add(Vector3D(0, 0, s)), cameraPos, cameraOrientation, fv, olc::GREEN);
				draw3DLine(p.add(Vector3D(0, 0, s)), p.add(Vector3D(0, 0, 0)), cameraPos, cameraOrientation, fv, olc::GREEN);

				draw3DLine(p.add(Vector3D(0, s, 0)), p.add(Vector3D(s, s, 0)), cameraPos, cameraOrientation, fv, olc::GREEN);
				draw3DLine(p.add(Vector3D(s, s, 0)), p.add(Vector3D(s, s, s)), cameraPos, cameraOrientation, fv, olc::GREEN);
				draw3DLine(p.add(Vector3D(s, s, s)), p.add(Vector3D(0, s, s)), cameraPos, cameraOrientation, fv, olc::GREEN);
				draw3DLine(p.add(Vector3D(0, s, s)), p.add(Vector3D(0, s, 0)), cameraPos, cameraOrientation, fv, olc::GREEN);

				draw3DLine(p.add(Vector3D(0, s, 0)), p.add(Vector3D(0, 0, 0)), cameraPos, cameraOrientation, fv, olc::GREEN);
				draw3DLine(p.add(Vector3D(s, s, 0)), p.add(Vector3D(s, 0, 0)), cameraPos, cameraOrientation, fv, olc::GREEN);
				draw3DLine(p.add(Vector3D(s, s, s)), p.add(Vector3D(s, 0, s)), cameraPos, cameraOrientation, fv, olc::GREEN);
				draw3DLine(p.add(Vector3D(0, s, s)), p.add(Vector3D(0, 0, s)), cameraPos, cameraOrientation, fv, olc::GREEN);
			}
		}
	}
	clearZBuffer();
}

bool SpaceMinerGame::OnUserUpdate(float fElapsedTime) {
	static double dropTime = 100000;
	static int i = 0;
	dropTime += fElapsedTime;

	auto t1 = std::chrono::system_clock::now();
	gameUpdate(fElapsedTime);
	auto t2 = std::chrono::system_clock::now();
	gameRender();
	auto t3 = std::chrono::system_clock::now();
	
	std::chrono::duration<float> update = t2 - t1;
	std::chrono::duration<float> render = t3 - t2;
	//printf("update: %f, render: %f\n", update.count(), render.count());

	return true;
}

int main()
{
	SpaceMinerGame demo;
	if (demo.Construct(960, 540, 2, 2))
		demo.Start();

	return 0;
}

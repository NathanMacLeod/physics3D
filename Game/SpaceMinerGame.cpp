#define OLC_PGE_APPLICATION
#include "SpaceMinerGame.h"
#include "Player.h"
#include "Windows.h"

//ScreenHeight()
//ScreenWidth()
//Draw(x, y, olc::Pixel(r, g, b));

PhysicsObject* createBox(double x, double y, double z, double xPos, double yPos, double zPos, olc::Pixel color, olc::Pixel lineColor, bool fixed) {
	std::vector<Polygon3D> polygons;
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

	polygons.push_back(Polygon3D(p1, p2, p3, lineColor, color));
	polygons.push_back(Polygon3D(p2, p4, p3, lineColor, color));

	polygons.push_back(Polygon3D(p5, p7, p6, lineColor, color));
	polygons.push_back(Polygon3D(p6, p7, p8, lineColor, color));

	polygons.push_back(Polygon3D(p3, p4, p7, lineColor, color));
	polygons.push_back(Polygon3D(p7, p4, p8, lineColor, color));

	polygons.push_back(Polygon3D(p1, p5, p2, lineColor, color));
	polygons.push_back(Polygon3D(p5, p6, p2, lineColor, color));

	polygons.push_back(Polygon3D(p2, p6, p4, lineColor, color));
	polygons.push_back(Polygon3D(p6, p8, p4, lineColor, color));

	polygons.push_back(Polygon3D(p1, p3, p5, lineColor, color));
	polygons.push_back(Polygon3D(p5, p3, p7, lineColor, color));

	std::vector<RigidSurface*> surfaces;

	std::vector<Vector3D> s1Points;
	s1Points.push_back(p1);
	s1Points.push_back(p3);
	s1Points.push_back(p7);
	s1Points.push_back(p5);
	surfaces.push_back(new RigidSurface(&s1Points, Vector3D(0, -1, 0)));

	std::vector<Vector3D> s2Points;
	s2Points.push_back(p2);
	s2Points.push_back(p4);
	s2Points.push_back(p8);
	s2Points.push_back(p6);
	surfaces.push_back(new RigidSurface(&s2Points, Vector3D(0, 1, 0)));

	std::vector<Vector3D> s3Points;
	s3Points.push_back(p1);
	s3Points.push_back(p2);
	s3Points.push_back(p6);
	s3Points.push_back(p5);
	surfaces.push_back(new RigidSurface(&s3Points, Vector3D(-1, 0, 0)));

	std::vector<Vector3D> s4Points;
	s4Points.push_back(p3);
	s4Points.push_back(p4);
	s4Points.push_back(p8);
	s4Points.push_back(p7);
	surfaces.push_back(new RigidSurface(&s4Points, Vector3D(1, 0, 0)));

	std::vector<Vector3D> s5Points;
	s5Points.push_back(p1);
	s5Points.push_back(p2);
	s5Points.push_back(p4);
	s5Points.push_back(p3);
	surfaces.push_back(new RigidSurface(&s5Points, Vector3D(0, 0, -1)));

	std::vector<Vector3D> s6Points;
	s6Points.push_back(p5);
	s6Points.push_back(p6);
	s6Points.push_back(p8);
	s6Points.push_back(p7);
	surfaces.push_back(new RigidSurface(&s6Points, Vector3D(0, 0, 1)));

	std::vector<ConvexHull*> hulls;
	hulls.push_back(new ConvexHull(&surfaces, 1));
	RigidBody* body = new RigidBody(hulls, 1, 1, 0.3, fixed);
	PolyModel* model = new PolyModel(&polygons, body->getCenterOfMass());
	return new PhysicsObject(model, body);
}

bool SpaceMinerGame::OnUserCreate() {

	initZBuffer();
	
	int worldSize = 3000;

	cameraPos = Vector3D(0, -6.5, -250);

	{
		PhysicsObject* floor = createBox(2500, 10, 2500, 0, -1, 0, olc::DARK_RED, olc::BLACK, true);

		objects.push_back(floor);
		pEngine.addRigidBody(floor->getRigidBody());
	}

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

		PhysicsObject* object;
		//object = createBox(75, 75, 375, x, y, z, olc::GREEN, olc::BLACK, false);

		RigidBody* body;
		PolyModel* model;
		createShipMesh(false, 35, olc::VERY_DARK_MAGENTA, olc::BLACK, olc::VERY_DARK_MAGENTA, olc::BLACK, olc::DARK_GREEN, olc::BLACK,
			&body, &model);
		object = new PhysicsObject(model, body);

		object->getRigidBody()->translate(Vector3D(0, -700, 0));

		object->getRigidBody()->setAngVelocity(Vector3D(0, 1, 0));
		objects.push_back(object);
		pEngine.addRigidBody(object->getRigidBody());
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

void SpaceMinerGame::createShipMesh(bool playerShip, double size, olc::Pixel color, olc::Pixel lineColor, olc::Pixel highlight,
	olc::Pixel highlightLine, olc::Pixel cockpit, olc::Pixel cockpitLine, RigidBody** bodyOut, PolyModel** meshOut) {

	std::vector<Vector3D> shipPoints;

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
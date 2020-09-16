#define OLC_PGE_APPLICATION
#include "PixelEngine3D.h"


//ScreenHeight()
//ScreenWidth()
//Draw(x, y, olc::Pixel(r, g, b));

std::vector<Polygon3D>* createBox(double x, double y, double z, double xPos, double yPos, double zPos) {
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

	polygons->push_back(Polygon3D(p1, p2, p3, olc::WHITE, olc::BLACK));
	polygons->push_back(Polygon3D(p2, p4, p3, olc::WHITE, olc::BLACK));

	polygons->push_back(Polygon3D(p5, p7, p6, olc::WHITE, olc::BLACK));
	polygons->push_back(Polygon3D(p6, p7, p8, olc::WHITE, olc::BLACK));

	polygons->push_back(Polygon3D(p3, p4, p7, olc::WHITE, olc::BLACK));
	polygons->push_back(Polygon3D(p7, p4, p8, olc::WHITE, olc::BLACK));

	polygons->push_back(Polygon3D(p1, p5, p2, olc::WHITE, olc::BLACK));
	polygons->push_back(Polygon3D(p5, p6, p2, olc::WHITE, olc::BLACK));

	polygons->push_back(Polygon3D(p2, p6, p4, olc::WHITE, olc::BLACK));
	polygons->push_back(Polygon3D(p6, p8, p4, olc::WHITE, olc::BLACK));

	polygons->push_back(Polygon3D(p1, p3, p5, olc::WHITE, olc::BLACK));
	polygons->push_back(Polygon3D(p5, p3, p7, olc::WHITE, olc::BLACK));

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

class Example : public PixelEngine3D {

	PhysicsEngine pEngine = PhysicsEngine(0.02);
	std::vector<PhysicsObject> models;
	Vector3D cameraPos;
	Rotor cameraOrientation;

public:

	Example() : PixelEngine3D() {
		sAppName = "physicz";
	}

	bool OnUserCreate() override
	{

		initZBuffer();

		std::vector<Polygon3D>* b2polygons = createBox(500, 5, 500, 0, 0, 0);
		std::vector<ConvexHull*>* hulls = new std::vector<ConvexHull*>();
		hulls->push_back(new ConvexHull(createRigidBodyFromPolygons(b2polygons), 1));
		RigidBody* b2 = new RigidBody(*hulls, 1, 1, 0.3, true);

		//models.push_back(PhysicsObject(&PolyModel(b2polygons, b2->getCenterOfMass()), b2));

		//pEngine.addRigidBody(b2);
		pEngine.setGravity(Vector3D(0, 0, 0));
		pEngine.setOctree(true, Vector3D(0, 0, 0), 100000, 300);

		return true;
	}

	bool OnUserUpdate(float fElapsedTime) override
	{
		static double dropTime = 100000;
		dropTime += fElapsedTime;
		if (dropTime > 0.5) {
			dropTime = 0;
			double x = 5;// 52;
			double y = -70;
			double z = 0;
			std::vector<Polygon3D>* b1polygons = createBox(4, 28, 4, x, y, z);
			std::vector<Polygon3D>* b2polygons = createBox(15, 5, 15, x, y - 9, z);
			std::vector<Polygon3D>* b3polygons = createBox(15, 5, 15, x, y + 9, z);
			std::vector<ConvexHull*>* hulls = new std::vector<ConvexHull*>();
			hulls->push_back(new ConvexHull(createRigidBodyFromPolygons(b1polygons), 1));
			hulls->push_back(new ConvexHull(createRigidBodyFromPolygons(b2polygons), 1));
			hulls->push_back(new ConvexHull(createRigidBodyFromPolygons(b3polygons), 1));
			RigidBody* b1 = new RigidBody(*hulls, 1, 1, 0.3, false);
			int v = 20;
			b1->setVelocity(Vector3D(2 * (rand() % v) - v, 2 * (rand() % v) - v, 2 * (rand() % v) - v));

			std::vector<Polygon3D> polygons;

			for (Polygon3D p : *b1polygons) {
				polygons.push_back(p);
			}
			for (Polygon3D p : *b2polygons) {
				polygons.push_back(p);
			}
			for (Polygon3D p : *b3polygons) {
				polygons.push_back(p);
			}

			models.push_back(PhysicsObject(&PolyModel(&polygons, b1->getCenterOfMass()), b1));
			pEngine.addRigidBody(b1);
			delete b1polygons;
			delete b2polygons;
			delete b3polygons;
		}

		auto t1 = std::chrono::system_clock::now();
		
		static double fv = 400;

		Vector3D dir = cameraOrientation.rotate(Vector3D(0, 0, 1));
		Vector3D mov = Vector3D(dir.x, 0, dir.z).getUnitVector();
		float dTheta = fElapsedTime;
		if (GetKey(olc::Key::W).bHeld) {
			cameraPos.z += dTheta * 50 * mov.z;
			cameraPos.x += dTheta * 50 * mov.x;
		}
		if (GetKey(olc::Key::S).bHeld) {
			cameraPos.z -= dTheta * 50 * mov.z;
			cameraPos.x -= dTheta * 50 * mov.x;
		}
		if (GetKey(olc::Key::D).bHeld) {
			cameraPos.x += dTheta * 50 * mov.z;
			cameraPos.z -= dTheta * 50 * mov.x;
		}
		if (GetKey(olc::Key::A).bHeld) {
			cameraPos.x -= dTheta * 50 * mov.z;
			cameraPos.z += dTheta * 50 * mov.x;
		}
		if (GetKey(olc::Key::LEFT).bHeld) {
			cameraOrientation = cameraOrientation.applyRotor(Rotor(Vector3D(0, 1, 0), -dTheta / 3));
		}
		if (GetKey(olc::Key::RIGHT).bHeld) {
			cameraOrientation = cameraOrientation.applyRotor(Rotor(Vector3D(0, 1, 0), dTheta / 3));
		}
		if (GetKey(olc::Key::UP).bHeld) {
			Vector3D axis = cameraOrientation.rotate(Vector3D(1, 0, 0));
			cameraOrientation = cameraOrientation.applyRotor(Rotor(axis, dTheta / 3));
		}
		if (GetKey(olc::Key::DOWN).bHeld) {
			Vector3D axis = cameraOrientation.rotate(Vector3D(1, 0, 0));
			cameraOrientation = cameraOrientation.applyRotor(Rotor(axis, -dTheta / 3));
		}
		if (GetKey(olc::Key::SPACE).bHeld) {
			cameraPos.y -= dTheta * 50;
		}
		if (GetKey(olc::Key::CTRL).bHeld) {
			cameraPos.y += dTheta * 50;
		}
		if (GetKey(olc::Key::I).bHeld) {
			fv += abs(fv) * dTheta;
		}
		if (GetKey(olc::Key::K).bHeld) {
			fv -= abs(fv) * dTheta;
		}

		pEngine.iterateEngine(fElapsedTime);
		auto t2 = std::chrono::system_clock::now();

		Clear(olc::VERY_DARK_GREY);
		auto t3 = std::chrono::system_clock::now();
		clearZBuffer();
		auto t4 = std::chrono::system_clock::now();
		static double t = 0;
		t += fElapsedTime;

		for (PhysicsObject model : models) {
			PolyModel* polygons = model.getModelToDraw();
			if (!polygons->outOfView(cameraPos, cameraOrientation, fv, ScreenWidth(), ScreenHeight())) {
				drawPolygons(polygons->getPolygons(), cameraPos, cameraOrientation, fv, false);
			}
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
					draw3DLine(p.add(Vector3D(0, 0, 0)), p.add(Vector3D(s, 0, 0)), cameraPos, cameraOrientation, fv);
					draw3DLine(p.add(Vector3D(s, 0, 0)), p.add(Vector3D(s, 0, s)), cameraPos, cameraOrientation, fv);
					draw3DLine(p.add(Vector3D(s, 0, s)), p.add(Vector3D(0, 0, s)), cameraPos, cameraOrientation, fv);
					draw3DLine(p.add(Vector3D(0, 0, s)), p.add(Vector3D(0, 0, 0)), cameraPos, cameraOrientation, fv);

					draw3DLine(p.add(Vector3D(0, s, 0)), p.add(Vector3D(s, s, 0)), cameraPos, cameraOrientation, fv);
					draw3DLine(p.add(Vector3D(s, s, 0)), p.add(Vector3D(s, s, s)), cameraPos, cameraOrientation, fv);
					draw3DLine(p.add(Vector3D(s, s, s)), p.add(Vector3D(0, s, s)), cameraPos, cameraOrientation, fv);
					draw3DLine(p.add(Vector3D(0, s, s)), p.add(Vector3D(0, s, 0)), cameraPos, cameraOrientation, fv);

					draw3DLine(p.add(Vector3D(0, s, 0)), p.add(Vector3D(0, 0, 0)), cameraPos, cameraOrientation, fv);
					draw3DLine(p.add(Vector3D(s, s, 0)), p.add(Vector3D(s, 0, 0)), cameraPos, cameraOrientation, fv);
					draw3DLine(p.add(Vector3D(s, s, s)), p.add(Vector3D(s, 0, s)), cameraPos, cameraOrientation, fv);
					draw3DLine(p.add(Vector3D(0, s, s)), p.add(Vector3D(0, 0, s)), cameraPos, cameraOrientation, fv);
				}
			}
		}

		auto t5 = std::chrono::system_clock::now();

		std::chrono::duration<float> physTime = t2 - t1;
		std::chrono::duration<float> renderTime = t5 - t2;
		std::chrono::duration<float> rectTime = t3 - t2;
		std::chrono::duration<float> clearZTime = t4 - t3;
		std::chrono::duration<float> drawPolygons = t5 - t4;

		//printf("renderTime: %f, physicsTime: %f, drawRect: %f, clearZBuffer: %f, drawpolygons %f\n", renderTime.count(),
			//physTime.count(), rectTime.count(), clearZTime.count(), drawPolygons.count());
		return true;
	}
};


int main()
{
	Example demo;
	if (demo.Construct(960, 540, 2, 2))
		demo.Start();

	return 0;
}

#include "ConsoleEngine.h"
#include "PhysicsEngine.h"
#include <stdio.h>
#include <chrono>
#include <vector>

std::vector<Polygon3D*>* createBox(double x, double y, double z, double xPos, double yPos, double zPos) {
	std::vector<Polygon3D*>* polygons = new std::vector<Polygon3D*>();
	polygons->push_back(new Polygon3D(new Point3D(-x / 2.0, -y / 2.0, -z / 2.0), new Point3D(-x / 2.0, y / 2.0, -z / 2.0), new Point3D(x / 2.0, -y / 2.0, -z / 2.0), black, red));
	polygons->push_back(new Polygon3D(new Point3D(-x / 2.0, y / 2.0, -z / 2.0), new Point3D(x / 2.0, y / 2.0, -z / 2.0), new Point3D(x / 2.0, -y / 2.0, -z / 2.0), black, red));

	polygons->push_back(new Polygon3D(new Point3D(-x / 2.0, -y / 2.0, z / 2.0), new Point3D(x / 2.0, -y / 2.0, z / 2.0), new Point3D(-x / 2.0, y / 2.0, z / 2.0), black, orange));
	polygons->push_back(new Polygon3D(new Point3D(-x / 2.0, y / 2.0, z / 2.0), new Point3D(x / 2.0, -y / 2.0, z / 2.0), new Point3D(x / 2.0, y / 2.0, z / 2.0), black, orange));

	polygons->push_back(new Polygon3D(new Point3D(x / 2.0, -y / 2.0, -z / 2.0), new Point3D(x / 2.0, y / 2.0, -z / 2.0), new Point3D(x / 2.0, -y / 2.0, z / 2.0), black, green));
	polygons->push_back(new Polygon3D(new Point3D(x / 2.0, -y / 2.0, z / 2.0), new Point3D(x / 2.0, y / 2.0, -z / 2.0), new Point3D(x / 2.0, y / 2.0, z / 2.0), black, green));

	polygons->push_back(new Polygon3D(new Point3D(-x / 2.0, -y / 2.0, -z / 2.0), new Point3D(-x / 2.0, -y / 2.0, z / 2.0), new Point3D(-x / 2.0, y / 2.0, -z / 2.0), black, blue));
	polygons->push_back(new Polygon3D(new Point3D(-x / 2.0, -y / 2.0, z / 2.0), new Point3D(-x / 2.0, y / 2.0, z / 2.0), new Point3D(-x / 2.0, y / 2.0, -z / 2.0), black, blue));

	polygons->push_back(new Polygon3D(new Point3D(-x / 2.0, y / 2.0, -z / 2.0), new Point3D(-x / 2.0, y / 2.0, z / 2.0), new Point3D(x / 2.0, y / 2.0, -z / 2.0), black, Color::light_gray));
	polygons->push_back(new Polygon3D(new Point3D(-x / 2.0, y / 2.0, z / 2.0), new Point3D(x / 2.0, y / 2.0, z / 2.0), new Point3D(x / 2.0, y / 2.0, -z / 2.0), black, light_gray));

	polygons->push_back(new Polygon3D(new Point3D(-x / 2.0, -y / 2.0, -z / 2.0), new Point3D(x / 2.0, -y / 2.0, -z / 2.0), new Point3D(-x / 2.0, -y / 2.0, z / 2.0), black, gray));
	polygons->push_back(new Polygon3D(new Point3D(-x / 2.0, -y / 2.0, z / 2.0), new Point3D(x / 2.0, -y / 2.0, -z / 2.0), new Point3D(x / 2.0, -y / 2.0, z / 2.0), black, gray));

	Vector3D translation(xPos, yPos, zPos);
	std::vector<Point3D*> points;
	for (Polygon3D* polygon : *polygons) {
		points.push_back(polygon->p1);
		points.push_back(polygon->p2);
		points.push_back(polygon->p3);
	}
	transformation3D::translatePoints(&points, translation);
	return polygons;
}

std::vector<RigidSurface*>* createRigidBodyFromPolygons(std::vector<Polygon3D*>& polygons) {
	std::vector<RigidSurface*>* surfaces = new std::vector<RigidSurface*>();
	for (Polygon3D* polygon : polygons) {
		Vector3D v1(*(polygon->p1), *(polygon->p2));
		Vector3D v2(*(polygon->p1), *(polygon->p3));
		Vector3D normalVector;
		v1.crossProduct(v2, &normalVector);
		normalVector.getUnitVector(&normalVector);
		std::vector<Point3D*>* points = new std::vector<Point3D*>();
		points->push_back(polygon->p1);
		points->push_back(polygon->p2);
		points->push_back(polygon->p3);
		RigidSurface* surface = new RigidSurface(*points, normalVector);
		surfaces->push_back(surface);
	}
	return surfaces;
}

int main() {
	ConsoleEngine engine(180, 180, 2);
	PhysicsEngine pEngine;
	engine.fillRect(0, 0, 180, 180, engine.PICZEL, blue);
	//engine.fillRect(10, 20, 20, 20, engine.PICZEL, green);
	//engine.fillRect(0, 0, 30, 60, engine.PICZEL, white);
	std::chrono::system_clock::time_point t1 = std::chrono::system_clock::now();
	std::chrono::system_clock::time_point t2;
	//engine.drawLine(20, 20, 90, 80, engine.PICZEL, black);
	float queuedTime = 0;
	std::vector<Polygon3D*>* allPolygons = new std::vector<Polygon3D*>();

	std::vector<Polygon3D*>* b1polygons = createBox(10, 10, 10, 0, -23.6, 100);
	std::vector<Polygon3D*>* b2polygons = createBox(100, 5, 100, 0, 30, 100);
	RigidBody* b1 = new RigidBody(*createRigidBodyFromPolygons(*b1polygons), 1, 0.3, 0.3, false);
	RigidBody* b2 = new RigidBody(*createRigidBodyFromPolygons(*b2polygons), 1, 0.3, 0.3, true);

	pEngine.addRigidBody(b1);
	pEngine.addRigidBody(b2);

	for (Polygon3D* polygon : *b2polygons) {
		allPolygons->push_back(polygon);
	}
	for (Polygon3D* polygon : *b1polygons) {
		allPolygons->push_back(polygon);
	}
	

	Point3D cameraPos(0, 0, 0);

	float r = 15;

	double yAng = 3.14159 / 2.0;
	double xAng = 0;

	bool w;
	bool a;
	bool s;
	bool d;
	bool up;
	bool down;
	bool left;
	bool right;
	bool space;
	bool ctrl;
	bool x;
	bool xHeld = false;
	bool v;
	bool vHeld = false;

	while (true) {
		t2 = std::chrono::system_clock::now();
		std::chrono::duration<float> timePassed = t2 - t1;
		t1 = t2;
		queuedTime += timePassed.count();
		while (queuedTime > pEngine.getTimestep()) {
			//std::cout << "X:" << b1->getCenterOfMass()->x << " y:" << b1->getCenterOfMass()->y << " z:" << b1->getCenterOfMass()->z << "\n";
			pEngine.iterateEngineTimestep();
			queuedTime -= pEngine.getTimestep();
		}

		w = (GetAsyncKeyState(0x57) & 0x8000) != 0;
		a = (GetAsyncKeyState(0x41) & 0x8000) != 0;
		s = (GetAsyncKeyState(0x53) & 0x8000) != 0;
		d = (GetAsyncKeyState(0x44) & 0x8000) != 0;
		x = (GetAsyncKeyState(0x58) & 0x8000) != 0;
		v = (GetAsyncKeyState(0x56) & 0x8000) != 0;
		up = (GetAsyncKeyState(VK_UP) & 0x8000) != 0;
		down = (GetAsyncKeyState(VK_DOWN) & 0x8000) != 0;
		left = (GetAsyncKeyState(VK_LEFT) & 0x8000) != 0;
		right = (GetAsyncKeyState(VK_RIGHT) & 0x8000) != 0;
		space = (GetAsyncKeyState(VK_SPACE) & 0x8000) != 0;
		ctrl = (GetAsyncKeyState(VK_CONTROL) & 0x8000) != 0;

		Vector3D axis(0, 1, -1);
		engine.fillRect(0, 0, 180, 180, engine.PICZEL, white);
		float dTheta = timePassed.count();
		if(w) {
			cameraPos.z += dTheta * 50 * sin(yAng);
			cameraPos.x -= dTheta * 50 * cos(yAng);
		}
		if (s) {
			cameraPos.z -= dTheta * 50 * sin(yAng);
			cameraPos.x += dTheta * 50 * cos(yAng);
		}
		if (d) {
			cameraPos.x += dTheta * 50 * sin(yAng);
			cameraPos.z += dTheta * 50 * cos(yAng);
		}
		if (a) {
			cameraPos.x -= dTheta * 50 * sin(yAng);
			cameraPos.z -= dTheta * 50 * cos(yAng);
		}
		if (left) {
			yAng -= dTheta/3;
		}
		if (right) {
			yAng += dTheta/3;
		}
		if (up) {
			xAng += dTheta/3;
		}
		if (down) {
			xAng -= dTheta/3;
		}
		if (space) {
			cameraPos.y -= dTheta * 50;
		}
		if (ctrl) {
			cameraPos.y += dTheta * 50;
		}
		/*if (x) {
			//cameraPos.y += dTheta * 10;
			if (!xHeld) {
				transformation3D::rotatePointsAroundArbitraryAxis(&points, axis, 0, 0, cubeCenter.z + 20, -3.1415926535/2, 0, 0);
				xHeld = true;
			}
		}
		else {
			xHeld = false;
		}
		if (v) {
			//cameraPos.y += dTheta * 10;
			if (!vHeld) {
				transformation3D::rotatePointsAroundArbitraryAxis(&points, axis, 0, 0, cubeCenter.z + 20, 3.1415926535 / 2, 0, 0);
				vHeld = true;
			}
		}
		else {
			vHeld = false;
		}*/
		//transformation3D::rotatePointsAroundYParralelAxis(&points, dTheta/3.0, 0, cubeCenter.z);
		//transformation3D::rotatePointsAroundXParralelAxis(&points, dTheta / 1.5, 0, cubeCenter.z);
			
		//axis.getUnitVector(&axis);
		//int n = 10000;
		//for (int i = 0; i < n; i++) {
			//transformation3D::rotatePointsAroundArbitraryAxis(&points, axis, 0, 0, cubeCenter.z + 20, 3.1415926535, 0, 0);
		//}
		//cameraPos.x = r * cos(queuedTime);
		//cameraPos.y = -5;
		//cameraPos.y = r * sin(queuedTime);
		//transformation3D::rotatePointsAroundArbitraryAxis(&points, axis, 0, 0, cubeCenter.z, dTheta, 0, 0);
		engine.drawPolygons(*allPolygons, cameraPos, yAng, xAng, 200, false);
		//engine.draw3DLine(p1, p2, cameraPos, yAng, xAng, 200);

		//engine.drawLine(0, 0, 179, 0, engine.PICZEL, white);
		//engine.drawLine(179, 0, 179, 179, engine.PICZEL, white);
		//engine.drawLine(179, 179, 0, 179, engine.PICZEL, white);
		//engine.drawLine(0, 179, 0, 0, engine.PICZEL, white);

		engine.outputScreen();

	}
	return 1;
}
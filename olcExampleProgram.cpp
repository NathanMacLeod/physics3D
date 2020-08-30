#define OLC_PGE_APPLICATION
#include "olcPixelGameEngine.h"
#include "Vector3D.h"
#include <thread>
#include <stdio.h>
#include "Point.h"
#include <vector>
#include "Color.h"
#include <list>
#include <chrono>
#include "Polygon3D.h"
#include "transformation3D.h"
#include "PhysicsEngine.h"


//ScreenHeight()
//ScreenWidth()
//Draw(x, y, olc::Pixel(r, g, b));

std::vector<Polygon3D*>* createBox(double x, double y, double z, double xPos, double yPos, double zPos) {
	std::vector<Polygon3D*>* polygons = new std::vector<Polygon3D*>();
	Point3D* p1 = new Point3D(-x / 2.0, -y / 2.0, -z / 2.0);
	Point3D* p2 = new Point3D(-x / 2.0, y / 2.0, -z / 2.0);
	Point3D* p3 = new Point3D(x / 2.0, -y / 2.0, -z / 2.0);
	Point3D* p4 = new Point3D(x / 2.0, y / 2.0, -z / 2.0);
	Point3D* p5 = new Point3D(-x / 2.0, -y / 2.0, z / 2.0);
	Point3D* p6 = new Point3D(-x / 2.0, y / 2.0, z / 2.0);
	Point3D* p7 = new Point3D(x / 2.0, -y / 2.0, z / 2.0);
	Point3D* p8 = new Point3D(x / 2.0, y / 2.0, z / 2.0);

	polygons->push_back(new Polygon3D(p1, p2, p3, black, red));
	polygons->push_back(new Polygon3D(p2, p4, p3, black, red));

	polygons->push_back(new Polygon3D(p5, p7, p6, black, orange));
	polygons->push_back(new Polygon3D(p6, p7, p8, black, orange));

	polygons->push_back(new Polygon3D(p3, p4, p7, black, green));
	polygons->push_back(new Polygon3D(p7, p4, p8, black, green));

	polygons->push_back(new Polygon3D(p1, p5, p2, black, blue));
	polygons->push_back(new Polygon3D(p5, p6, p2, black, blue));

	polygons->push_back(new Polygon3D(p2, p6, p4, black, light_gray));
	polygons->push_back(new Polygon3D(p6, p8, p4, black, light_gray));

	polygons->push_back(new Polygon3D(p1, p3, p5, black, gray));
	polygons->push_back(new Polygon3D(p5, p3, p7, black, gray));

	Vector3D translation(xPos, yPos, zPos);
	std::vector<Point3D*> points;
	points.push_back(p1);
	points.push_back(p2);
	points.push_back(p3);
	points.push_back(p4);
	points.push_back(p5);
	points.push_back(p6);
	points.push_back(p7);
	points.push_back(p8);
	transformation3D::translatePoints(&points, translation);
	return polygons;
}

std::vector<RigidSurface*>* createRigidBodyFromPolygons(std::vector<Polygon3D*>& polygons) {
	std::vector<RigidSurface*>* surfaces = new std::vector<RigidSurface*>();
	for (Polygon3D* polygon : polygons) {
		Vector3D v1(*(polygon->p1), *(polygon->p2));
		Vector3D v2(*(polygon->p1), *(polygon->p3));
		Vector3D normalVector = (v1.crossProduct(v2)).getUnitVector();
		std::vector<Point3D*>* points = new std::vector<Point3D*>();
		points->push_back(polygon->p1);
		points->push_back(polygon->p2);
		points->push_back(polygon->p3);
		RigidSurface* surface = new RigidSurface(*points, normalVector);
		surfaces->push_back(surface);
	}
	return surfaces;
}

class Example : public olc::PixelGameEngine {

	double queuedTime;
	Point3D cameraPos;
	PhysicsEngine pEngine;
	std::vector<Polygon3D*>* allPolygons;
	double yAng = 3.14159 / 2.0;
	double xAng = 0;

	double* zBuffer;
	void clearZBuffer() {
		for (int i = 0; i < ScreenWidth() * ScreenHeight(); i++) {
			zBuffer[i] = -1;
		}
	}

	int getPixelIndex(int x, int y) {
		return x + y * ScreenWidth();
	}

public:

	Example() : PixelGameEngine() {
		sAppName = "physicz";
	}
	~Example() {
		delete zBuffer;
	}

	void drawPiczel(int x, int y, Color color) {
		Draw(x, y, olc::Pixel(getR(color), getG(color), getB(color)));
	}

	void drawLine(double x1, double y1, double x2, double y2, Color color) {
		int changeX = x2 - x1;
		int changeY = y2 - y1;

		int absChangeX = abs(changeX);
		int absChangeY = abs(changeY);

		if (x1 >= 0 && x1 < ScreenWidth() && y1 >= 0 && y1 < ScreenHeight())
			drawPiczel(x1, y1, color);
		if (x2 >= 0 && x2 < ScreenWidth() && y2 >= 0 && y2 < ScreenHeight())
			drawPiczel(x2, y2, color);

		//Iterate along the longer dimension to avoid gaps in line
		if (absChangeX > absChangeY) {
			float m = (float)changeY / changeX;
			for (int i = 0; i < absChangeX; i++) {
				int dx = (changeX < 0) ? -i : i;
				float dy = dx * m;
				int x = x1 + dx;
				int y = y1 + dy;
				if (x >= 0 && x < ScreenWidth() && y >= 0 && y < ScreenHeight())
					drawPiczel(x, y, color);
			}
		}
		else {
			float k = (float)changeX / changeY;
			for (int i = 0; i < absChangeY; i++) {
				int dy = (changeY < 0) ? -i : i;
				float dx = dy * k;
				int x = x1 + dx;
				int y = y1 + dy;
				if (x >= 0 && x < ScreenWidth() && y >= 0 && y < ScreenHeight())
					drawPiczel(x, y, color);
			}
		}
	}

	void drawTriangle(double x1, double y1, double x2, double y2, double x3, double y3, Color color) {
		drawLine(x1, y1, x2, y2, color);
		drawLine(x2, y2, x3, y3, color);
		drawLine(x3, y3, x1, y1, color);
	}

	void drawTriangle(const Point3D p1, const Point3D p2, const Point3D p3, Color color) {
		drawTriangle(p1.x, p1.y, p2.x, p2.y, p3.x, p3.y, color);
	}

	void fillTriangle(double x1, double y1, double x2, double y2, double x3, double y3, Color color) {
		Point3D p1(x1, y1);
		Point3D p2(x2, y2);
		Point3D p3(x3, y3);
		fillTriangle(p1, p2, p3, color);
	}

	void fillTriangle(const Point3D p1, const Point3D p2, const Point3D p3, Color color, bool useZBuffer = false) {
		Vector3D normalVector;
		double invNz = 0;
		if (useZBuffer) {
			Vector3D v1(p1, p2);
			Vector3D v2(p1, p3);
			normalVector = v1.crossProduct(v2);
			invNz = 1.0 / normalVector.z;
		}
		const Point3D* upperPoint = &p1;
		const Point3D* midPoint = &p2;
		const Point3D* lowerPoint = &p3;

		const Point3D* temp = nullptr;
		if (upperPoint->y > midPoint->y) {
			temp = upperPoint;
			upperPoint = midPoint;
			midPoint = temp;
		}
		if (midPoint->y > lowerPoint->y) {
			temp = midPoint;
			midPoint = lowerPoint;
			lowerPoint = temp;
		}
		if (upperPoint->y > midPoint->y) {
			temp = upperPoint;
			upperPoint = midPoint;
			midPoint = temp;
		}

		float upperMidInverseSlope = (midPoint->x - upperPoint->x) / (midPoint->y - upperPoint->y);
		float upperLowerInverseSlope = (lowerPoint->x - upperPoint->x) / (lowerPoint->y - upperPoint->y);

		float leftMostSlope = upperMidInverseSlope;
		float rightMostSlope = upperLowerInverseSlope;

		if (leftMostSlope > rightMostSlope) {
			float temp = leftMostSlope;
			leftMostSlope = rightMostSlope;
			rightMostSlope = temp;
		}

		if ((int)(upperPoint->y) != (int)(midPoint->y)) {

			for (int i = (int)(upperPoint->y); i <= (int)(midPoint->y); i++) {
				//std::cout << "I:" << i << " height:" << heightIndex << "\n";
				if (i < 0 || i >= ScreenHeight()) {
					continue;
				}
				int leftBound = leftMostSlope * (i - upperPoint->y) + upperPoint->x;
				int rightBound = rightMostSlope * (i - upperPoint->y) + upperPoint->x;

				if (leftBound < 0)
					leftBound = 0;
				else if (leftBound >= ScreenWidth()) {
					continue;
				}
				if (rightBound >= ScreenWidth())
					rightBound = ScreenWidth() - 1;
				else if (rightBound < 0) {
					continue;
				}

				for (int j = leftBound; j <= rightBound; j++) {
					if (useZBuffer) {
						double z = p1.z - invNz * (normalVector.x * (j - p1.x) + normalVector.y * (i - p1.y));
						int potato = ScreenHeight();
						if (zBuffer[getPixelIndex(j, i)] > 0 && zBuffer[getPixelIndex(j, i)] <= z)
							continue;
						zBuffer[getPixelIndex(j, i)] = z;
					}
					drawPiczel(j, i, color);
				}

			}

		}

		float lowerMidInverseSlope = (midPoint->x - lowerPoint->x) / (midPoint->y - lowerPoint->y);

		leftMostSlope = lowerMidInverseSlope;
		rightMostSlope = upperLowerInverseSlope;

		if (leftMostSlope < rightMostSlope) {
			float temp = leftMostSlope;
			leftMostSlope = rightMostSlope;
			rightMostSlope = temp;
		}

		for (int i = (int)lowerPoint->y; i > (int)(midPoint->y); i--) {
			if (i < 0 || i >= ScreenHeight()) {
				continue;
			}

			int leftBound = leftMostSlope * (i - lowerPoint->y) + lowerPoint->x;
			int rightBound = rightMostSlope * (i - lowerPoint->y) + lowerPoint->x;


			if (leftBound < 0)
				leftBound = 0;
			else if (leftBound >= ScreenWidth()) {
				continue;
			}
			if (rightBound >= ScreenWidth())
				rightBound = ScreenWidth() - 1;
			else if (rightBound < 0) {
				continue;
			}


			for (int j = leftBound; j <= rightBound; j++) {
				if (useZBuffer) {
					double z = p1.z - invNz * (normalVector.x * (j - p1.x) + normalVector.y * (i - p1.y));
					//std::cout << z << "\n";
					if (zBuffer[getPixelIndex(j, i)] > 0 && zBuffer[getPixelIndex(j, i)] <= z)
						continue;
					zBuffer[getPixelIndex(j, i)] = z;
				}
				drawPiczel(j, i, color);
			}
		}
	}

	void fillRect(float x, float y, float width, float height, Color color) {
		for (int i = 0; i < height; i++) {
			for (int j = 0; j < width; j++) {
				drawPiczel(i, j, color);
			}
		}
	}

	void projectPoint(Point3D* point, float fov) {
		point->x = point->x * fov / point->z + ScreenWidth() / 2;
		point->y = point->y * fov / point->z + ScreenHeight() / 2;
	}

	Color getPhongLighting(Color surfColor, Vector3D& camDir, Vector3D& surfNV, Vector3D& lightDir) {
		Vector3D ambColor(255, 255, 255);
		double ambBrightness = 0.6;
		ambColor = ambColor.multiply(ambBrightness);
		Vector3D dirColor(255, 255, 255);
		double dirBrightness = 1;
		Vector3D specColor = dirColor;
		double dirLight = (surfNV.dotProduct(lightDir) > 0) ? surfNV.dotProduct(lightDir) : 0;
		dirColor = dirColor.multiply(dirLight * dirBrightness);
		
		ambColor.x *= getR(surfColor) / 255.0;
		ambColor.y *= getG(surfColor) / 255.0;
		ambColor.z *= getB(surfColor) / 255.0;

		dirColor.x *= getR(surfColor) / 255.0;
		dirColor.y *= getG(surfColor) / 255.0;
		dirColor.z *= getB(surfColor) / 255.0;

		int shinyC = 32;
		Vector3D refl = surfNV.multiply(-2 * surfNV.dotProduct(lightDir)).add(lightDir);
		double dot = -refl.dotProduct(camDir);
		double c = 1;
		for (int i = 0; i < shinyC; i++)
			c *= dot;

		specColor = specColor.multiply(c);

		Vector3D totalColor = ambColor.add(dirColor).add(specColor);

		if (totalColor.x > 255)
			totalColor.x = 255;
		if (totalColor.y > 255)
			totalColor.y = 255;
		if (totalColor.z > 255)
			totalColor.z = 255;

		return getColor((int) totalColor.x, (int) totalColor.y, (int) totalColor.z);
	}

	void drawPolygons(const std::vector<Polygon3D*> polygons, const Point3D cameraPosition, float orientationYAng, float orientationPitch, float fov, bool drawLines) {
		clearZBuffer();
		//copy reference to copy of polygon points

		//Vector3D cameraVector(cos(orientationYAng) * cos(orientationPitch), sin(orientationPitch), sin(orientationYAng) * cos(orientationPitch));
		std::vector<Point3D*> points;
		std::vector<Polygon3D*> includedPolygons;

		for (Polygon3D* polygon : polygons) {
			Vector3D cameraToPolygon(polygon->p2->x - cameraPosition.x, polygon->p2->y - cameraPosition.y, polygon->p2->z - cameraPosition.z);
			Vector3D p1p2(polygon->p2->x - polygon->p1->x, polygon->p2->y - polygon->p1->y, polygon->p2->z - polygon->p1->z);
			Vector3D p1p3(polygon->p3->x - polygon->p1->x, polygon->p3->y - polygon->p1->y, polygon->p3->z - polygon->p1->z);
			Vector3D normalVector = p1p2.crossProduct(p1p3);
			if (cameraToPolygon.dotProduct(normalVector) < 0) {
				polygon->copyPointsToTemp();
				points.push_back(polygon->tempP1);
				points.push_back(polygon->tempP2);
				points.push_back(polygon->tempP3);
				includedPolygons.push_back(polygon);
			}
		}

		Vector3D cameraPositionToOrigin(-cameraPosition.x, -cameraPosition.y, -cameraPosition.z);
		//translate points to origin
		transformation3D::translatePoints(&points, cameraPositionToOrigin);
		//rotate so that world so that camera stares towards z axis
		transformation3D::rotatePointsAroundYParralelAxis(&points, -orientationYAng + 1.570795, 0, 0);
		//pitch world so to that camera is inline with z axis
		transformation3D::rotatePointsAroundXParralelAxis(&points, -orientationPitch, 0, 0);
		for (Point3D* p : points) {
			projectPoint(p, fov);
			//drawPiczel(p->x, p->y, PICZEL, white);
		}

		//TEMPORARY FOR PHONG LIGHTING TEST
		Vector3D camDir(cos(orientationYAng + 1.570795), sin(orientationPitch), cos(orientationPitch) * sin(orientationYAng + 1.570795));

		for (Polygon3D* polygon : includedPolygons) {

			//TODO IMPLEMENT NOT DUMB
			Vector3D p1p2(polygon->p2->x - polygon->p1->x, polygon->p2->y - polygon->p1->y, polygon->p2->z - polygon->p1->z);
			Vector3D p1p3(polygon->p3->x - polygon->p1->x, polygon->p3->y - polygon->p1->y, polygon->p3->z - polygon->p1->z);
			Vector3D normalVector = p1p2.crossProduct(p1p3).getUnitVector();
			Vector3D lightDir(1, -1, -1);
			lightDir = lightDir.getUnitVector();

			//Color color = getPhongLighting(polygon->color, camDir, normalVector, lightDir);
			//FillTriangle(polygon->tempP1->x, polygon->tempP1->y, polygon->tempP2->x, polygon->tempP2->y, polygon->tempP3->x, polygon->tempP3->y, olc::Pixel(getR(color), getG(color), getB(color)));
			
			fillTriangle(*(polygon->tempP1), *(polygon->tempP2), *(polygon->tempP3), getPhongLighting(polygon->color, camDir, normalVector, lightDir), true);
			if (drawLines)
				drawTriangle(*(polygon->tempP1), *(polygon->tempP2), *(polygon->tempP3), polygon->p1p2Color);
		}
	}

	void draw3DLine(Point3D p1, Point3D p2, const Point3D cameraPosition, float orientationYAng, float orientationPitch, float fov) {
		Vector3D cameraPositionToOrigin(-cameraPosition.x, -cameraPosition.y, -cameraPosition.z);

		transformation3D::translatePoint(&p1, cameraPositionToOrigin);
		transformation3D::rotatePointAroundYParralelAxis(&p1, -orientationYAng + 1.570795, 0, 0);
		transformation3D::rotatePointAroundXParralelAxis(&p1, -orientationPitch, 0, 0);
		projectPoint(&p1, fov);

		transformation3D::translatePoint(&p2, cameraPositionToOrigin);
		transformation3D::rotatePointAroundYParralelAxis(&p2, -orientationYAng + 1.570795, 0, 0);
		transformation3D::rotatePointAroundXParralelAxis(&p2, -orientationPitch, 0, 0);
		projectPoint(&p2, fov);

		drawLine(p1.x, p1.y, p2.x, p2.y, green);
	}

	void draw3DPoint(Point3D p,const Point3D cameraPosition, float orientationYAng, float orientationPitch, float fov) {
		Vector3D cameraPositionToOrigin(-cameraPosition.x, -cameraPosition.y, -cameraPosition.z);

		transformation3D::translatePoint(&p, cameraPositionToOrigin);
		transformation3D::rotatePointAroundYParralelAxis(&p, -orientationYAng + 1.570795, 0, 0);
		transformation3D::rotatePointAroundXParralelAxis(&p, -orientationPitch, 0, 0);
		projectPoint(&p, fov);

		drawPiczel((int)p.x, (int)p.y, red);
	}

	bool OnUserCreate() override
	{
		zBuffer = new double[ScreenWidth() * ScreenHeight()];
		allPolygons = new std::vector<Polygon3D*>();
		cameraPos.z = -150;
		//cameraPos.y = 27.5;
		//cameraPos.x = 52;

		std::vector<Polygon3D*>* b2polygons = createBox(100, 5, 100, 0, 35, 0);
		std::vector<ConvexHull*>* hulls = new std::vector<ConvexHull*>();
		hulls->push_back(new ConvexHull(*createRigidBodyFromPolygons(*b2polygons), 1));
		RigidBody* b2 = new RigidBody(*hulls, 1, 1, 0.3, true);
		pEngine.addRigidBody(b2);

		for (Polygon3D* polygon : *b2polygons) {
			allPolygons->push_back(polygon);
		}
		return true;

		queuedTime = 0;
	}

	bool OnUserUpdate(float fElapsedTime) override
	{
		static double dropTime = 10000000;
		dropTime += fElapsedTime;
		if (dropTime > 5) {
			dropTime = 0;
			double x = 5;// 52;
			double y = -10;
			double z = 0;
			std::vector<Polygon3D*>* b1polygons = createBox(1, 25, 1, x, y, z);

			std::vector<Point3D*> points;
			for (Polygon3D* polygon : *b1polygons) {
				allPolygons->push_back(polygon);
				bool p1 = true;
				bool p2 = true;
				bool p3 = true;
				for (Point3D* p : points) {
					if (polygon->p1 == p)
						p1 = false;
					if (polygon->p2 == p)
						p2 = false;
					if (polygon->p3 == p)
						p3 = false;
				}
				if (p1)
					points.push_back(polygon->p1);
				if (p2)
					points.push_back(polygon->p2);
				if (p3)
					points.push_back(polygon->p3);
			}
			Vector3D axis(0, 1, 0);
			axis = axis.getUnitVector();
			//transformation3D::rotatePointsAroundArbitraryAxis(&points, axis, x, y, z, 3.14159 / 3.0);
			Vector3D axis2(0, 0, 1);
			axis2 = axis2.getUnitVector();

			std::vector<Polygon3D*>* b2polygons = createBox(10, 10, 10, x, y - 15, z);

			std::vector<Point3D*> b2points;
			for (Polygon3D* polygon : *b2polygons) {
				//allPolygons->push_back(polygon);
				bool p1 = true;
				bool p2 = true;
				bool p3 = true;
				for (Point3D* p : points) {
					if (polygon->p1 == p)
						p1 = false;
					if (polygon->p2 == p)
						p2 = false;
					if (polygon->p3 == p)
						p3 = false;
				}
				if (p1)
					b2points.push_back(polygon->p1);
				if (p2)
					b2points.push_back(polygon->p2);
				if (p3)
					b2points.push_back(polygon->p3);
			}
			//Vector3D axis(0, 1, 0);
			//axis = axis.getUnitVector();
			//transformation3D::rotatePointsAroundArbitraryAxis(&points, axis, x, y, z, 3.14159 / 3.0);
			//Vector3D axis2(0, 0, 1);
			//axis2 = axis2.getUnitVector();

			std::vector<Polygon3D*>* b3polygons = createBox(25, 3, 25, x, y - 8, z);

			std::vector<Point3D*> b3points;
			for (Polygon3D* polygon : *b3polygons) {
				allPolygons->push_back(polygon);
				bool p1 = true;
				bool p2 = true;
				bool p3 = true;
				for (Point3D* p : points) {
					if (polygon->p1 == p)
						p1 = false;
					if (polygon->p2 == p)
						p2 = false;
					if (polygon->p3 == p)
						p3 = false;
				}
				if (p1)
					b3points.push_back(polygon->p1);
				if (p2)
					b3points.push_back(polygon->p2);
				if (p3)
					b3points.push_back(polygon->p3);
			}
			//Vector3D axis(0, 1, 0);
			//axis = axis.getUnitVector();
			//transformation3D::rotatePointsAroundArbitraryAxis(&points, axis, x, y, z, 3.14159 / 3.0);
			//Vector3D axis2(0, 0, 1);
			//axis2 = axis2.getUnitVector();
			//transformation3D::rotatePointsAroundArbitraryAxis(&points, axis2, x, y, z, -3.14159 / 3);
			std::vector<ConvexHull*>* hulls = new std::vector<ConvexHull*>();
			hulls->push_back(new ConvexHull(*createRigidBodyFromPolygons(*b1polygons), 1));
			//hulls->push_back(new ConvexHull(*createRigidBodyFromPolygons(*b2polygons), 1));
			hulls->push_back(new ConvexHull(*createRigidBodyFromPolygons(*b3polygons), 1));
			RigidBody* b1 = new RigidBody(*hulls, 1, 1, 0.3, false);
			pEngine.addRigidBody(b1);
			delete b1polygons;
			delete b2polygons;
			delete b3polygons;
		}

		auto t1 = std::chrono::system_clock::now();
		
		float dTheta = fElapsedTime;
		if (GetKey(olc::Key::W).bHeld) {
			cameraPos.z += dTheta * 50 * sin(yAng);
			cameraPos.x -= dTheta * 50 * cos(yAng);
		}
		if (GetKey(olc::Key::S).bHeld) {
			cameraPos.z -= dTheta * 50 * sin(yAng);
			cameraPos.x += dTheta * 50 * cos(yAng);
		}
		if (GetKey(olc::Key::D).bHeld) {
			cameraPos.x += dTheta * 50 * sin(yAng);
			cameraPos.z += dTheta * 50 * cos(yAng);
		}
		if (GetKey(olc::Key::A).bHeld) {
			cameraPos.x -= dTheta * 50 * sin(yAng);
			cameraPos.z -= dTheta * 50 * cos(yAng);
		}
		if (GetKey(olc::Key::LEFT).bHeld) {
			yAng -= dTheta / 3;
		}
		if (GetKey(olc::Key::RIGHT).bHeld) {
			yAng += dTheta / 3;
		}
		if (GetKey(olc::Key::UP).bHeld) {
			xAng += dTheta / 3;
		}
		if (GetKey(olc::Key::DOWN).bHeld) {
			xAng -= dTheta / 3;
		}
		if (GetKey(olc::Key::SPACE).bHeld) {
			cameraPos.y -= dTheta * 50;
		}
		if (GetKey(olc::Key::CTRL).bHeld) {
			cameraPos.y += dTheta * 50;
		}

		queuedTime += fElapsedTime;
		while (queuedTime > pEngine.getTimestep()) {
			//std::cout << "X:" << b1->getCenterOfMass()->x << " y:" << b1->getCenterOfMass()->y << " z:" << b1->getCenterOfMass()->z << "\n";
			pEngine.iterateEngineTimestep();
			queuedTime -= pEngine.getTimestep();
		}
		auto t2 = std::chrono::system_clock::now();

		FillRect(0, 0, ScreenWidth(), ScreenHeight(), olc::Pixel(255, 255, 255));
		clearZBuffer();
		drawPolygons(*allPolygons, cameraPos, yAng, xAng, 200, false);

		//draw octree
		/*std::vector<OctreeNode*> nodes;
		OctreeNode::getAllNodes(&pEngine.root, &nodes);
		for (OctreeNode* node : nodes) {
			double s = node->getSize();
			Point3D p = node->getPos();
			draw3DLine(Point3D(p, 0, 0, 0), Point3D(p, s, 0, 0), cameraPos, yAng, xAng, 200);
			draw3DLine(Point3D(p, s, 0, 0), Point3D(p, s, 0, s), cameraPos, yAng, xAng, 200);
			draw3DLine(Point3D(p, s, 0, s), Point3D(p, 0, 0, s), cameraPos, yAng, xAng, 200);
			draw3DLine(Point3D(p, 0, 0, s), Point3D(p, 0, 0, 0), cameraPos, yAng, xAng, 200);

			draw3DLine(Point3D(p, 0, s, 0), Point3D(p, s, s, 0), cameraPos, yAng, xAng, 200);
			draw3DLine(Point3D(p, s, s, 0), Point3D(p, s, s, s), cameraPos, yAng, xAng, 200);
			draw3DLine(Point3D(p, s, s, s), Point3D(p, 0, s, s), cameraPos, yAng, xAng, 200);
			draw3DLine(Point3D(p, 0, s, s), Point3D(p, 0, s, 0), cameraPos, yAng, xAng, 200);

			draw3DLine(Point3D(p, 0, s, 0), Point3D(p, 0, 0, 0), cameraPos, yAng, xAng, 200);
			draw3DLine(Point3D(p, s, s, 0), Point3D(p, s, 0, 0), cameraPos, yAng, xAng, 200);
			draw3DLine(Point3D(p, s, s, s), Point3D(p, s, 0, s), cameraPos, yAng, xAng, 200);
			draw3DLine(Point3D(p, 0, s, s), Point3D(p, 0, 0, s), cameraPos, yAng, xAng, 200);
		}*/

		for (RigidBody* b : pEngine.rigidBodies) {
			draw3DPoint(b->getCenterOfMass(), cameraPos, yAng, xAng, 200);
		}

		auto t3 = std::chrono::system_clock::now();

		std::chrono::duration<float> physTime = t2 - t1;
		std::chrono::duration<float> renderTime = t3 - t2;

		//printf("renderTime: %f, physicsTime: %f\n", renderTime.count(), physTime.count());
		return true;
	}
};


int main()
{
	Example demo;
	if (demo.Construct(550, 550, 1, 1))
		demo.Start();

	return 0;
}

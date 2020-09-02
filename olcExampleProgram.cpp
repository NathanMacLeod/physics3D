#define OLC_PGE_APPLICATION
#include "olcPixelGameEngine.h"
#include "Vector3D.h"
#include <thread>
#include <stdio.h>
#include <vector>
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
	Vector3D* p1 = new Vector3D(-x / 2.0, -y / 2.0, -z / 2.0);
	Vector3D* p2 = new Vector3D(-x / 2.0, y / 2.0, -z / 2.0);
	Vector3D* p3 = new Vector3D(x / 2.0, -y / 2.0, -z / 2.0);
	Vector3D* p4 = new Vector3D(x / 2.0, y / 2.0, -z / 2.0);
	Vector3D* p5 = new Vector3D(-x / 2.0, -y / 2.0, z / 2.0);
	Vector3D* p6 = new Vector3D(-x / 2.0, y / 2.0, z / 2.0);
	Vector3D* p7 = new Vector3D(x / 2.0, -y / 2.0, z / 2.0);
	Vector3D* p8 = new Vector3D(x / 2.0, y / 2.0, z / 2.0);

	polygons->push_back(new Polygon3D(p1, p2, p3, olc::BLACK, olc::BLACK));
	polygons->push_back(new Polygon3D(p2, p4, p3, olc::BLACK, olc::BLACK));

	polygons->push_back(new Polygon3D(p5, p7, p6, olc::BLACK, olc::BLACK));
	polygons->push_back(new Polygon3D(p6, p7, p8, olc::BLACK, olc::BLACK));

	polygons->push_back(new Polygon3D(p3, p4, p7, olc::BLACK, olc::BLACK));
	polygons->push_back(new Polygon3D(p7, p4, p8, olc::BLACK, olc::BLACK));

	polygons->push_back(new Polygon3D(p1, p5, p2, olc::BLACK, olc::BLACK));
	polygons->push_back(new Polygon3D(p5, p6, p2, olc::BLACK, olc::BLACK));

	polygons->push_back(new Polygon3D(p2, p6, p4, olc::BLACK, olc::BLACK));
	polygons->push_back(new Polygon3D(p6, p8, p4, olc::BLACK, olc::BLACK));

	polygons->push_back(new Polygon3D(p1, p3, p5, olc::BLACK, olc::BLACK));
	polygons->push_back(new Polygon3D(p5, p3, p7, olc::BLACK, olc::BLACK));

	Vector3D translation(xPos, yPos, zPos);
	std::vector<Vector3D*> points;
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
		Vector3D v1 = polygon->p2->sub(*polygon->p1);
		Vector3D v2 = polygon->p3->sub(*polygon->p1); 
		Vector3D normalVector = (v1.crossProduct(v2)).getUnitVector();
		std::vector<Vector3D*>* points = new std::vector<Vector3D*>();
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
	Vector3D cameraPos;
	PhysicsEngine pEngine = PhysicsEngine(0.02);
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
	
	void drawPixel3D(int x, int y, double z, olc::Pixel color) {
		int indx = getPixelIndex(x, y);
		if (zBuffer[indx] > 0 && zBuffer[indx] <= z)
			return;
		zBuffer[indx] = z;
		Draw(x, y, color);
	}

	void drawLine3D(double x1, double y1, double z1, double x2, double y2, double z2, olc::Pixel color) {
		int changeX = x2 - x1;
		int changeY = y2 - y1;

		int absChangeX = abs(changeX);
		int absChangeY = abs(changeY);

		if (x1 >= 0 && x1 < ScreenWidth() && y1 >= 0 && y1 < ScreenHeight())
			Draw(x1, y1, color);
		if (x2 >= 0 && x2 < ScreenWidth() && y2 >= 0 && y2 < ScreenHeight())
			Draw(x2, y2, color);

		//Iterate along the longer dimension to avoid gaps in line
		if (absChangeX > absChangeY) {
			float m = (float)changeY / changeX;
			for (int i = 0; i < absChangeX; i++) {
				int dx = (changeX < 0) ? -i : i;
				float dy = dx * m;
				int x = x1 + dx;
				int y = y1 + dy;
				if (x >= 0 && x < ScreenWidth() && y >= 0 && y < ScreenHeight())
					drawPixel3D(x, y, -0.2 + z1 + (z2 - z1) * dx / absChangeX, color);
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
					drawPixel3D(x, y, -0.2 + z1 + (z2 - z1) * dy / absChangeY, color);
			}
		}
	}

	/*void drawTriangle(double x1, double y1, double x2, double y2, double x3, double y3, olc::Pixel color) {
		DrawLine(x1, y1, x2, y2, color);
		DrawLine(x2, y2, x3, y3, color);
		DrawLine(x3, y3, x1, y1, color);
	}*/

	void drawTriangle3D(const Vector3D p1, const Vector3D p2, const Vector3D p3, olc::Pixel color) {
		drawLine3D(p1.x, p1.y, p1.z, p2.x, p2.y, p2.z, color);
		drawLine3D(p2.x, p2.y, p2.z, p3.x, p3.y, p3.z, color);
		drawLine3D(p3.x, p3.y, p1.z, p1.x, p1.y, p1.z, color);
	}

	/*void fillTriangle(double x1, double y1, double x2, double y2, double x3, double y3, olc::Pixel color) {
		Vector3D p1(x1, y1, 0);
		Vector3D p2(x2, y2, 0);
		Vector3D p3(x3, y3, 0);
		fillTriangle3D(p1, p2, p3, color);
	}*/

	void fillTriangle3D(const Vector3D p1, const Vector3D p2, const Vector3D p3, olc::Pixel color, const Vector3D p1Pre, const Vector3D p2Pre, const Vector3D p3Pre, double fov, bool useZBuffer) {
		Vector3D normalVector;
		double invNz = 0;
		//double a, b, c, d;
		Vector3D v1Pre = p2Pre.sub(p1Pre);
		Vector3D v2Pre = p3Pre.sub(p1Pre);
		if (useZBuffer) {
			Vector3D v1 = p2.sub(p1);
			Vector3D v2 = p3.sub(p1);

			//b = v1.y / (v2.x * v1.y - v2.y * v1.x);
			//a = (1 - b * v2.x) / v1.x;

			//d = v1.x / (v2.y * v1.x - v2.x * v1.y);
			//c = (1 - d * v2.y) / v1.y;

			normalVector = v1Pre.crossProduct(v2Pre);
			invNz = 1.0 / normalVector.z;
			//printf("%f %f %f\n %f %f %f\n %f %f %f\n %f\n\n", p1Pre.x, p1Pre.y, p1Pre.z, p2Pre.x, p2Pre.y, p2Pre.z, p3Pre.x, p3Pre.y, p3Pre.z, invNz);
		}
		const Vector3D* upperPoint = &p1;
		const Vector3D* midPoint = &p2;
		const Vector3D* lowerPoint = &p3;

		const Vector3D* temp = nullptr;
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
						//double x = a * (j - p1.x) + b * (i - p1.y);
						//double y = c * (j - p1.x) + d * (i - p1.y);

						double dy = i - ScreenHeight() / 2.0;
						double dx = j - ScreenWidth() / 2.0;

						double z; 
						if (normalVector.y == 0) {
							z = p1Pre.z;
						}
						else {
							z = (p1Pre.y + p1Pre.z * normalVector.z / normalVector.y) / (normalVector.z / normalVector.y + dy / fov);
						}
						if(normalVector.x != 0) {
							z = (p1Pre.x + z * normalVector.z / normalVector.x) / (normalVector.z / normalVector.x + dx / fov);
						}
						//z += (p1.x - j) * (fov + z) / (fov * (-normalVector.z / normalVector.y - (j - ScreenWidth() / 2.0) / fov));
						//double z = p1Pre.z + (j - p1.x) * (a * v1Pre.z + b * v2Pre.z) + (i - p1.y) * (c * v1Pre.z + d * v2Pre.z);

						int col = 255 * z / 200;
						if (col > 255) {
							col = 255;
						}
						bool red = col < 0;

						//if (red) {
						//	red = red;
						//}

						drawPixel3D(j, i, z, olc::Pixel(red ? 255 : col, red ? 0 : col, red ? 0 : col));
					}
					else {
						Draw(j, i, color);
					}
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
					//double x = a * (j - p1.x) + b * (i - p1.y);
					//double y = c * (j - p1.x) + d * (i - p1.y);

					double dy = i - ScreenHeight() / 2.0;
					double dx = j - ScreenWidth() / 2.0;

					double z;
					if (normalVector.y == 0) {
						z = p1Pre.z;
					}
					else {
						z = (p1Pre.y + p1Pre.z * normalVector.z / normalVector.y) / (normalVector.z / normalVector.y + dy / fov);
					}
					if (normalVector.x != 0) {
						z = (p1Pre.x + z * normalVector.z / normalVector.x) / (normalVector.z / normalVector.x + dx / fov);
					}
					
					int col = 255 * z / 200;
					if (col > 255) {
						col = 255;
					}
					bool red = col < 0;
					

					drawPixel3D(j, i, z, olc::Pixel(red? 255 : col, red? 0 : col, red? 0 : col));
				}
				else {
					Draw(j, i, color);
				}
			}
		}
	}

	void projectPoint(Vector3D* point, float fov) {
		point->x = point->x * fov / point->z + ScreenWidth() / 2;
		point->y = point->y * fov / point->z + ScreenHeight() / 2;
	}

	/*Color getPhongLighting(Color surfColor, Vector3D& camDir, Vector3D& surfNV, Vector3D& lightDir) {
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
	}*/

	struct Tri {
		Vector3D p1;
		Vector3D p2;
		Vector3D p3;

		Tri(Vector3D p1, Vector3D p2, Vector3D p3) {
			this->p1 = p1;
			this->p2 = p2;
			this->p3 = p3;
		}
	};

	Vector3D clipLine(Vector3D p1, Vector3D p2) {
		double clipZ = 1;

		double dz = p2.z - p1.z;
		if (dz == 0) {
			return p1;
		}
		double dx = p2.x - p1.x;
		double dy = p2.y - p1.y;

		return Vector3D(p1.x + (clipZ - p1.z) * dx / dz, p1.y + (clipZ - p1.z) * dy / dz, clipZ);
	}

	void clipPolygon(Polygon3D* poly, std::vector<Tri>* out) {
		Vector3D minZ = *poly->tempP1;
		Vector3D midZ = *poly->tempP2;
		Vector3D maxZ= *poly->tempP3;

		if (minZ.z > midZ.z) {
			Vector3D tmp = minZ;
			minZ = midZ;
			midZ = tmp;
		}
		if (midZ.z > maxZ.z) {
			Vector3D tmp = midZ;
			midZ = maxZ;
			maxZ = tmp;

			if (minZ.z > midZ.z) {
				Vector3D tmp = minZ;
				minZ = midZ;
				midZ = tmp;
			}
		}

		if (maxZ.z <= 0)
			return;
		else if (midZ.z <= 0) {
			out->push_back(Tri(maxZ, clipLine(maxZ, midZ), clipLine(maxZ, minZ)));
			return;
		}
		else if (minZ.z <= 0){
			Vector3D clipP1 = clipLine(maxZ, minZ);
			Vector3D clipP2 = clipLine(midZ, minZ);
			out->push_back(Tri(maxZ, clipP1, midZ));
			out->push_back(Tri(midZ, clipP2, clipP1));
			return;
		}
		else {
			out->push_back(Tri(maxZ, midZ, minZ));
		}
	}

	void drawPolygons(const std::vector<Polygon3D*> polygons, const Vector3D cameraPosition, float orientationYAng, float orientationPitch, float fov, bool drawLines) {
		clearZBuffer();
		//copy reference to copy of polygon points

		//Vector3D cameraVector(cos(orientationYAng) * cos(orientationPitch), sin(orientationPitch), sin(orientationYAng) * cos(orientationPitch));
		//std::vector<Vector3D*> points;
		//std::vector<Polygon3D*> includedPolygons;

		for (Polygon3D* polygon : polygons) {
			std::vector<Vector3D*> points;
			Vector3D cameraToPolygon(polygon->p2->x - cameraPosition.x, polygon->p2->y - cameraPosition.y, polygon->p2->z - cameraPosition.z);
			Vector3D p1p2(polygon->p2->x - polygon->p1->x, polygon->p2->y - polygon->p1->y, polygon->p2->z - polygon->p1->z);
			Vector3D p1p3(polygon->p3->x - polygon->p1->x, polygon->p3->y - polygon->p1->y, polygon->p3->z - polygon->p1->z);
			Vector3D normalVector = p1p2.crossProduct(p1p3);

			if (cameraToPolygon.dotProduct(normalVector) < 0) {
				polygon->copyPointsToTemp();
				points.push_back(polygon->tempP1);
				points.push_back(polygon->tempP2);
				points.push_back(polygon->tempP3);

				Vector3D cameraPositionToOrigin(-cameraPosition.x, -cameraPosition.y, -cameraPosition.z);
				transformation3D::translatePoints(&points, cameraPositionToOrigin);
				transformation3D::rotatePointsAroundYParralelAxis(&points, -orientationYAng + 1.570795, 0, 0);
				transformation3D::rotatePointsAroundXParralelAxis(&points, -orientationPitch, 0, 0);

				std::vector<Tri> clips;
				clipPolygon(polygon, &clips);

				for (Tri t : clips) {
					Vector3D p1Pre = t.p1;
					Vector3D p2Pre = t.p2;
					Vector3D p3Pre = t.p3;

					projectPoint(&t.p1, fov);
					projectPoint(&t.p2, fov);
					projectPoint(&t.p3, fov);

					fillTriangle3D(t.p1, t.p2, t.p3, polygon->color, p1Pre, p2Pre, p3Pre, fov, true);
					if (drawLines)
						drawTriangle3D(t.p1, t.p2, t.p3, olc::WHITE);
				}
				
			}
		}

		/*Vector3D cameraPositionToOrigin(-cameraPosition.x, -cameraPosition.y, -cameraPosition.z);
		//translate points to origin
		transformation3D::translatePoints(&points, cameraPositionToOrigin);
		//rotate so that world so that camera stares towards z axis
		transformation3D::rotatePointsAroundYParralelAxis(&points, -orientationYAng + 1.570795, 0, 0);
		//pitch world so to that camera is inline with z axis
		transformation3D::rotatePointsAroundXParralelAxis(&points, -orientationPitch, 0, 0);
		for (Vector3D* p : points) {
			projectPoint(p, fov);
			//drawPiczel(p->x, p->y, PICZEL, white);
		}

		//TEMPORARY FOR PHONG LIGHTING TEST
		//Vector3D camDir(cos(orientationYAng + 1.570795), sin(orientationPitch), cos(orientationPitch) * sin(orientationYAng + 1.570795));

		for (Polygon3D* polygon : includedPolygons) {

			//TODO IMPLEMENT NOT DUMB
			//Vector3D p1p2(polygon->p2->x - polygon->p1->x, polygon->p2->y - polygon->p1->y, polygon->p2->z - polygon->p1->z);
			//Vector3D p1p3(polygon->p3->x - polygon->p1->x, polygon->p3->y - polygon->p1->y, polygon->p3->z - polygon->p1->z);
			//Vector3D normalVector = p1p2.crossProduct(p1p3).getUnitVector();
			//Vector3D lightDir(1, -1, -1);
			//lightDir = lightDir.getUnitVector();

			//FillTriangle(polygon->tempP1->x, polygon->tempP1->y, polygon->tempP2->x, polygon->tempP2->y, polygon->tempP3->x, polygon->tempP3->y, olc::Pixel(getR(color), getG(color), getB(color)));
			
			fillTriangle3D(*(polygon->tempP1), *(polygon->tempP2), *(polygon->tempP3), polygon->color, true);
			if (drawLines)
				drawTriangle3D(*(polygon->tempP1), *(polygon->tempP2), *(polygon->tempP3), olc::WHITE);
		}*/
	}

	void draw3DLine(Vector3D p1, Vector3D p2, const Vector3D cameraPosition, float orientationYAng, float orientationPitch, float fov) {
		Vector3D cameraPositionToOrigin(-cameraPosition.x, -cameraPosition.y, -cameraPosition.z);

		transformation3D::translatePoint(&p1, cameraPositionToOrigin);
		transformation3D::rotatePointAroundYParralelAxis(&p1, -orientationYAng + 1.570795, 0, 0);
		transformation3D::rotatePointAroundXParralelAxis(&p1, -orientationPitch, 0, 0);
		projectPoint(&p1, fov);

		transformation3D::translatePoint(&p2, cameraPositionToOrigin);
		transformation3D::rotatePointAroundYParralelAxis(&p2, -orientationYAng + 1.570795, 0, 0);
		transformation3D::rotatePointAroundXParralelAxis(&p2, -orientationPitch, 0, 0);
		projectPoint(&p2, fov);

		drawLine3D(p1.x, p1.y, p1.z, p2.x, p2.y, p2.z, olc::GREEN);
	}

	void draw3DPoint(Vector3D p,const Vector3D cameraPosition, float orientationYAng, float orientationPitch, float fov) {
		Vector3D cameraPositionToOrigin(-cameraPosition.x, -cameraPosition.y, -cameraPosition.z);

		transformation3D::translatePoint(&p, cameraPositionToOrigin);
		transformation3D::rotatePointAroundYParralelAxis(&p, -orientationYAng + 1.570795, 0, 0);
		transformation3D::rotatePointAroundXParralelAxis(&p, -orientationPitch, 0, 0);
		projectPoint(&p, fov);

		Draw((int)p.x, (int)p.y, olc::RED);
	}

	bool OnUserCreate() override
	{
		zBuffer = new double[ScreenWidth() * ScreenHeight()];
		allPolygons = new std::vector<Polygon3D*>();
		cameraPos.z = -150;
		//cameraPos.y = 27.5;
		//cameraPos.x = 52;

		std::vector<Polygon3D*>* b2polygons = createBox(300, 5, 290, 0, 35, 0);
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
		if (dropTime > 2) {
			dropTime = 0;
			double x = 5;// 52;
			double y = -50;
			double z = 0;
			std::vector<Polygon3D*>* b1polygons = createBox(12, 12, 12, x, y, z);

			std::vector<Vector3D*> points;
			for (Polygon3D* polygon : *b1polygons) {
				allPolygons->push_back(polygon);
				bool p1 = true;
				bool p2 = true;
				bool p3 = true;
				for (Vector3D* p : points) {
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

			std::vector<Vector3D*> b2points;
			for (Polygon3D* polygon : *b2polygons) {
				//allPolygons->push_back(polygon);
				bool p1 = true;
				bool p2 = true;
				bool p3 = true;
				for (Vector3D* p : points) {
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

			std::vector<Vector3D*> b3points;
			for (Polygon3D* polygon : *b3polygons) {
				//allPolygons->push_back(polygon);
				bool p1 = true;
				bool p2 = true;
				bool p3 = true;
				for (Vector3D* p : points) {
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
			//hulls->push_back(new ConvexHull(*createRigidBodyFromPolygons(*b3polygons), 1));
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

		pEngine.iterateEngine(fElapsedTime);
		auto t2 = std::chrono::system_clock::now();

		Clear(olc::VERY_DARK_GREY);
		auto t3 = std::chrono::system_clock::now();
		clearZBuffer();
		auto t4 = std::chrono::system_clock::now();
		drawPolygons(*allPolygons, cameraPos, yAng, xAng, 400, false);

		//draw octree
		/*std::vector<OctreeNode*> nodes;
		OctreeNode::getAllNodes(&pEngine.getOctreeRoot(), &nodes);
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

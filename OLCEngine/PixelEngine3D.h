#pragma once
#include "olcPixelGameEngine.h"
#include "../Math/Vector3D.h"
#include <thread>
#include <stdio.h>
#include <vector>
#include <list>
#include <chrono>
#include "Polygon3D.h"
#include "../Math/transformation3D.h"
#include "../Math/Rotor.h"

class PixelEngine3D : public olc::PixelGameEngine {

protected:

	void clearZBuffer() {
		for (int i = 0; i < ScreenWidth() * ScreenHeight(); i++) {
			zBuffer[i] = -1;
		}
	}

	void initZBuffer() {
		zBuffer = new double[ScreenWidth() * ScreenHeight()];
	}

private:

	double* zBuffer;

	int getPixelIndex(int x, int y) {
		return x + y * ScreenWidth();
	}

	void drawLine3D(double x1, double y1, double x2, double y2, double fov, Vector3D p1Pre, Vector3D p2Pre, olc::Pixel color) {
		double weight = -0.3; //help draw lines over z buffer from the faces

		int absChangeX = abs(x2 - x1);
		int absChangeY = abs(y2 - y1);

		Vector3D l = p2Pre.sub(p1Pre);
		Vector3D n;
		if (absChangeY > absChangeX) {
			n = l.crossProduct(Vector3D(1, 0, 0));
		}
		else {
			n = l.crossProduct(Vector3D(0, 1, 0));
		}
		double k = fov * n.dotProduct(p1Pre);
		//Iterate along the longer dimension to avoid gaps in line
		if (absChangeX > absChangeY) {

			if (x2 < x1) {
				double tempx = x1;
				double tempy = y1;
				x1 = x2;
				y1 = y2;
				x2 = tempx;
				y2 = tempy;
			}

			float m = (float)((int)(y2 - y1)) / (int) (x2 - x1);
			for (int x = std::max<int>(0, x1); x <= std::min<int>(ScreenWidth() - 1, x2); x++) {

				int y = y1 + m * (x - x1);

				if (y >= 0 && y < ScreenHeight()) {
					double z = -1;
					if (p1Pre.z == p2Pre.z) {
						z = p1Pre.z;
					}
					else {
						double sX = x - ScreenWidth() / 2.0;
						double sY = y - ScreenHeight() / 2.0;
						Vector3D v(sX, sY, fov);
						double t = n.dotProduct(p1Pre) / n.dotProduct(v);
						z = v.z * t;
						//z = k / (n.x * sX + n.y * sY + n.z * fov);
					}

					int col = 255 * z / 250;
					if (col > 255) {
						col = 255;
					}
					bool red = col < 0;

					drawPixel3D(x, y, z + weight, color);
					//drawPixel3D(x, y, z + weight, olc::Pixel(red ? 255 : col, red ? 0 : col, red ? 0 : col));
				}
			}
		}
		else {

			if (y2 < y1) {
				double tempx = x1;
				double tempy = y1;
				x1 = x2;
				y1 = y2;
				x2 = tempx;
				y2 = tempy;
			}

			float k = (float)((int)(x2 - x1)) / (int)(y2 - y1);
			for (int y = std::max<int>(0, y1); y <= std::min<int>(ScreenHeight() - 1, y2); y++) {
				int x = x1 + k * (y - y1);

				if (x >= 0 && x < ScreenWidth()) {
					double z = -1;
					if (p1Pre.z == p2Pre.z) {
						z = p1Pre.z;
					}
					else {
						double sX = x - ScreenWidth() / 2.0;
						double sY = y - ScreenHeight() / 2.0;
						Vector3D v(sX, sY, fov);
						double t = n.dotProduct(p1Pre) / n.dotProduct(v);
						z = v.z * t;
						//z = k / (n.x * sX + n.y * sY + n.z * fov);
					}

					int col = 255 * z / 250;
					if (col > 255) {
						col = 255;
					}
					bool red = col < 0;

					drawPixel3D(x, y, z + weight, color);
				}
			}
		}
	}

	void drawTriangle3D(const Vector3D p1, const Vector3D p2, const Vector3D p3, double fov, Vector3D p1Pre, Vector3D p2Pre, Vector3D p3Pre, olc::Pixel color) {
		drawLine3D(p1.x, p1.y, p2.x, p2.y, fov, p1Pre, p2Pre, color);
		drawLine3D(p2.x, p2.y, p3.x, p3.y, fov, p2Pre, p3Pre, color);
		drawLine3D(p3.x, p3.y, p1.x, p1.y, fov, p3Pre, p1Pre, color);
	}

	void fillTriangle3D(const Vector3D p1, const Vector3D p2, const Vector3D p3, olc::Pixel color, const Vector3D p1Pre, const Vector3D p2Pre, const Vector3D p3Pre, double fov, bool useZBuffer) {
		Vector3D normalVector;
		double invNz = 0;
		double k;
		Vector3D v1Pre = p2Pre.sub(p1Pre);
		Vector3D v2Pre = p3Pre.sub(p1Pre);
		if (useZBuffer) {
			Vector3D v1 = p2.sub(p1);
			Vector3D v2 = p3.sub(p1);

			normalVector = v1Pre.crossProduct(v2Pre);
			k = fov * normalVector.dotProduct(p1Pre);
			invNz = 1.0 / normalVector.z;
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

			for (int i = std::max<int>(0, (int)(upperPoint->y)); i <= std::min<int>((int)(midPoint->y), ScreenHeight() - 1); i++) {

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
						double dy = i - ScreenHeight() / 2.0;
						double dx = j - ScreenWidth() / 2.0;

						//Vector3D v(dx, dy, fov);
						//double t = normalVector.dotProduct(p1Pre) / normalVector.dotProduct(v);
						double z = k / (normalVector.x * dx + normalVector.y * dy + normalVector.z * fov);

						int col = 255 * z / 800;
						if (col > 255) {
							col = 255;
						}
						//bool red = col < 0;

						//drawPixel3D(j, i, z, olc::Pixel(col, col, col));
						drawPixel3D(j, i, z, color);
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

		for (int i = std::min<int>(ScreenHeight() - 1, (int)lowerPoint->y); i > std::max<int>(0, (int)(midPoint->y)); i--) {

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

					double dy = i - ScreenHeight() / 2.0;
					double dx = j - ScreenWidth() / 2.0;

					//Vector3D v(dx, dy, fov);
					//double t = normalVector.dotProduct(p1Pre) / normalVector.dotProduct(v);
					double z = k / (normalVector.x * dx + normalVector.y * dy + normalVector.z * fov);

					int col = 255 * z / 800;
					if (col > 255) {
						col = 255;
					}
					//bool red = col < 0;

					//drawPixel3D(j, i, z, olc::Pixel(col, col, col));
					drawPixel3D(j, i, z, color);
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

	void clipPolygon(Tri* poly, std::vector<Tri>* out, double fov) {
		Vector3D minZ = poly->p1;
		Vector3D midZ = poly->p2;
		Vector3D maxZ = poly->p3;
		double maxX;
		double minX;
		double maxY;
		double minY;

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

		if (maxZ.z <= 0) {
			return; //returning early results in nothing in out and thus isnt drawn
		}

		if (midZ.z <= 0) {
			out->push_back(Tri(maxZ, clipLine(maxZ, midZ), clipLine(maxZ, minZ)));
			return;
		}
		else if (minZ.z <= 0) {
			Vector3D clipP1 = clipLine(maxZ, minZ);
			Vector3D clipP2 = clipLine(midZ, minZ);
			out->push_back(Tri(maxZ, clipP1, midZ));
			out->push_back(Tri(clipP2, clipP1, midZ));
			return;
		}
		else {
			out->push_back(Tri(maxZ, midZ, minZ));
		}
	}

	void drawPixel3D(int x, int y, double z, olc::Pixel color) {
		int indx = getPixelIndex(x, y);
		if (zBuffer[indx] > 0 && zBuffer[indx] <= z)
			return;
		zBuffer[indx] = z;
		Draw(x, y, color);
	}

public:

	~PixelEngine3D() {
		delete zBuffer;
	}

	void drawPolygon(Polygon3D& polygon, Vector3D cameraPosition, Rotor cameraOrientation, float fov, bool drawLines) {
		Vector3D cameraToPolygon(polygon.p2.x - cameraPosition.x, polygon.p2.y - cameraPosition.y, polygon.p2.z - cameraPosition.z);
		Vector3D p1p2(polygon.p2.x - polygon.p1.x, polygon.p2.y - polygon.p1.y, polygon.p2.z - polygon.p1.z);
		Vector3D p1p3(polygon.p3.x - polygon.p1.x, polygon.p3.y - polygon.p1.y, polygon.p3.z - polygon.p1.z);
		Vector3D normalVector = p1p2.crossProduct(p1p3);

		if (cameraToPolygon.dotProduct(normalVector) < 0) {
			Tri copy(polygon.p1, polygon.p2, polygon.p3);

			Vector3D cameraPositionToOrigin(-cameraPosition.x, -cameraPosition.y, -cameraPosition.z);
			copy.p1 = copy.p1.add(cameraPositionToOrigin);
			copy.p2 = copy.p2.add(cameraPositionToOrigin);
			copy.p3 = copy.p3.add(cameraPositionToOrigin);

			Rotor rotate = cameraOrientation.getInverse();
			copy.p1 = rotate.rotate(copy.p1);
			copy.p2 = rotate.rotate(copy.p2);
			copy.p3 = rotate.rotate(copy.p3);

			std::vector<Tri> clips;
			clipPolygon(&copy, &clips, fov);

			for (Tri t : clips) {
				Vector3D p1Pre = t.p1;
				Vector3D p2Pre = t.p2;
				Vector3D p3Pre = t.p3;

				projectPoint(&t.p1, fov);
				projectPoint(&t.p2, fov);
				projectPoint(&t.p3, fov);

				fillTriangle3D(t.p1, t.p2, t.p3, polygon.color, p1Pre, p2Pre, p3Pre, fov, true);

				if (drawLines) {
					if (clips.size() == 2) { //avoid drawing extra line in case that polygon is split into 2 new triangles
						drawLine3D(t.p1.x, t.p1.y, t.p2.x, t.p2.y, fov, p1Pre, p2Pre, polygon.lineColor);
						drawLine3D(t.p1.x, t.p1.y, t.p3.x, t.p3.y, fov, p1Pre, p3Pre, polygon.lineColor);
					}
					else {
						drawTriangle3D(t.p1, t.p2, t.p3, fov, p1Pre, p2Pre, p3Pre, polygon.lineColor);
					}
				}
			}

		}
	}

	void drawPolygons(const std::vector<Polygon3D>* polygons, const Vector3D cameraPosition, Rotor cameraOrientation, float fov, bool drawLines) {

		for (Polygon3D polygon : *polygons) {
			drawPolygon(polygon, cameraPosition, cameraOrientation, fov, drawLines);
		}
	}

	void draw3DLine(Vector3D p1, Vector3D p2, const Vector3D cameraPosition, Rotor cameraOrientation, float fov, olc::Pixel color) {

		Vector3D cameraPositionToOrigin(-cameraPosition.x, -cameraPosition.y, -cameraPosition.z);
		Rotor rotate = cameraOrientation.getInverse();

		transformation3D::translatePoint(&p1, cameraPositionToOrigin);
		p1 = rotate.rotate(p1);

		transformation3D::translatePoint(&p2, cameraPositionToOrigin);
		p2 = rotate.rotate(p2);

		Vector3D maxZ = p1;
		Vector3D minZ = p2;
		if (p2.z > p1.z) {
			maxZ = p2;
			minZ = p1;
		}

		if (maxZ.z < 0)
			return;

		else if (minZ.z < 0) {
			minZ = clipLine(maxZ, minZ);
		}

		Vector3D maxZPre = maxZ;
		Vector3D minZPre = minZ;

		projectPoint(&maxZ, fov);
		projectPoint(&minZ, fov);

		drawLine3D(maxZ.x, maxZ.y, minZ.x, minZ.y, fov, maxZPre, minZPre, color);
	}

	Vector3D getPixelCoord(Vector3D p, const Vector3D cameraPosition, Rotor cameraDir, float fov) {
		Vector3D cameraPositionToOrigin(-cameraPosition.x, -cameraPosition.y, -cameraPosition.z);

		transformation3D::translatePoint(&p, cameraPositionToOrigin);

		Rotor rotate = cameraDir.getInverse();
		p = rotate.rotate(p);

		projectPoint(&p, fov);

		return Vector3D(p.x, p.y, 0);
	}

	void draw3DPoint(Vector3D p, const Vector3D cameraPosition, Rotor cameraDir, float fov, olc::Pixel color, bool skipZ=false) {
		Vector3D cameraPositionToOrigin(-cameraPosition.x, -cameraPosition.y, -cameraPosition.z);

		transformation3D::translatePoint(&p, cameraPositionToOrigin);

		Rotor rotate = cameraDir.getInverse();
		p = rotate.rotate(p);

		projectPoint(&p, fov);

		int x = (int)p.x;
		int y = (int)p.y;

		if (p.z <= 0 || x < 0 || x >= ScreenWidth() || y < 0 || y >= ScreenHeight()) {
			return;
		}

		drawPixel3D(x, y, skipZ? 0 : p.z, color);
	}
};
#ifndef CONSOLEENGINE_H
#define CONSOLEENGINE_H

#include <Windows.h>
#include <iostream>
#include <stdio.h>
#include "Point.h"
#include "Color.h"
#include "Vector3D.h"
#include <vector>
#include <list>
#include "Polygon3D.h"
#include "transformation3D.h"

class ConsoleEngine {
	CHAR_INFO* screen;
	double* zBuffer;
	HANDLE console;
	DWORD dwBytesWritten;
	SMALL_RECT rectangle;
	Color currentColor;
	int sWidth;
	int sHeight;
	int piczelSize;
	void clearZBuffer() {
		for (int i = 0; i < sWidth * sHeight; i++) {
			zBuffer[i] = -1;
		}
	}
public:
	ConsoleEngine(int width, int height, int piczelSize) {

		console = CreateConsoleScreenBuffer(GENERIC_READ | GENERIC_WRITE, 0, NULL, CONSOLE_TEXTMODE_BUFFER, NULL);
		sWidth = width;
		sHeight = height;
		rectangle = { 0, 0, (short)sWidth - 1, (short)sHeight - 1 };
		screen = new CHAR_INFO[width * height];
		zBuffer = new double[width * height];
		clearZBuffer();

		SMALL_RECT rect = { 0, 0, short((width - 1) * piczelSize), short((height - 1) * piczelSize)};
		SetConsoleWindowInfo(console, TRUE, &rectangle);

		COORD coord = { (short)sWidth, (short)sHeight};
		SetConsoleScreenBufferSize(console, coord);

		CONSOLE_FONT_INFOEX cfi;
		cfi.cbSize = sizeof(cfi);
		cfi.nFont = 0;
		cfi.dwFontSize.X = piczelSize;
		cfi.dwFontSize.Y = piczelSize;
		cfi.FontFamily = FF_DONTCARE;
		cfi.FontWeight = FW_NORMAL;

		if (!SetConsoleScreenBufferSize(console, coord))
			std::cout << "Failed to set size" << "\n";

		if (!SetCurrentConsoleFontEx(console, false, &cfi))
			std::cout << "Failed to load font" << "\n";

		SetConsoleActiveScreenBuffer(console);

	}
	~ConsoleEngine() {
		delete[] screen;
	}

	const short PICZEL = 0x2588;

	void setCurrentColor(Color c) {
		currentColor = c;
	}

	Color getCurrentColor() {
		return currentColor;
	}

	void outputScreen() {
		WriteConsoleOutput(console, screen, { (short)sWidth, (short)sHeight }, { 0,0 }, &rectangle);
	}

	int getPiczelIndex(int x, int y) {
		if (x < 0 || x >= sWidth || y < 0 || y >= sHeight)
			std::cout << "Illegal coordiantes entered" << std::endl;
		return x + y * sWidth;
	}

	void drawPiczel(int x, int y, short character, Color color) {
		drawPiczel(getPiczelIndex(x, y), character, color);
	}

	void drawPiczel(int index, short character, Color color) {
		screen[index].Char.UnicodeChar = character;
		screen[index].Attributes = color;
	}

	void drawLine(float x1, float y1, float x2, float y2, short character, Color color) {
		int changeX = x2 - x1;
		int changeY = y2 - y1;

		int absChangeX = abs(changeX);
		int absChangeY = abs(changeY);

		if (x1 >= 0 && x1 < sWidth && y1 >= 0 && y1 < sHeight)
			drawPiczel(x1, y1, character, color);
		if (x2 >= 0 && x2 < sWidth && y2 >= 0 && y2 < sHeight)
			drawPiczel(x2, y2, character, color);

		//Iterate along the longer dimension to avoid gaps in line
		if (absChangeX > absChangeY) {
			float m = (float) changeY / changeX;
			for (int i = 0; i < absChangeX; i++) {
				int dx = (changeX < 0) ? -i : i;
				float dy = dx * m;
				int x = x1 + dx;
				int y = y1 + dy;
				if(x >= 0 && x < sWidth && y >= 0 && y < sHeight) 
					drawPiczel(x, y, character, color);
			}
		}
		else {
			float k = (float) changeX / changeY;
			for (int i = 0; i < absChangeY; i++) {
				int dy = (changeY < 0) ? -i : i;
				float dx = dy * k;
				int x = x1 + dx;
				int y = y1 + dy;
				if (x >= 0 && x < sWidth && y >= 0 && y < sHeight)
					drawPiczel(x, y, character, color);
			}
		}
	}

	void drawTriangle(float x1, float y1, float x2, float y2, float x3, float y3, short character, Color color) {
		drawLine(x1, y1, x2, y2, character, color);
		drawLine(x2, y2, x3, y3, character, color);
		drawLine(x3, y3, x1, y1, character, color);
	}

	void drawTriangle(const Point3D p1, const Point3D p2, const Point3D p3, short character, Color color) {
		drawTriangle(p1.x, p1.y, p2.x, p2.y, p3.x, p3.y, character, color);
	}

	void fillTriangle(float x1, float y1, float x2, float y2, float x3, float y3, short character, Color color) {
		Point3D p1(x1, y1);
		Point3D p2(x2, y2);
		Point3D p3(x3, y3);
		fillTriangle(p1, p2, p3, character, color);
	}

	void fillTriangle(const Point3D p1, const Point3D p2, const Point3D p3, short character, Color color, bool useZBuffer=false) {
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

		int heightIndex = (int) (upperPoint->y) * sWidth;

		if ((int)(upperPoint->y) != (int)(midPoint->y)) {

			for (int i = (int)(upperPoint->y); i <= (int)(midPoint->y); i++) {
				//std::cout << "I:" << i << " height:" << heightIndex << "\n";
				if (i < 0 || i >= sHeight) {
					heightIndex += sWidth;
					continue;
				}
				int leftBound = leftMostSlope * (i - upperPoint->y) + upperPoint->x;
				int rightBound = rightMostSlope * (i - upperPoint->y) + upperPoint->x;

				if (leftBound < 0)
					leftBound = 0;
				else if (leftBound >= sWidth) {
					heightIndex += sWidth;
					continue;
				}
				if (rightBound >= sWidth)
					rightBound = sWidth - 1;
				else if (rightBound < 0) {
					heightIndex += sWidth;
					continue;
				}

				for (int j = leftBound; j <= rightBound; j++) {
					if (useZBuffer) {
						double z = p1.z - invNz * (normalVector.x * (j - p1.x) + normalVector.y * (i - p1.y));
						if (zBuffer[j + heightIndex] > 0 && zBuffer[j + heightIndex] <= z)
							continue;
						zBuffer[j + heightIndex] = z;
					}
					drawPiczel(j + heightIndex, character, color);
				}

				heightIndex += sWidth;
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

		heightIndex = (int) (lowerPoint->y) * sWidth;

		for (int i = (int)lowerPoint->y; i > (int)(midPoint->y); i--) {
			if (i < 0 || i >= sHeight) {
				heightIndex -= sWidth;
				continue;
			}

			int leftBound = leftMostSlope * (i - lowerPoint->y) + lowerPoint->x;
			int rightBound = rightMostSlope * (i - lowerPoint->y) + lowerPoint->x;

			
			if (leftBound < 0)
				leftBound = 0;
			else if (leftBound >= sWidth) {
				heightIndex -= sWidth;
				continue;
			}
			if (rightBound >= sWidth)
				rightBound = sWidth - 1;
			else if (rightBound < 0) {
				heightIndex -= sWidth;
				continue;
			}
			

			for (int j = leftBound; j <= rightBound; j++) {
				if (useZBuffer) {
					double z = p1.z - invNz * (normalVector.x * (j - p1.x) + normalVector.y * (i - p1.y));
					//std::cout << z << "\n";
					if (zBuffer[j + heightIndex] > 0 && zBuffer[j + heightIndex] <= z)
						continue;
					zBuffer[j + heightIndex] = z;
				}
				drawPiczel(j + heightIndex, character, color);
			}

			heightIndex -= sWidth;
		}
	}

	void fillRect(float x, float y, float width, float height, short character, Color color) {
		for (int i = 0; i < height; i++) {
			int startIndex = getPiczelIndex(x, y + i);
			for (int j = 0; j < width; j++) {
				drawPiczel(startIndex + j, character, color);
			}
		}
	}

	void projectPoint(Point3D* point, float fov) {
		point->x = point->x * fov / point->z + sWidth/2;
		point->y = point->y *fov / point->z + sHeight/2;
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
		for (Polygon3D* polygon : includedPolygons) {
			fillTriangle(*(polygon->tempP1), *(polygon->tempP2), *(polygon->tempP3), PICZEL, polygon->color, true);
			if(drawLines)
				drawTriangle(*(polygon->tempP1), *(polygon->tempP2), *(polygon->tempP3), PICZEL, polygon->p1p2Color);
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

		drawLine(p1.x, p1.y, p2.x, p2.y, PICZEL, red);
	}
	
};

#endif
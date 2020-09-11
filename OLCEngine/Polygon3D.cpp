#include "Polygon3D.h"

Polygon3D::Polygon3D(Vector3D p1, Vector3D p2, Vector3D p3, olc::Pixel lineColor, olc::Pixel color) {
	this->p1 = p1;
	this->p2 = p2;
	this->p3 = p3;
	this->color = color;
	this->lineColor = lineColor;
}
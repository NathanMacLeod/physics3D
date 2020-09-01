#include "Polygon3D.h"

Polygon3D::Polygon3D(Vector3D* p1, Vector3D* p2, Vector3D* p3, olc::Pixel lineColor, olc::Pixel color) {
	this->p1 = p1;
	this->p2 = p2;
	this->p3 = p3;
	this->tempP1 = new Vector3D();
	this->tempP2 = new Vector3D();
	this->tempP3 = new Vector3D();
	this->color = color;
	p1p2Color = lineColor;
	p2p3Color = lineColor;
	p3p1Color = lineColor;
}

void Polygon3D::copyPointsToTemp() {
	tempP1->x = p1->x;
	tempP1->y = p1->y;
	tempP1->z = p1->z;

	tempP2->x = p2->x;
	tempP2->y = p2->y;
	tempP2->z = p2->z;

	tempP3->x = p3->x;
	tempP3->y = p3->y;
	tempP3->z = p3->z;
}
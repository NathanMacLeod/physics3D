#pragma once
#include "../OLCEngine/PixelEngine3D.h"


class Drawable {
public:
	virtual void draw(PixelEngine3D* g, Vector3D cameraPos, Rotor cameraDir, double FOV) = 0;
};
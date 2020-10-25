#pragma once
#include "Drawable.h"
#include "Updatable.h"
#include "Expireable.h"
#include "CooldownTimer.h"

class Particle : public Drawable, public Updatable, public Expireable {
private:
	CooldownTimer life = CooldownTimer(0.8); // default
	Vector3D vel;
	Vector3D pos;
	olc::Pixel color;
public:
	Particle(float life, Vector3D vel, Vector3D pos, olc::Pixel color);
	void update(SpaceMinerGame* game, float fElapsedtime);
	void draw(PixelEngine3D* g, Vector3D cameraPos, Rotor cameraDir, double FOV);
	bool isExpired();

	static void generateExplosion(SpaceMinerGame* game, Vector3D pos, double time, double size, int nParticles, olc::Pixel color);
};
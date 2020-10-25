#pragma once
#include "PhysicsObject.h"
#include "Updatable.h"
#include "Expireable.h"
#include "CooldownTimer.h"
#include "MovingObject.h"
#include <string>

class Ore : public PhysicsObject, public Updatable, public Expireable{
public:
	static enum Material { Iron, Copper, Gold, Silver, Plutonium, Cybernium, Anthium, NormalDebris, EliteDebris };
	static const int N_TYPES = 9;

	Ore(Vector3D pos, Material type, MovingObject::DebrisType debrisType = MovingObject::HeadDebris);
	static double getValue(Material m);
	static Material pickTypeRandomly();
	static std::string getAbbrev(Material m);
	static std::string getName(Material m);
	static olc::Pixel getColor(Material m);
	
	Material getMaterial();
	void update(SpaceMinerGame* game, float fElapsedTime);
	void performDeathActions(SpaceMinerGame* game);
	void pickUp();
	bool isExpired();
	//void draw(PixelEngine3D* g, Vector3D cameraPos, Rotor cameraDir, double FOV);
private:
	double dampenVal = 0.3;
	Material material;
	void dampen(float fElapsedTime);
	bool pickedUp;
	CooldownTimer life = CooldownTimer(120);
};
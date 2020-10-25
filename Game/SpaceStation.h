#pragma once
#include "PhysicsObject.h"
#include "Updatable.h"
#include "CooldownTimer.h"
#include "MovingObject.h"

class SpaceStation : public Updatable, public Drawable {
public:
	SpaceStation(SpaceMinerGame* game, Vector3D pos);
	
	void update(SpaceMinerGame* game, float fElapsedTime);
	void draw(PixelEngine3D* g, Vector3D cameraPos, Rotor cameraDir, double FOV);
private:
	void createStructure();

	class Turret : public Updatable {
	public:
		Turret(PhysicsObject* body, PhysicsObject* gun, Vector3D upDir);
		void update(SpaceMinerGame* game, float fElapsedTime);

		PhysicsObject* body;
		PhysicsObject* gun;
		static double size;
	public: //TODO REMAKE PRIVATE
		
		uint16_t targetID;
		Vector3D prevTargetVel;
		Rotor getGunDir();
		void shoot(SpaceMinerGame* game);
		void aimAndShoot(SpaceMinerGame* game, float fElapsedTime);
		bool inRange(MovingObject* o);

		CooldownTimer fireRate = CooldownTimer(0.125);
		MovingObject* target;
		double maxRange = 20000;
		double maxPitch = 3.14159 * 0.8 / 2.0;
		double minPitch = -3.14159 / 6;
		double currPitch = 0;
		Vector3D upDir;
		double spread = 0.00;
		double pitchRate = 1.2;
		double bulletVel = 4500;
		double turnRate = 1.2;
	};

	Turret* turret1;
	Turret* turret2;
	PhysicsObject* mainStructure;

	std::vector<Vector3D> points; //temp
	std::vector<Polygon3D> polygons;
	
};
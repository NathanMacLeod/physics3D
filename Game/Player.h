#pragma once
#include "MovingObject.h"
#include "Updatable.h"
#include "CooldownTimer.h"
#include "Enemy.h"
#include "Ore.h"
#include "Asteroid.h"
//#include "SpaceMinerGame.h"

class Player : public MovingObject, public Updatable {
public:
	enum GunType { Default, Twin, Gatling };
	enum Tool { Guns, Missiles };

	struct UpgradeProfile {
		UpgradeProfile();
		GunType gun;
		bool hasMissiles;
		bool hasGunsight;
		int inventoryLevel;
		int thrustLevel;
		int radarLevel;
	};

	void applyUpgradeProfile(const UpgradeProfile& upgrades);
	int* getInventory();
	Player(Vector3D position, GunType gun);
	void setEquippedTool(Tool t);
	void update(SpaceMinerGame* game, float fElapsedTime);
	void shoot(SpaceMinerGame* game);
	void drawPlayerUI(SpaceMinerGame* g, double FOV);
	void getCameraPosOrient(Vector3D* posOut, Rotor* orientOut);
	void pickUpOre(SpaceMinerGame* game);
	void selectTarget(SpaceMinerGame* game);
	
private:
	static const int KM_CONST = 20000;
	
	double radarRange = 1;
	double radarSize = 0.7;
	std::vector<Vector3D> radarPoints;
	Vector3D radarPos;

	Enemy* target;
	uint16_t targetID;
	Vector3D targetPrevVel;
	double timeStep;
	Vector3D savedPrevVel;
	bool targetLocked;
	CooldownTimer targetLockTime = CooldownTimer(2);
	CooldownTimer missileRechargeTime = CooldownTimer(35);
	CooldownTimer missileFireRate = CooldownTimer(0.5);
	double missileLockAngle = 3.14159 / 8;
	double missileLockCos;

	Tool equippedTool;

	double grabDist;
	double maxGrabAng = cos(3.14159 / 8);
	double maxLockAng = cos(3.14159 * 60.0 / 180);
	double bulletVel = 4500;
	int inventory[Ore::N_TYPES];
	CooldownTimer shootTimer = CooldownTimer(0.25);
	GunType gun;
	int nMaxRockets;
	int nRockets;

	double getKMToP(Vector3D p);
	Asteroid* getTargetedAsteroid(SpaceMinerGame* game);
	Ore* getOreToGrab(SpaceMinerGame* game);
	void fireRocket(SpaceMinerGame* game);
	void fireBullet(SpaceMinerGame* game);
	void createRadarPoints();

	bool hasRockets;
	bool hasGunsight;
};
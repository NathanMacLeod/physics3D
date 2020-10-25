#include "Player.h"
#include "Bullet.h"
#include "Missile.h"
#include "SpaceMinerGame.h"

Player::Player(Vector3D position, GunType gun) {

	this->gun = gun;
	angularDampFactor = 3; //temporary
	linearDampFactor = 0.45;

	equippedTool = Guns;

	MovingObject::createShipMesh(true, 35, olc::VERY_DARK_RED, olc::BLACK,
		olc::VERY_DARK_MAGENTA, olc::BLACK, olc::DARK_GREEN, olc::VERY_DARK_GREEN, &body, &model);
	//createStructure();

	grabDist = body->getCollisionRadius() * 2;
	missileRechargeTime.reset();
	missileLockCos = cos(missileLockAngle);

	applyUpgradeProfile(UpgradeProfile());

	body->translate(position);
	createRadarPoints();

	findIVals();

}

Player::UpgradeProfile::UpgradeProfile() {
	gun = Default;
	hasMissiles = false;
	hasGunsight = false;
	thrustLevel = 1;
	radarLevel = 1;
}

void Player::applyUpgradeProfile(const UpgradeProfile& upgrades) {
	gun = upgrades.gun;
	hasRockets = upgrades.hasMissiles;
	hasGunsight = upgrades.hasGunsight;
	pitchRate = 6;
	yawRate = 6;
	rollRate = 3;
	forwardThrust = 850 + 300 * upgrades.thrustLevel;
	radarRange = 1 + .5 * upgrades.radarLevel;
	sideThrust = 650;
	verticleThrust = 650;
	nMaxRockets = 2;

	switch (gun) {
	case Default:
		shootTimer = CooldownTimer(0.25);
		break;
	case Twin:
		shootTimer = CooldownTimer(0.17);
		break;
	case Gatling:
		shootTimer = CooldownTimer(0.07);
		break;
	}
}

void Player::createRadarPoints() {
	int nRows = 10;
	int nCols = 18;
	radarPos = Vector3D(4, 2.5, 0);
	
	for (int i = 0; i < nCols; i++) {
		double theta = 2 * 3.14159 * i / nCols;
		for (int j = 0; j < nRows; j++) {
			double phi = 3.14159 * (j + 1) / (nRows + 1);
			radarPoints.push_back(radarPos.add(Vector3D(sin(phi) * cos(theta), cos(phi), sin(phi) * sin(theta)).multiply(radarSize)));
		}
	}
}

void Player::setEquippedTool(Tool t) {
	switch (t) {
	case Missiles:
		if (hasRockets) {
			if (equippedTool != t) {
				targetLocked = false;
			}
			equippedTool = t;
		}
		break;
	default:
		equippedTool = t;
	}
	
}

void Player::getCameraPosOrient(Vector3D* posOut, Rotor* orientOut) {
	*orientOut = Rotor(Vector3D(0, 1, 0), 3.14159 / 2.0).applyRotor(body->getOrientation());
	*posOut = orientOut->rotate(Vector3D(0, 0, 65)).add(body->getCenterOfMass());
}

void Player::selectTarget(SpaceMinerGame* game) {
	double maxCos = maxLockAng;
	Enemy* newTarget = nullptr;
	Vector3D dir = getDir();
	for (Enemy* e : *game->getEnemies()) {
		Vector3D toE = e->getPos().sub(getPos()).getUnitVector();
		double cosAng = toE.dotProduct(dir);
		if (cosAng > maxLockAng && (cosAng > maxCos || newTarget == target)) {
			newTarget = e;
			maxCos = cosAng;
		}
	}
	if (newTarget != nullptr && newTarget != target) {
		target = newTarget;
		targetID = target->getRigidBody()->getID();
		targetLocked = false;
		targetLockTime.reset();
	}
}

void Player::update(SpaceMinerGame* game, float fElapsedTime) {
	if (target == nullptr) {
		selectTarget(game);
	}
	else if (!game->getPhysicsEngine()->bodyInUse(targetID) || target->getPos().sub(getPos()).getUnitVector().dotProduct(getDir()) < maxLockAng) {
		target = nullptr;
	}
	else {
		timeStep = fElapsedTime;
		savedPrevVel = targetPrevVel;
		targetPrevVel = target->getRigidBody()->getVelocity();

		if (equippedTool == Missiles && !targetLocked) {
			double targetAngCos = target->getPos().sub(getPos()).getUnitVector().dotProduct(getDir());
			if (targetAngCos >= missileLockCos) {
				targetLockTime.updateTimer(fElapsedTime);
				if (targetLockTime.isReady()) {
					targetLocked = true;
					targetLockTime.reset();
				}
			}
			else {
				targetLocked = false;
				targetLockTime.reset();
			}
		}
	}

	if (nRockets < nMaxRockets) {
		missileRechargeTime.updateTimer(fElapsedTime);
		if (missileRechargeTime.isReady()) {
			nRockets++;
			missileRechargeTime.reset();
		}
	}
	
	missileFireRate.updateTimer(fElapsedTime);
	shootTimer.updateTimer(fElapsedTime);
	dampenMotion(fElapsedTime);
}

Ore* Player::getOreToGrab(SpaceMinerGame* game) {
	double closestAngle = 0;
	Ore* grabbedOre = nullptr;
	for (Ore* o : *game->getOre()) {
		Vector3D toOre = o->getPos().sub(getPos());
		double grabAngle = getDir().dotProduct(toOre.getUnitVector());
		if (toOre.getMagnitudeSquared() < grabDist * grabDist && grabAngle > closestAngle) {
			grabbedOre = o;
			closestAngle = grabAngle;
		}
	}
	return grabbedOre;
}

double Player::getKMToP(Vector3D p) {
	return p.sub(getPos()).getMagnitude() / KM_CONST;
}

Asteroid* Player::getTargetedAsteroid(SpaceMinerGame* game) {
	double closestAsteroid = 0.5 * KM_CONST; //max dist
	Asteroid* asteroid = nullptr;
	double rExpandFactor = 2;
	for (Asteroid* a : *game->getAsteroids()) {
		Vector3D toA = a->getPos().sub(getPos());
		double distSqrd = toA.getMagnitudeSquared();
		if (distSqrd < closestAsteroid * closestAsteroid) {
			double dist = sqrt(distSqrd);
			double ang = acos(getDir().dotProduct(toA.getUnitVector()));
			double r = a->getRigidBody()->getCollisionRadius();
			double reqAng = atan(r * rExpandFactor / dist);
			if (ang <= reqAng) {
				closestAsteroid = dist;
				asteroid = a;
			}
		}
	}
	return asteroid;
}

void Player::pickUpOre(SpaceMinerGame* game) {
	Ore* ore = getOreToGrab(game);
	if (ore != nullptr) {
		ore->pickUp();
		Ore::Material material = ore->getMaterial();
		inventory[(int)material]++;
	}
}

void Player::fireRocket(SpaceMinerGame* game) {
	if (nRockets > 0 && missileFireRate.isReady() && targetLocked) {
		nRockets--;
		double launchVel = 900;
		Vector3D launchDir = body->getOrientation().rotate(Vector3D(0, 1, 0)).multiply(launchVel);
		Missile* missile = new Missile(body->getCenterOfMass(), body->getOrientation(), body->getVelocity().add(launchDir), target, body->getID(), true);
		game->addProjectile(missile);
		missileFireRate.reset();
	}
}

void Player::fireBullet(SpaceMinerGame* game) {
	if (shootTimer.isReady()) {
		shootTimer.reset();

		Vector3D dir = body->getOrientation().rotate(Vector3D(1, 0, 0));
		double spawnDist = 150;

		Vector3D variation(0, 0, 0);
		Vector3D offset;
		double r;
		Vector3D spawnPos = body->getCenterOfMass().add(dir.multiply(spawnDist));

		static int twinDir = 1;
		switch (gun) {

		case Twin:
			r = 22;
			offset = body->getOrientation().rotate(Vector3D(0, 0, 1));
			offset = offset.multiply(r * twinDir);
			twinDir *= -1;
			spawnPos = spawnPos.add(offset);
			break;
		case Gatling:
			variation = Vector3D(0.5 - (float)rand() / RAND_MAX, 0.5 - (float)rand() / RAND_MAX, 0.5 - (float)rand() / RAND_MAX).getUnitVector().multiply(0.04);
			break;
		}


		Bullet* bullet = new Bullet(spawnPos, body->getOrientation(), body->getVelocity().add(dir.add(variation).multiply(bulletVel)), 700, body->getID(), olc::RED, olc::WHITE);
		game->addProjectile(bullet);
	}
}

void Player::shoot(SpaceMinerGame* game) {
	switch (equippedTool) {
	case Guns:
		fireBullet(game);
		break;
	case Missiles:
		fireRocket(game);
		break;
	}
}

void Player::drawPlayerUI(SpaceMinerGame* g, double FOV) {
	static char buff[64];

	olc::Pixel color = olc::CYAN;

	int centerX = g->ScreenWidth() / 2;
	int centerY = g->ScreenHeight() / 2;

	//draw crosshair
	if (equippedTool != Missiles) {
		double lineWidth = 1;
		double gapSize = 6;
		double size = 10;

		g->FillRect(centerX - (gapSize + size), centerY - lineWidth / 2, size, lineWidth, color);
		g->FillRect(centerX + gapSize, centerY - lineWidth / 2, size, lineWidth, color);
		g->FillRect(centerX - lineWidth / 2.0, centerY - (gapSize + size), lineWidth, size, color);
		g->FillRect(centerX - lineWidth / 2.0, centerY + (gapSize), lineWidth, size, color);
	}
	else {
		double lockOnRadius = FOV * sin(missileLockAngle);
		g->DrawCircle(centerX, centerY, lockOnRadius, color);
		
		if (nRockets == 0) {
			g->DrawString(centerX - 5 * 8, centerY, "NO MISSILE", color, 1);
		}
		else {
			sprintf_s(buff, "MISSILE: %d", nRockets);
			g->DrawString(centerX - 5 * 8, centerY + lockOnRadius + 4, buff, color, 1);
		}
	}
	Vector3D camPos;
	Rotor camOrient;
	getCameraPosOrient(&camPos, &camOrient);
	//highlight enemy target
	
	if (target != nullptr) {
		Vector3D toEnemy = target->getPos().sub(getPos());
		if (getDir().dotProduct(toEnemy) > 0) {

			double r1Min = 12;
			
			double r1 = target->getRigidBody()->getCollisionRadius() * 1.25 * FOV / getDir().dotProduct(toEnemy);
			r1 = std::max(r1Min, r1);
			Vector3D enemyScreenPos = g->getPixelCoord(target->getPos(), camPos, camOrient, FOV);
			double enemyDist = getKMToP(target->getPos());
			sprintf_s(buff, "%.2f KM", enemyDist);
			g->DrawString(enemyScreenPos.x - 3 * 8, enemyScreenPos.y + r1 + 4, buff, color, 1);

			if (equippedTool == Guns) {
				g->DrawCircle(enemyScreenPos.x, enemyScreenPos.y, r1, color);

				if (hasGunsight && enemyDist <= 1.5) {
					double r2 = 7;
					Vector3D leadPos = getPos().add(Projectile::calculateLeadPoint(getPos(), target->getPos(), body->getVelocity(), target->getRigidBody()->getVelocity(),
						bulletVel, true, 0, targetPrevVel, timeStep));
					Vector3D leadScreenPos = g->getPixelCoord(leadPos, camPos, camOrient, FOV);
					g->DrawCircle(leadScreenPos.x, leadScreenPos.y, r2, color);

					Vector3D r1ToR2 = leadScreenPos.sub(enemyScreenPos);
					double dist = r1ToR2.getMagnitude();
					if (dist > r2 + r1) {
						Vector3D p1 = enemyScreenPos.add(r1ToR2.multiply(r1 / dist));
						Vector3D p2 = enemyScreenPos.add(r1ToR2.multiply((dist - r2) / dist));
						g->DrawLine(p1.x, p1.y, p2.x, p2.y, color);
					}
				}
			}
			else if (equippedTool == Missiles) {
				olc::Pixel mColor = targetLocked ? olc::RED : color;

				g->DrawRect(enemyScreenPos.x - r1, enemyScreenPos.y - r1, r1 * 2, r1 * 2, mColor);
				double flickerRate = 2;
				double t = flickerRate * fmod(targetLockTime.getTime(), 1.0 / flickerRate);
				if (targetLocked || t > 0.5) {
					g->DrawLine(enemyScreenPos.x, enemyScreenPos.y - r1, enemyScreenPos.x + r1, enemyScreenPos.y, mColor);
					g->DrawLine(enemyScreenPos.x + r1, enemyScreenPos.y, enemyScreenPos.x, enemyScreenPos.y + r1, mColor);
					g->DrawLine(enemyScreenPos.x, enemyScreenPos.y + r1, enemyScreenPos.x - r1, enemyScreenPos.y, mColor);
					g->DrawLine(enemyScreenPos.x - r1, enemyScreenPos.y, enemyScreenPos.x, enemyScreenPos.y - r1, mColor);
				}
			}
			//accel lead
			/*r2 = 7;
			leadPos = getPos().add(Projectile::calculateLeadPoint(getPos(), target->getPos(), body->getVelocity(), target->getRigidBody()->getVelocity(), 
				bulletVel));
			leadScreenPos = g->getPixelCoord(leadPos, camPos, camOrient, FOV);
			g->DrawCircle(leadScreenPos.x, leadScreenPos.y, r2, olc::RED);*/
		}
	}

	Ore* highlightedOre = getOreToGrab(g);
	if (highlightedOre != nullptr) {
		olc::Pixel oreCol = olc::RED;
		double zDist = highlightedOre->getPos().sub(getPos()).dotProduct(getDir());
		double r = highlightedOre->getRigidBody()->getCollisionRadius() * 1.1 * FOV / zDist;
		Vector3D orePos = g->getPixelCoord(highlightedOre->getPos(), camPos, camOrient, FOV);
		g->DrawCircle(orePos.x, orePos.y, r, oreCol);

		Ore::Material material = highlightedOre->getMaterial();
		sprintf_s(buff, "%s\n\n$%d", Ore::getAbbrev(material), (int) Ore::getValue(material));
		g->DrawString(orePos.x  + r, orePos.y - r, buff, oreCol, 2);
	}

	Asteroid* asteroid = getTargetedAsteroid(g);
	if (asteroid != nullptr) {
		olc::Pixel astCol = olc::WHITE;
		double zDist = asteroid->getPos().sub(getPos()).dotProduct(getDir());
		double r = asteroid->getRigidBody()->getCollisionRadius() * 1.25 * FOV / zDist;
		Vector3D astPos = g->getPixelCoord(asteroid->getPos(), camPos, camOrient, FOV);

		g->DrawLine(astPos.x - r, astPos.y - r, astPos.x - r + r / 8, astPos.y - r, astCol);
		g->DrawLine(astPos.x - r, astPos.y - r, astPos.x - r, astPos.y - r + r/8, astCol);
		g->DrawLine(astPos.x + r, astPos.y - r, astPos.x + r - r / 8, astPos.y - r, astCol);
		g->DrawLine(astPos.x + r, astPos.y - r, astPos.x + r, astPos.y - r + r / 8, astCol);
		g->DrawLine(astPos.x + r, astPos.y + r, astPos.x + r - r / 8, astPos.y + r, astCol);
		g->DrawLine(astPos.x + r, astPos.y + r, astPos.x + r, astPos.y + r - r / 8, astCol);
		g->DrawLine(astPos.x - r, astPos.y + r, astPos.x - r + r / 8, astPos.y + r, astCol);
		g->DrawLine(astPos.x - r, astPos.y + r, astPos.x - r, astPos.y + r - r / 8, astCol);

		double dist = getKMToP(asteroid->getPos());
		sprintf_s(buff, "%.2f KM", dist);
		g->DrawString(astPos.x - 3 * 8, astPos.y - r - 10, buff, astCol, 1);
		sprintf_s(buff, "%d / %d", (int) asteroid->getHp(), (int) asteroid->getMaxHp());
		g->DrawString(astPos.x - strlen(buff) * 8 / 2, astPos.y + r + 2, buff, astCol, 1);
	}

	Rotor rDir(Vector3D(0, 1, 0), 3.14159 / 2.0);
	Rotor inv = body->getOrientation().getInverse();

	double width = radarSize * 0.12 / radarRange;
	double length = radarSize * 0.2 / radarRange;
	std::vector<Vector3D> radarTri;
	radarTri.push_back(Vector3D(-length / 2.0, 0, width / 2.0));
	radarTri.push_back(Vector3D(-length / 2.0, 0, -width / 2.0));
	radarTri.push_back(Vector3D(length / 2.0, 0, 0));
	radarTri.push_back(Vector3D(length, 0, 0));

	std::vector<Vector3D> points = radarTri;
	for (int i = 0; i < points.size(); i++) {
		points[i] = radarPos.add(points[i]);
	}
	Polygon3D p1 = Polygon3D(points.at(0), points.at(1), points.at(2), olc::BLUE, olc::DARK_BLUE);
	g->drawPolygon(p1, Vector3D(), rDir, FOV, true);
	g->draw3DLine(radarPos, points.at(3), Vector3D(), rDir, FOV, olc::BLUE);

	for (Vector3D p : radarPoints) {
		g->draw3DPoint(inv.rotate(p.sub(radarPos)).add(radarPos), Vector3D(), rDir, FOV, color);
	}
	for (Enemy* e : *g->getEnemies()) {
		Vector3D toE = e->getPos().sub(getPos());
		double dist = toE.getMagnitude();
		if (dist / KM_CONST <= radarRange) {
			
			Vector3D eRadarPos = radarPos.add(inv.rotate(toE.multiply(radarSize / (KM_CONST * radarRange))));
			Rotor enemyRelOrient = e->getRigidBody()->getOrientation().applyRotor(inv);
			std::vector<Vector3D> points = radarTri;

			for (int i = 0; i < points.size(); i++) {
				points[i] = eRadarPos.add(enemyRelOrient.rotate(points[i]));
			}
			Polygon3D p1 = Polygon3D(points.at(0), points.at(1), points.at(2), olc::RED, olc::DARK_RED);
			Polygon3D p2 = Polygon3D(points.at(2), points.at(1), points.at(0), olc::RED, olc::DARK_RED);
			
			g->drawPolygon(p1, Vector3D(), rDir, FOV, true);
			g->drawPolygon(p2, Vector3D(), rDir, FOV, true);
			g->draw3DLine(eRadarPos, points.at(3), Vector3D(), rDir, FOV, olc::RED);
		}
	}
}

int* Player::getInventory() {
	return inventory;
}
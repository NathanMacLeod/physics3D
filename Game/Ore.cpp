#include "Ore.h"
#include "SpaceMinerGame.h"
#include "Asteroid.h"


Ore::Ore(Vector3D pos, Material type, MovingObject::DebrisType debrisType) {
	double density = 1;
	double roughness = 0.2;
	double detail = 2;
	double size = 70;
	material = type;

	olc::Pixel lineColor;
	olc::Pixel color;

	switch (type) {
	case Iron:
		density = 1;
		lineColor = olc::VERY_DARK_RED;
		color = olc::VERY_DARK_GREY;
		break;
	case Copper:
		density = 1;
		lineColor = olc::VERY_DARK_RED;
		color = olc::Pixel(122, 45, 20);
		break;
	case Silver:
		density = 1;
		lineColor = olc::WHITE;
		color = olc::GREY;
		break;
	case Gold:
		density = 3;
		lineColor = olc::VERY_DARK_YELLOW;
		color = olc::YELLOW;
		break;
	case Plutonium:
		density = 1;
		lineColor = olc::GREEN;
		color = olc::DARK_GREY;
		break;
	case Cybernium:
		size = 100;
		roughness = 0.1;
		detail = 1;
		density = 1;
		lineColor = olc::CYAN;
		color = olc::DARK_CYAN;
		break;
	case Anthium:
		detail = 3;
		roughness = 0.4;
		density = 1;
		lineColor = olc::MAGENTA;
		color = olc::DARK_MAGENTA;
		break;
	}
	if (type != NormalDebris && type != EliteDebris) {
		Asteroid::createRockMesh(pos, size, detail, density, roughness, lineColor, color, &body, &model);
	}
	else {
		MovingObject::getShipDebris(debrisType, 35, olc::VERY_DARK_MAGENTA, olc::BLACK,
			olc::VERY_DARK_MAGENTA, olc::BLACK, olc::DARK_GREEN, olc::VERY_DARK_GREEN, &body, &model);
		body->translate(pos);
	}

	life.reset();
	pickedUp = false();
}

Ore::Material Ore::pickTypeRandomly() {
	static float probVals[N_TYPES] =
	{
		35, //iron
		18, //copper
		4, //gold
		10, //silver
		2, //plutonium
		0.6, //cybernium
		0.35, //anthium
		0, //normal debris
		0, //elite debris
	};

	static float total = -1;
	if (total == -1) {
		total = 0;
		for (int i = 0; i < N_TYPES; i++) {
			total += probVals[i];
		}
	}

	int chosenVal = 1 + (fmod(rand(), total));
	for (int i = 0; i < N_TYPES; i++) {
		chosenVal -= probVals[i];
		if (chosenVal <= 0) {
			return (Ore::Material) i;
		}
	}
	
	//shouldnt reach here
	return Iron;
}

Ore::Material Ore::getMaterial() {
	return material;
}

olc::Pixel Ore::getColor(Material m) {
	switch (m) {
	case Iron:
		return olc::VERY_DARK_GREY;
	case Copper:
		return olc::Pixel(122, 45, 20);
	case Silver:
		return olc::GREY;
	case Gold:
		return olc::YELLOW;
	case Plutonium:
		return olc::GREEN;
	case Cybernium:
		return olc::DARK_CYAN;
	case Anthium:
		return olc::DARK_MAGENTA;
	case NormalDebris:
		return olc::VERY_DARK_MAGENTA;
	}
}

std::string Ore::getName(Material m) {
	switch (m) {
	case Iron:
		return std::string("Iron");
	case Copper:
		return std::string("Copper");
	case Silver:
		return std::string("Silver");
	case Gold:
		return std::string("Gold");
	case Plutonium:
		return std::string("Plutonium");
	case Cybernium:
		return std::string("Cybernium");
	case Anthium:
		return std::string("Anthium");
	case NormalDebris:
		return std::string("Debris");
	}
}

std::string Ore::getAbbrev(Material m) {
	switch (m) {
	case Iron:
		return std::string("Fe");
	case Copper:
		return std::string("Cu");
	case Silver:
		return std::string("Ag");
	case Gold:
		return std::string("Au");
	case Plutonium:
		return std::string("Pu");
	case Cybernium:
		return std::string("Cb");
	case Anthium:
		return std::string("An");
	case NormalDebris:
		return std::string("Debris");
	}
}

double Ore::getValue(Material material) {
	switch (material) {
	case Iron:
		return 100;
	case Copper:
		return 200;
	case Silver:
		return 1000;
	case Gold:
		return 2500;
	case Plutonium:
		return 5000;
	case Cybernium:
		return 15000;
	case Anthium:
		return 50000;
	case NormalDebris:
		return 3500;
	}
}

void Ore::performDeathActions(SpaceMinerGame* game) {
	game->getPhysicsEngine()->removeRigidBody(body);
}

void Ore::dampen(float fElapsedTime) {
	float t = std::fmin(1, fElapsedTime);
	Vector3D vel = body->getVelocity();
	body->setVelocity(vel.sub(vel.multiply(t * dampenVal)));
}

void Ore::update(SpaceMinerGame* game, float fElapsedTime) {
	dampen(fElapsedTime);
	life.updateTimer(fElapsedTime);
	if (material == NormalDebris || material == EliteDebris) {
		float lifeF = std::fmaxf(1, (life.getTimerTime() - life.getTime()) / 2.0);
		Vector3D originVar = Vector3D((float)rand() / RAND_MAX, (float)rand() / RAND_MAX, (float)rand() / RAND_MAX).multiply(0.75 * 35);
		Particle::generateExplosion(game, getPos().add(originVar), 0.7, 50, 3 / lifeF, olc::RED);
		Particle::generateExplosion(game, getPos().add(originVar), 0.9, 100, 1 / lifeF, olc::GREEN);
		Particle::generateExplosion(game, getPos().add(originVar), 4.5, 250, 3 / lifeF, olc::DARK_GREY);
	}
}

void Ore::pickUp() {
	pickedUp = true;
}

bool Ore::isExpired() {
	return pickedUp || life.isReady();
}

//void Ore::draw(PixelEngine3D* g, Vector3D cameraPos, Rotor cameraDir, double FOV) {
//	debugDraw(g, cameraPos, cameraDir, FOV);
//}
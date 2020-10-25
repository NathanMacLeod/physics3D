#include "Particle.h"
#include "SpaceMinerGame.h"

Particle::Particle(float life, Vector3D vel, Vector3D pos, olc::Pixel color) {
	this->life = CooldownTimer(life);
	this->vel = vel;
	this->pos = pos;
	this->color = color;

	this->life.reset();
}

void Particle::update(SpaceMinerGame* game, float fElapsedTime) {
	life.updateTimer(fElapsedTime);
	pos = pos.add(vel.multiply(fElapsedTime));
}

void Particle::draw(PixelEngine3D* g, Vector3D cameraPos, Rotor cameraDir, double FOV) {
	g->draw3DPoint(pos, cameraPos, cameraDir, FOV, color);
}

bool Particle::isExpired() {
	return life.isReady();
}

void Particle::generateExplosion(SpaceMinerGame* game, Vector3D pos, double time, double size, int nParticles, olc::Pixel color) {
	for (int i = 0; i < nParticles; i++) {
		double phi = 3.14159 * ((float) rand() / (float) RAND_MAX);
		double theta = 2 * 3.14159 * ((float)rand() / (float) RAND_MAX);

		Vector3D dir(sin(phi) * cos(theta), sin(phi) * sin(theta), cos(phi));

		double vel = (size / time) * ((float)rand() / (float) RAND_MAX);


		Particle* particle = new Particle(time, dir.multiply(vel), pos, color);
		game->addParticle(particle);
	}
}
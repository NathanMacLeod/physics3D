#include "Damageable.h"

Damageable::~Damageable() {

}

void Damageable::damage(double damage) {
	hp -= damage;
}

bool Damageable::isExpired() {
	return hp <= 0;
}
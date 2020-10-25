#pragma once
#include "Expireable.h"

class Damageable : public Expireable {
protected:
	double hp;
	double maxHp;
public:
	virtual ~Damageable();

	virtual void damage(double damage);
	double getHp() {
		return hp;
	}
	double getMaxHp() {
		return maxHp;
	}

	virtual bool isExpired();
};
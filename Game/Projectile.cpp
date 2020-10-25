#include "Projectile.h"
#include <stdio.h>

Projectile::~Projectile() {

}

Vector3D Projectile::calculateLeadPoint(Vector3D shooterPos, Vector3D targetPos, Vector3D shooterVel, Vector3D targetVel, double bulletVel, bool guessAccel, double distOffset, Vector3D targetPrevVel, float fElapsedTime) {
	Vector3D toTarget = targetPos.sub(shooterPos);
	Vector3D toTargetUnit = toTarget.getUnitVector();

	Vector3D relVel = targetVel.sub(shooterVel);
	double distVel = relVel.dotProduct(toTargetUnit);
	Vector3D accel = (guessAccel) ? targetVel.sub(targetPrevVel).multiply(1.0 / fElapsedTime) : Vector3D(0, 0, 0);
	double distAccel = accel.dotProduct(toTargetUnit);

	double dist = toTarget.getMagnitude() - distOffset;
	double vDif = bulletVel - distVel;
	double radical = vDif * vDif - 2 * distAccel * dist;
	
	double t;

	if (distAccel == 0) {
		t = dist / vDif;
	}
	else {
		if (radical < 0) {
			t = 0;
		}
		else {
			double t1 = (vDif + sqrt(radical)) / (distAccel);
			double t2 = (vDif - sqrt(radical)) / (distAccel);

			if (t1 <= 0) {
				t = t2;
			}
			else if (t2 <= 0) {
				t = t1;
			}
			else if (t1 > 0 && t2 > 0) {
				t = std::fmin(t1, t2);
			}
			else {
				t = 0;
			}
		}
		
	}

	return toTarget.add(relVel.multiply(t).add(accel.multiply(t * t /2.0)));
}
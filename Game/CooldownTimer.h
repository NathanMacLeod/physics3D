#pragma once

class CooldownTimer {
private:
	float currTime;
	float totalTime;
public:

	CooldownTimer(float time) {
		currTime = 0;
		totalTime = time;
	}

	float getTimerTime() {
		return totalTime;
	}

	float getTime() {
		return currTime;
	}

	void updateTimer(float fElapsedTime) {
		currTime -= fElapsedTime;
	}

	void setCustum(float t) {
		currTime = t;
	}

	bool isReady() {
		return currTime <= 0;
	}

	void reset() {
		currTime = totalTime;
	}
};
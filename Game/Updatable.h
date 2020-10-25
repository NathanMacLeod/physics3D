#pragma once

class SpaceMinerGame;

class Updatable {
public:
	virtual void update(SpaceMinerGame* game, float fElapsedTime) = 0;
};
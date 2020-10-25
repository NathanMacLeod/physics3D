#pragma once
class SpaceMinerGame;

class Expireable {
public:
	virtual bool isExpired() = 0;
	virtual void performDeathActions(SpaceMinerGame* game) {}
};
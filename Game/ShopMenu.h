#pragma once
#include "Drawable.h"
#include "Ore.h"

class ShopMenu {
public:
	ShopMenu();
	void draw(PixelEngine3D* g);
private:
	struct ShopItem {
		ShopItem(const std::string& title, const std::string& description, int cost, int oreReq[Ore::N_TYPES]);
		void draw(PixelEngine3D* g, int x, int y, int width, int height);

		std::string title;
		std::string description;
		int cost;
		int oreReq[Ore::N_TYPES];
	};

	std::vector<struct ShopItem> shopItems;
};
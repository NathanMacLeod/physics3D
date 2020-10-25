#include "ShopMenu.h"

ShopMenu::ShopMenu() {
	int req[Ore::N_TYPES];
	for (int i = 0; i < Ore::N_TYPES; i++) {
		req[i] = 0;
	}
	req[Ore::Iron] = 15;
	req[Ore::Plutonium] = 15;
	shopItems.push_back(ShopItem(std::string("Guided Missile"), std::string("The missile knows where it is\nbecause it knows where it isn't\nWhere it now is, it wasn't"), 42069, req));
}

ShopMenu::ShopItem::ShopItem(const std::string& title, const std::string& description, int cost, int oreReq[Ore::N_TYPES]) {
	this->title = title;
	this->description = description;
	this->cost = cost;
	for (int i = 0; i < Ore::N_TYPES; i++) {
		this->oreReq[i] = oreReq[i];
	}
}

void ShopMenu::ShopItem::draw(PixelEngine3D* g, int x, int y, int width, int height) {
	static char buff[64];

	g->DrawRect(x, y,width,  height, olc::CYAN);
	g->DrawString(x + 0.05 * width, y + 4, title, olc::WHITE, 2);
	g->DrawString(x + 0.05 * width, y + 22, description, olc::WHITE, 1);
	sprintf_s(buff, "$%d ", cost);
	g->DrawString(x + 0.05 * width, y + 50, buff, olc::WHITE, 1);

	double xPos = x + 0.05 * width + 8 * strlen(buff);
	for (int i = 0; i < Ore::N_TYPES; i++) {
		if (oreReq[i] > 0) {
			Ore::Material m = (Ore::Material) i;
			sprintf_s(buff, "%s: %d ", Ore::getName(m).c_str(), oreReq[i]);
			g->DrawString(xPos, y + 50, buff, Ore::getColor(m), 1);
			xPos += 8 * strlen(buff);
		}
	}
}

void ShopMenu::draw(PixelEngine3D* g) {
	int height = 0.8 * g->ScreenHeight();
	int width = g->ScreenWidth() - 0.2 * g->ScreenHeight();

	g->FillRect((g->ScreenWidth() - width) / 2, (g->ScreenHeight() - height) / 2, width, height, olc::BLACK);
	g->DrawRect((g->ScreenWidth() - width) / 2, (g->ScreenHeight() - height) / 2, width, height, olc::CYAN);

	int margain = 0.1 * width;
	for (int i = 0; i < shopItems.size(); i++) {
		ShopItem s = shopItems.at(i);
		s.draw(g, margain, margain + 0.15 * height * i, 0.4 * width, 0.15 * height);
	}
}
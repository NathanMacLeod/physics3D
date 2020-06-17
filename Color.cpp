#include "Color.h"
#include <stdio.h>

Color getColor(int32_t r, int32_t g, int32_t b) {
	Color color = 0;
	color += (r << 16);
	color += (g << 8);
	color += b;
	return color;
}

const Color black = getColor(0, 0, 0);
const Color red = getColor(255, 0, 0);
const Color green = getColor(0, 255, 0);
const Color blue = getColor(0, 0, 255);
const Color gray = getColor(50, 50, 50);
const Color light_gray = getColor(150, 150, 150);
const Color orange = getColor(255, 0, 255);
const Color white = getColor(255, 255, 255);

int getR(Color color) {
	return ((color & 0x00FF0000) >> 16);
}

int getG(Color color) {
	return ((color & 0x0000FF00) >> 8);
}

int getB(Color color) {
	return (color & 0x000000FF);
}

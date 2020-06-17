#pragma once
#include <inttypes.h>

#ifndef COLOR_H
#define COLOR_H

typedef int32_t Color;

Color getColor(int32_t r, int32_t g, int32_t b);

int getR(Color color);

int getG(Color color);

int getB(Color color);

const extern Color black;
const extern Color red;
const extern  Color green;
const extern  Color blue;
const extern  Color gray;
const extern  Color light_gray;
const extern  Color orange;
const extern  Color white;

#endif
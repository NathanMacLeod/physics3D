#include "ConsoleEngine.h"
#include <stdio.h>
#include <chrono>

//choose which function to draw by commenting out
float getRadius(float theta) {
	float returnVal;
	//returnVal = 10 + sin(2 * 3.14 * theta); //bracelet
	//returnVal = 10 * cos(.95 * theta);
	//returnVal = (theta/10) +  ((theta / 10)) * sin(2 * 3.14 * theta)/10; //spiral rose pattern
	returnVal = 1 + (theta / 10) + sin(2 * 3.14 * theta); //spiral flower 2
	return 10 * returnVal;
}

void draw(float dTheta, float& r, float& theta, int* p, ConsoleEngine* engine) {
	float dx = r * cos(theta);
	float dy = r * sin(theta);
	theta += dTheta;
	r = getRadius(theta);
	float dx1 = r * cos(theta);
	float dy1 = r * sin(theta);
	engine->drawLine(p[1] + dx, p[1] + dy, p[0] + dx1, p[1] + dy1, engine->PICZEL, white);
	//engine->drawTriangle(p[0], p[1], p[1] + dx, p[1] + dy, p[0] + dx1, p[1] + dy1, engine->PICZEL, red);
	//engine.drawLine(p[0] - dy, p[1] + dx, p[0] - dy1, p[1] + dx1, engine.PICZEL, white);
	//engine.drawLine(p[0] + dy, p[1] - dx, p[0] + dy1, p[1] - dx1, engine.PICZEL, white);
	engine->drawLine(p[0] - dx, p[1] - dy, p[0] - dx1, p[1] - dy1, engine->PICZEL, red);
	//engine.drawPiczel(x0, y0, engine.PICZEL, red);
	//engine.drawPiczel(x1, y1, engine.PICZEL, green);
	engine->outputScreen();
}

void drawPinwheel(float dTheta, float r, float& theta, int* p, ConsoleEngine* engine) {
	theta += dTheta;
	int numSegments = 55;
	int color = 0;
	for (float i = 0; i < 2 * 3.14159; i += 2 * 3.14159 / numSegments) {

		float x1 = p[0] + cos(theta + i) * r;
		float y1 = p[1] + sin(theta + i) * r;

		float x2 = p[0] + cos((double)theta + i + 2.0f * 3.14159f / numSegments) * r;
		float y2 = p[1] + sin((double)theta + i + 2.0f * 3.14159f / numSegments) * r;

		Color colorOfTriangle;
		switch (color) {
		case 0:
			colorOfTriangle = red;
			break;
		case 1:
			colorOfTriangle = orange;
			break;
		case 2:
			colorOfTriangle = yellow;
			break;
		case 3:
			colorOfTriangle = green;
			break;
		case 4:
			colorOfTriangle = blue;
			break;
		default:
			colorOfTriangle = red;
			color = 0;
		}
		color++;

		engine->fillTriangle(p[0], p[1], x1, y1, x2, y2, engine->PICZEL, colorOfTriangle);
		
	}
	engine->outputScreen();
}

int main() {
	ConsoleEngine engine(180, 180, 3);
	wchar_t b = 0x2588;
	wchar_t a = engine.PICZEL;
	engine.fillRect(0, 0, 180, 180, engine.PICZEL, black);
	//engine.fillRect(10, 20, 20, 20, engine.PICZEL, green);
	//engine.fillRect(0, 0, 30, 60, engine.PICZEL, white);
	std::chrono::system_clock::time_point t1 = std::chrono::system_clock::now();
	std::chrono::system_clock::time_point t2;
	//engine.drawLine(20, 20, 90, 80, engine.PICZEL, black);
	float queuedTime = 0;

	float theta = 0;
	float r = 0;
	float totalT = 0;
	int p[2];
	p[0] = 90;
	p[1] = 90;

	r = getRadius(theta);

	while (true) {
		t2 = std::chrono::system_clock::now();
		std::chrono::duration<float> timePassed = t2 - t1;
		t1 = t2;
		queuedTime += timePassed.count();
		totalT += timePassed.count();

		if (1) {
			engine.fillRect(0, 0, 180, 180, engine.PICZEL, white);
			float dTheta = timePassed.count();
			drawPinwheel(dTheta, 70, theta, p, &engine);
			//draw(dTheta, r, theta, p, &engine);
		}
		
	}
	return 1;
}
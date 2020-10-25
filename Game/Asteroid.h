#pragma once
#include "PhysicsObject.h"
#include "Damageable.h"

class SpaceMinerGame;

class Asteroid : public PhysicsObject, public Damageable {
private:
	static void generateFace(Vector3D p1, Vector3D p2, Vector3D p3, Vector3D p4, int detail, std::vector<Vector3D*>* face_out, std::vector<Vector3D*>* allP_out);
	static void generatePolygons(std::vector<Vector3D*>* face, std::vector<Polygon3D>* polygons, Vector3D center, int detail, olc::Pixel lineColor, olc::Pixel color);
	double size;
public:
	~Asteroid() {};
	Asteroid(Vector3D position, double size, int detail);
	static void createRockMesh(Vector3D pos, double size, double detail, double density, double roughness, olc::Pixel lineColor, olc::Pixel color, RigidBody** bodyOut, PolyModel** meshOut);
	static double pickAsteroidSize();
	void performDeathActions(SpaceMinerGame* game);

	static const int MIN_SIZE = 250;
	static const int MAX_SIZE = 1450;
};
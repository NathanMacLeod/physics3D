#include "PhysicsObject.h"

PhysicsObject::PhysicsObject(PolyModel* model, RigidBody* body) {
	this->model = model;
	this->body = body;
}

PhysicsObject::PhysicsObject(const PhysicsObject& object) {
	this->model = new PolyModel(*object.model);
	this->body = new RigidBody(*object.body);
}

PhysicsObject::PhysicsObject() {
	model = nullptr;
	body = nullptr;
}

PhysicsObject::~PhysicsObject() {
	delete model;
	delete body;
}

void PhysicsObject::debugDraw(PixelEngine3D* g, Vector3D cameraPos, Rotor cameraDir, double FOV) {
	for (ConvexHull* h : *body->getHulls()) {

		g->draw3DPoint(h->getCenterOfMass(), cameraPos, cameraDir, FOV, olc::RED, true);
		for (RigidSurface* s : *h->getSurfaces()) {

			Vector3D avg(0, 0, 0);

			for (int i = 0; i < s->getPoints()->size(); i++) {
				Vector3D* p1 = s->getPoints()->at(i);
				int j = (i == s->getPoints()->size() - 1) ? 0 : i + 1;
				Vector3D* p2 = s->getPoints()->at(j);
				g->draw3DLine(*p1, *p2, cameraPos, cameraDir, FOV, olc::GREEN);
				avg = avg.add(*p1);
			}

			avg = avg.multiply(1.0 / s->getPoints()->size());

			g->draw3DLine(avg, avg.add(s->getUnitNorm().multiply(150)), cameraPos, cameraDir, FOV, (s->isInteriorSurface()) ? olc::CYAN : olc::MAGENTA);
		}
	}
}

void PhysicsObject::draw(PixelEngine3D* g, Vector3D cameraPos, Rotor cameraDir, double FOV) {
	if (model->getOrientation() != body->getOrientation()
		|| model->getPos() != body->getCenterOfMass()) {

		model->setPosAndOrientation(body->getOrientation(), body->getCenterOfMass());
	}
	model->draw(g, cameraPos, cameraDir, FOV);
}
 
Vector3D PhysicsObject::getDir() {
	return body->getOrientation().rotate(Vector3D(1, 0, 0));
}

RigidBody* PhysicsObject::getRigidBody() {
	return body;
}

PolyModel* PhysicsObject::getModel() {
	return model;
}

Vector3D PhysicsObject::getPos() {
	return body->getCenterOfMass();
}

void PhysicsObject::createSurfacesFromPolygons(std::vector<Polygon3D>* polygons, std::vector<RigidSurface*>* out) {
	std::vector<RigidSurface*>* surfaces = new std::vector<RigidSurface*>();
	for (Polygon3D polygon : *polygons) {
		Vector3D v1 = polygon.p2.sub(polygon.p1);
		Vector3D v2 = polygon.p3.sub(polygon.p1);
		Vector3D normalVector = (v1.crossProduct(v2)).getUnitVector();
		std::vector<Vector3D> points;
		points.push_back(polygon.p1);
		points.push_back(polygon.p2);
		points.push_back(polygon.p3);
		RigidSurface* surface = new RigidSurface(&points, normalVector);
		out->push_back(surface);
	}

}
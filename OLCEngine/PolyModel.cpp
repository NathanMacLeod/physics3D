#include "PolyModel.h"

PolyModel::PolyModel(std::vector<Polygon3D>* polygons, Vector3D center) {
	radius = 0;
	for (Polygon3D p : *polygons) {
		this->worldPolygons.push_back(p);

		Vector3D p1 = p.p1.sub(center);
		Vector3D p2 = p.p2.sub(center);
		Vector3D p3 = p.p3.sub(center);

		if (p1.getMagnitude() > radius) {
			radius = p1.getMagnitude();
		}
		if (p2.getMagnitude() > radius) {
			radius = p2.getMagnitude();
		}
		if (p3.getMagnitude() > radius) {
			radius = p3.getMagnitude();
		}

		this->polygonsOG.push_back(Polygon3D(p1, p2, p3, p.lineColor, p.color));
	}
	this->position = center;
}

PolyModel::PolyModel() {

}

void PolyModel::rotate(Rotor rotation) {
	setOrientation(orientation.applyRotor(rotation));
}

void PolyModel::setOrientation(Rotor orientation) {
	this->orientation = orientation;
	for (int i = 0; i < polygonsOG.size(); i++) {
		worldPolygons.at(i).p1 = position.add(orientation.rotate(polygonsOG.at(i).p1));
		worldPolygons.at(i).p2 = position.add(orientation.rotate(polygonsOG.at(i).p2));
		worldPolygons.at(i).p3 = position.add(orientation.rotate(polygonsOG.at(i).p3));
	}
}

void PolyModel::translate(Vector3D translation) {
	position = position.add(translation);
	for (Polygon3D p : worldPolygons) {
		p.p1.add(translation);
		p.p2.add(translation);
		p.p3.add(translation);
	}
}

void PolyModel::setPosition(Vector3D pos) {
	translate(pos.sub(this->position));
}

void PolyModel::setPosAndOrientation(Rotor orientation, Vector3D pos) {
	this->position = pos;
	setOrientation(orientation);
}

Vector3D PolyModel::getPos() {
	return position;
}

Rotor PolyModel::getOrientation() {
	return orientation;
}

std::vector<Polygon3D>* PolyModel::getPolygons() {
	return &worldPolygons;
}

//camera dir should be unit length
bool PolyModel::outOfView(Vector3D cameraPos, Rotor cameraDir, double fov, int screenWidth, int screenHeight) {
	Vector3D relPos = cameraDir.getInverse().rotate(position.sub(cameraPos));
	double z = relPos.z + radius;
	if (z <= 0) {
		return true;
	}

	double maxX = (relPos.x + radius) * fov / z;
	double minX = (relPos.x - radius) * fov / z;
	double maxY = (relPos.y + radius) * fov / z;
	double minY = (relPos.y - radius) * fov / z;

	if (maxX < -screenWidth/2 || minX > screenWidth/2 || maxY < -screenHeight/2 || minY > screenHeight/2) {
		return true;
	}
	return false;
}
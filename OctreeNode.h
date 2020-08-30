#pragma once
#include <vector>;
#include "RigidBody.h"

class OctreeNode {
private:
	static const int N_CHILD = 8;

	Point3D pos;
	double size;
	double minSize;
	bool leafNode;
	std::vector<RigidBody*> bodies;
	OctreeNode* children[N_CHILD];

public:
	~OctreeNode();
	OctreeNode();
	OctreeNode(Point3D pos, double size, double minSize);
	double getSize();
	Point3D getPos();
	void expandNode();
	void addBody(RigidBody* b);
	bool isLeafNode();
	std::vector<RigidBody*>* getBodies();
	static void getCollisionLeafs(OctreeNode* curr, std::vector<OctreeNode*>* out);
	static void getAllNodes(OctreeNode* curr, std::vector<OctreeNode*>* out); //for debug
};
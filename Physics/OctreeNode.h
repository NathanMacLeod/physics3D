#pragma once
#include <vector>;
#include "RigidBody.h"

class OctreeNode {
private:
	static const int N_CHILD = 8;

	Vector3D pos;
	double size;
	double minSize;
	bool leafNode;
	std::vector<RigidBody*> bodies;
	OctreeNode* children[N_CHILD];

	static void getCollisionLeafs(OctreeNode* curr, std::vector<OctreeNode*>* out);
	static void getAllNodes(OctreeNode* curr, std::vector<OctreeNode*>* out); //for debug
public:
	~OctreeNode();
	OctreeNode();
	OctreeNode(Vector3D pos, double size, double minSize);
	double getSize();
	Vector3D getPos();
	void expandNode();
	void addBody(RigidBody* b);
	bool isLeafNode();
	std::vector<RigidBody*>* getBodies();
	void getCollisionLeafs(std::vector<OctreeNode*>* out);
	void getAllNodes(std::vector<OctreeNode*>* out);
};
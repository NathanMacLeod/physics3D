#include "OctreeNode.h";

OctreeNode::OctreeNode(Point3D pos, double size, double minSize) {
	this->size = size;
	this->minSize = minSize;
	this->pos = pos;
}

OctreeNode::OctreeNode() {
	
}

OctreeNode::~OctreeNode() {
	if (leafNode) {
		return;
	}

	for (int i = 0; i < N_CHILD; i++) {
		if (children[i] != nullptr) {
			delete children[i];
		}
	}
}

void OctreeNode::addBody(RigidBody* b) {
	bodies.push_back(b);
}

std::vector<RigidBody*>* OctreeNode::getBodies() {
	return &bodies;
}

double OctreeNode::getSize() {
	return size;
}

Point3D OctreeNode::getPos() {
	return pos;
}

void OctreeNode::getAllNodes(OctreeNode* curr, std::vector<OctreeNode*>* out) {
	out->push_back(curr);
	if (curr->isLeafNode() && curr->bodies.size() >= 2) {
		return;
	}
	else if (!curr->isLeafNode()) {
		for (int i = 0; i < N_CHILD; i++) {
			if (curr->children[i] != nullptr) {
				getAllNodes(curr->children[i], out);
			}
		}
	}
}

void OctreeNode::getCollisionLeafs(OctreeNode* curr, std::vector<OctreeNode*>* out) {
	if (curr->isLeafNode() && curr->bodies.size() >= 2) {
		out->push_back(curr);
		return;
	}
	else if (!curr->isLeafNode()) {
		for (int i = 0; i < N_CHILD; i++) {
			if (curr->children[i] != nullptr) {
				getCollisionLeafs(curr->children[i], out);
			}
		}
	}
}

bool OctreeNode::isLeafNode() {
	return leafNode;
}

void OctreeNode::expandNode() {
	for (int i = 0; i < N_CHILD; i++) {
		children[i] = nullptr;
	}

	if (size <= minSize || bodies.size() <= 1) {
		leafNode = true;
		return;//no more children, end tree
	}

	for (RigidBody* b : bodies) {
		double r = b->getCollisionRadius();
		Point3D bPos = b->getCenterOfMass();

		//lack of else statements intentional,
		//as bodies can be in multiple nodes at once
		//e.g. sitting on edge

		//posX
		if (bPos.x + r > pos.x + size / 2.0) {
			//posZ
			if (bPos.z + r > pos.z + size / 2.0) {
				//posY
				if (bPos.y + r > pos.y + size / 2.0) {
					if (children[0] == nullptr) {
						children[0] = new OctreeNode(Point3D(pos, size / 2.0, size / 2.0, size / 2.0), size / 2.0, minSize);
					}
					children[0]->addBody(b);
				}
				//negY
				if (bPos.y - r < pos.y + size / 2.0) {
					if (children[1] == nullptr) {
						children[1] = new OctreeNode(Point3D(pos, size / 2.0, 0, size/2.0), size / 2.0, minSize);
					}
					children[1]->addBody(b);
				}
			}
			//negZ
			if (bPos.z - r < pos.z + size / 2.0) {
				//posY
				if (bPos.y + r > pos.y + size / 2.0) {
					if (children[2] == nullptr) {
						children[2] = new OctreeNode(Point3D(pos, size / 2.0, size/2.0, 0), size / 2.0, minSize);
					}
					children[2]->addBody(b);
				}
				//negY
				if (bPos.y - r < pos.y + size / 2.0) {
					if (children[3] == nullptr) {
						children[3] = new OctreeNode(Point3D(pos, size / 2.0, 0, 0), size / 2.0, minSize);
					}
					children[3]->addBody(b);
				}
			}
		}
		//negX
		if (bPos.x - r < pos.x + size / 2.0) {
			//posZ
			if (bPos.z + r > pos.z + size / 2.0) {
				//posY
				if (bPos.y + r > pos.y + size / 2.0) {
					if (children[4] == nullptr) {
						children[4] = new OctreeNode(Point3D(pos, 0, size / 2.0, size / 2.0), size / 2.0, minSize);
					}
					children[4]->addBody(b);
				}
				//negY
				if (bPos.y - r < pos.y + size / 2.0) {
					if (children[5] == nullptr) {
						children[5] = new OctreeNode(Point3D(pos, 0, 0, size/2.0), size / 2.0, minSize);
					}
					children[5]->addBody(b);
				}
			}
			//negZ
			if (bPos.z - r < pos.z + size / 2.0) {
				//posY
				if (bPos.y + r > pos.y + size / 2.0) {
					if (children[6] == nullptr) {
						children[6] = new OctreeNode(Point3D(pos, 0, size/2.0, 0), size / 2.0, minSize);
					}
					children[6]->addBody(b);
				}
				//negY
				if (bPos.y - r < pos.y + size / 2.0) {
					if (children[7] == nullptr) {
						children[7] = new OctreeNode(Point3D(pos, 0, 0, 0), size / 2.0, minSize);
					}
					children[7]->addBody(b);
				}
			}
		}
	}

	leafNode = true;
	for (int i = 0; i < N_CHILD; i++) {
		if (children[i] != nullptr) {
			leafNode = false;
			children[i]->expandNode();
		}
	}
}
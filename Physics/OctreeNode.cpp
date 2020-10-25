#include "OctreeNode.h";

OctreeNode::OctreeNode(Vector3D pos, double size, double minSize) {
	this->size = size;
	this->minSize = minSize;
	this->pos = pos;
	for (int i = 0; i < N_CHILD; i++) {
		children[i] = nullptr;
	}
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

Vector3D OctreeNode::getPos() {
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

void OctreeNode::getAllNodes(std::vector<OctreeNode*>* out) {
	getAllNodes(this, out);
}

void OctreeNode::getCollisionLeafs(std::vector<OctreeNode*>* out) {
	getCollisionLeafs(this, out);
}

bool OctreeNode::isLeafNode() {
	return leafNode;
}

void OctreeNode::expandNode() {

	if (size <= minSize || bodies.size() <= 1) {
		leafNode = true;
		return;//no more children, end tree
	}

	for (RigidBody* b : bodies) {
		Vector3D bPos = b->getCenterOfMass();
		Vector3D dim;
		if (b->getFixed()) {
			bPos = b->getDimCenter();
			dim = b->getDimensions().multiply(0.5);
		}
		else {
			double r = b->getCollisionRadius();
			dim = Vector3D(r, r, r);
		}
		

		//lack of else statements intentional,
		//as bodies can be in multiple nodes at once
		//e.g. sitting on edge

		//posX
		if (bPos.x + dim.x > pos.x + size / 2.0) {
			//posZ
			if (bPos.z + dim.z > pos.z + size / 2.0) {
				//posY
				if (bPos.y + dim.y > pos.y + size / 2.0) {
					if (children[0] == nullptr) {
						children[0] = new OctreeNode(pos.add(Vector3D(size / 2.0, size / 2.0, size / 2.0)), size / 2.0, minSize);
					}
					children[0]->addBody(b);
				}
				//negY
				if (bPos.y - dim.y < pos.y + size / 2.0) {
					if (children[1] == nullptr) {
						children[1] = new OctreeNode(pos.add(Vector3D(size / 2.0, 0, size/2.0)), size / 2.0, minSize);
					}
					children[1]->addBody(b);
				}
			}
			//negZ
			if (bPos.z - dim.z < pos.z + size / 2.0) {
				//posY
				if (bPos.y + dim.y > pos.y + size / 2.0) {
					if (children[2] == nullptr) {
						children[2] = new OctreeNode(pos.add(Vector3D(size / 2.0, size/2.0, 0)), size / 2.0, minSize);
					}
					children[2]->addBody(b);
				}
				//negY
				if (bPos.y - dim.y < pos.y + size / 2.0) {
					if (children[3] == nullptr) {
						children[3] = new OctreeNode(pos.add(Vector3D(size / 2.0, 0, 0)), size / 2.0, minSize);
					}
					children[3]->addBody(b);
				}
			}
		}
		//negX
		if (bPos.x - dim.x < pos.x + size / 2.0) {
			//posZ
			if (bPos.z + dim.z > pos.z + size / 2.0) {
				//posY
				if (bPos.y + dim.y > pos.y + size / 2.0) {
					if (children[4] == nullptr) {
						children[4] = new OctreeNode(pos.add(Vector3D(0, size / 2.0, size / 2.0)), size / 2.0, minSize);
					}
					children[4]->addBody(b);
				}
				//negY
				if (bPos.y - dim.y < pos.y + size / 2.0) {
					if (children[5] == nullptr) {
						children[5] = new OctreeNode(pos.add(Vector3D(0, 0, size/2.0)), size / 2.0, minSize);
					}
					children[5]->addBody(b);
				}
			}
			//negZ
			if (bPos.z - dim.z < pos.z + size / 2.0) {
				//posY
				if (bPos.y + dim.y > pos.y + size / 2.0) {
					if (children[6] == nullptr) {
						children[6] = new OctreeNode(pos.add(Vector3D(0, size/2.0, 0)), size / 2.0, minSize);
					}
					children[6]->addBody(b);
				}
				//negY
				if (bPos.y - dim.y < pos.y + size / 2.0) {
					if (children[7] == nullptr) {
						children[7] = new OctreeNode(pos.add(Vector3D(0, 0, 0)), size / 2.0, minSize);
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
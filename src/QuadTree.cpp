#include "box2d-lite/QuadTree.h"

#if SP_TYPE == 2

QuadTree::Node::Node(float centerX, float centerY, float halfWidth)
	: centerX(centerX), centerY(centerY), halfWidth(halfWidth)
{
	for (int i = 0; i < 4; i++) {
		children[i] = NULL;
	}
}

QuadTree::Node::~Node()
{
	// Recursively destroy the children of this node
	for (int i = 0; i < 4; i++) {
		delete children[i];
	}
}

QuadTree::Node* QuadTree::buildQuadTree(float centerX, float centerY, float halfWidth, int depth)
{
	// Stop if maximum depth is reached
	if (depth < 0) return NULL;

	// Create the current node
	Node* node = new Node(centerX, centerY, halfWidth);

	float offsetX, offsetY;
	float step = 0.5f * halfWidth;

	// Recursively construct the children of the current node
	for (int i = 0; i < 4; i++) {
		offsetX = ((i & 1) ? step : -step);
		offsetY = ((i & 2) ? step : -step);
		node->children[i] = buildQuadTree(centerX + offsetX, centerY + offsetY, step, depth - 1);
	}

	return node;
}

void QuadTree::insertBodyIntoQuadTree(Node* tree, Body* body, int level)
{
	/* Indices of the tree's children:
	* 2  |  3
	* ---|---
	* 0  |  1
	*/
	int index = 0; // The index of the child the object will be inserted in
	bool straddle = false; // Whether the object overlaps with any of the dividing axes
	
	Mat22 R(body->rotation);
	Vec2 pos = body->position;
	Vec2 h = 0.5f * body->width;

	Vec2 v1 = pos + R * Vec2(-h.x, -h.y);
	Vec2 v2 = pos + R * Vec2(h.x, -h.y);
	Vec2 v3 = pos + R * Vec2(h.x, h.y);
	Vec2 v4 = pos + R * Vec2(-h.x, h.y);

	//Vec2 extension = pos + R * halfWidth;

	// Determine the relative y-position of the body to the center of the node
	if (v1.y <= tree->centerY && v2.y <= tree->centerY && v3.y <= tree->centerY && v4.y <= tree->centerY) { // South
		index = 0;
	} else if (v3.y > tree->centerY && v4.y > tree->centerY && v1.y > tree->centerY && v2.y > tree->centerY) { // North
		index = 2;
	} else { // Overlaps with the x-axis
		straddle = true;
	}

	// Determine the relative x-position of the body to the center of the node
	// We can ignore the case where the object is in any of the west sided cells since the previous if-statement assumes that case
	if (v1.x >= tree->centerX && v2.x >= tree->centerX && v3.x >= tree->centerX && v4.x >= tree->centerX) { // East
		index += 1;
	} else if (!(v1.x < tree->centerX && v4.x < tree->centerX && v2.x < tree->centerX && v3.x < tree->centerX)) { // Overlaps with the y-axis
		straddle = true;
	}
	
	// Call this function with the appropriate child recursively if the body doesn't straddle and the max recursion depth hasn't been reached
	if (!straddle && level < QT_MAX_DEPTH) {

		// Create a new child if the node doesn't have one at the found index
		if (tree->children[index] == NULL) {
			float offsetX, offsetY;
			float step = 0.5f * tree->halfWidth;

			offsetX = ((index & 1) ? step : -step);
			offsetY = ((index & 2) ? step : -step);

			tree->children[index] = new Node(tree->centerX + offsetX, tree->centerY + offsetY, step);
		}

		insertBodyIntoQuadTree(tree->children[index], body, level + 1);

	} else { // Otherwise, insert the body into the current node
		tree->bodies.push_back(body);
	}

}

void QuadTree::flushQuadTree(Node* tree, int level)
{
	// Deletes all references to the current node's bodies
	tree->bodies.clear();

	// Recursively traverse the children of this node
	for (int i = 0; i < 4; i++) {
		
		if (tree->children[i] != NULL) {
			if (level < QT_MIN_DEPTH) {
				flushQuadTree(tree->children[i], level + 1);

			// Delete all nodes which are above the minimum depth
			} else {
				delete tree->children[i];
				tree->children[i] = NULL;
			}
		}
	}

}

QuadTree::Node* QuadTree::quadTree = QuadTree::buildQuadTree(0.0f, 0.0f, Max(WORLD_HEIGHT, WORLD_WIDTH) / 2.0f, QT_MIN_DEPTH);

#endif
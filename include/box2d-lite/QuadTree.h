#pragma once

#include "Base.h"

#if SP_TYPE == 2

#include <vector>
#include <map>

#include "Body.h"

#define QT_MIN_DEPTH 1
#define QT_MAX_DEPTH 8

#define QT_OFFSET_X 
#define QT_OFFSET_Y 

namespace QuadTree {

	struct Node {
		float centerX, centerY; // Center position of the node
		float halfWidth; // Half the width of the node

		std::vector<Body*> bodies; // The bodies which are inside this node

		// Pointers to the children
		Node* children[4];

		Node(float centerX, float centerY, float halfWidth);
		~Node();
	};

	/* Creates and populates a new QuadTree until the max depth is reached
	* @param centerX The x-coordinate of the root's center
	* @param centerY The y-coordinate of the root's center
	* @param halfWidth Half-Width of the root
	* @param depth The recursion depth
	*/
	Node* buildQuadTree(float centerX, float centerY, float halfWidth, int depth);

	/* Finds and inserts a body into the appropriate node
	* @param tree The root of the QuadTree
	* @param body The body which will be inserted into the QuadTree
	* @param level The recursion level (when inserting manually, this should be 0)
	*/
	void insertBodyIntoQuadTree(Node* tree, Body* body, int level = 0);

	/* Removes all body references and children above the minimum recursion depth
	* @param tree The QuadTree which will be flushed
	* @param level The current recursion level (starts at 0)
	*/
	void flushQuadTree(Node* tree, int level=0);

	extern Node* quadTree;

}

#endif
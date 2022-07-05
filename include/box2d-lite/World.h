/*
* Copyright (c) 2006-2009 Erin Catto http://www.gphysics.com
*
* Permission to use, copy, modify, distribute and sell this software
* and its documentation for any purpose is hereby granted without fee,
* provided that the above copyright notice appear in all copies.
* Erin Catto makes no representations about the suitability 
* of this software for any purpose.  
* It is provided "as is" without express or implied warranty.
*/

#ifndef WORLD_H
#define WORLD_H

#include <vector>
#include <map>
#include <chrono>
#include "Base.h"
#include "MathUtils.h"
#include "Arbiter.h"
#include "Sample.h"
#include "UniformGrid.h"
#include "QuadTree.h"

struct Body;
struct Joint;

struct World
{
	World(Vec2 gravity, int iterations) : gravity(gravity), iterations(iterations) {}

	void Add(Body* body);
	void Add(Joint* joint);
	void Clear();

	void Step(float dt);

	void BroadPhase();

#if SP_TYPE == 2

	/* Tests all possible collisions of the tree
	* @param tree The QuadTree containing the objects
	* @param possibleCollisions The number of possible collisions
	* @param actualCollisions The number of collisions that actually occured
	*/
	void testAllCollisions(QuadTree::Node* tree, int& possibleCollisions, int& actualCollisions);

#endif

	std::vector<Body*> bodies;
	std::vector<Joint*> joints;
	std::map<ArbiterKey, Arbiter> arbiters;
	Vec2 gravity;
	int iterations;
	static bool accumulateImpulses;
	static bool warmStarting;
	static bool positionCorrection;

};

#endif

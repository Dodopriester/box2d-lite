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

#include <iostream>

#include "box2d-lite/World.h"
#include "box2d-lite/Body.h"
#include "box2d-lite/Joint.h"
#include "box2d-lite/Sample.h"

#define NO_EXPORT_SAMPLING

using std::vector;
using std::map;
using std::pair;

typedef map<ArbiterKey, Arbiter>::iterator ArbIter;
typedef pair<ArbiterKey, Arbiter> ArbPair;

bool World::accumulateImpulses = true;
bool World::warmStarting = true;
bool World::positionCorrection = true;

void World::Add(Body* body)
{
	bodies.push_back(body);
}

void World::Add(Joint* joint)
{
	joints.push_back(joint);
}

void World::Clear()
{
	bodies.clear();
	joints.clear();
	arbiters.clear();
}

void World::BroadPhase()
{
	int foundCollisions = 0;
	int possibleCollisions = 0;
	double efficiency;
	bool shouldSample = false;

#ifndef NO_EXPORT_SAMPLING
	if (secondsPassed == samplingTime && currentFrame == 0) {
		std::string filename = "naive_200_trial_3"; //Name of the file without extension

		std::cout << "Sampling done" << std::endl;
		std::cout << "Writing to file " << filename << std::endl;

		exportSamplesToFile(filename);
	}
#endif

	if (currentFrame % (framerate / sampleRate) == 0 && secondsPassed < samplingTime) {
		shouldSample = true;
	}
	else {
		shouldSample = false;
	}
	
	auto start = std::chrono::steady_clock::now();

#ifdef USE_UGRID // Uniform Grid
	for (int i = 0; i < (int)bodies.size(); ++i) {
		Body* body = bodies[i];
	
		putBodyIntoCell(body);
	}

	int x_size = (int)(U_WORLD_WIDTH / U_GRID_CELL_SIZE) + 1;
	int y_size = (int)(U_WORLD_HEIGHT / U_GRID_CELL_SIZE) + 1;

	for (int x = 0; x < x_size; x++) {
		for (int y = 0; y < y_size; y++) {
			std::vector<Body*> cellBodies = cells[x][y]->bodies;

			// If there are less than two objects in a cell, then we can skip any collision checks for this cell
#ifndef DONT_SKIP_SINGLE_BODY_CELLS
			if (cellBodies.size() < 2)
				continue;
#endif

			for (int i = 0; i < (int)cellBodies.size(); ++i)
			{
				Body* bi = cellBodies[i];

				for (int j = i + 1; j < (int)cellBodies.size(); ++j)
				{
					Body* bj = cellBodies[j];

					if (bi->invMass == 0.0f && bj->invMass == 0.0f)
						continue;

					Arbiter newArb(bi, bj);
					ArbiterKey key(bi, bj);

					possibleCollisions++;

					if (newArb.numContacts > 0)
					{
						ArbIter iter = arbiters.find(key);
						if (iter == arbiters.end())
						{
							arbiters.insert(ArbPair(key, newArb));
						}
						else
						{
							iter->second.Update(newArb.contacts, newArb.numContacts);
						}

						foundCollisions++;
					}
					else
					{
						arbiters.erase(key);
					}
				}
			}
		}
	}

#else

	// O(n^2) broad-phase
	for (int i = 0; i < (int)bodies.size(); ++i)
	{
		Body* bi = bodies[i];

		for (int j = i + 1; j < (int)bodies.size(); ++j)
		{
			Body* bj = bodies[j];

			if (bi->invMass == 0.0f && bj->invMass == 0.0f)
				continue;

			Arbiter newArb(bi, bj);
			ArbiterKey key(bi, bj);

			possibleCollisions++;

			if (newArb.numContacts > 0)
			{
				ArbIter iter = arbiters.find(key);
				if (iter == arbiters.end())
				{
					arbiters.insert(ArbPair(key, newArb));
				}
				else
				{
					iter->second.Update(newArb.contacts, newArb.numContacts);
				}

				foundCollisions++;
			}
			else
			{
				arbiters.erase(key);
			}
		}
	}
#endif

	auto end = std::chrono::steady_clock::now();

	if (shouldSample) {
		efficiency = (float)foundCollisions / (float)possibleCollisions;

		Sample sample;
		sample.possibleCollisions = possibleCollisions;
		sample.actualCollisions = foundCollisions;
		sample.seconds = secondsPassed;
		sample.frame = currentFrame;
		sample.executionTime = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

		samples.push_back(sample);

		std::cout << "Found: " << possibleCollisions << " : Actual: " << foundCollisions
			<< " : Efficiency: " << efficiency*100 << "%" << std::endl;
	}

}

void World::Step(float dt)
{
	float inv_dt = dt > 0.0f ? 1.0f / dt : 0.0f;

	if (currentFrame < framerate) {
		currentFrame++;
	}
	else {
		currentFrame = 0;
		secondsPassed++;
	}

	// Determine overlapping bodies and update contact points.
	BroadPhase();

	// Integrate forces.
	for (int i = 0; i < (int)bodies.size(); ++i)
	{
		Body* b = bodies[i];

		if (b->invMass == 0.0f)
			continue;

		b->velocity += dt * (gravity + b->invMass * b->force);
		b->angularVelocity += dt * b->invI * b->torque;
	}

	// Perform pre-steps.
	for (ArbIter arb = arbiters.begin(); arb != arbiters.end(); ++arb)
	{
		arb->second.PreStep(inv_dt);
	}

	for (int i = 0; i < (int)joints.size(); ++i)
	{
		joints[i]->PreStep(inv_dt);	
	}

	// Perform iterations
	for (int i = 0; i < iterations; ++i)
	{
		for (ArbIter arb = arbiters.begin(); arb != arbiters.end(); ++arb)
		{
			arb->second.ApplyImpulse();
		}

		for (int j = 0; j < (int)joints.size(); ++j)
		{
			joints[j]->ApplyImpulse();
		}
	}

	// Integrate Velocities
	for (int i = 0; i < (int)bodies.size(); ++i)
	{
		Body* b = bodies[i];

		b->position += dt * b->velocity;
		b->rotation += dt * b->angularVelocity;

		b->force.Set(0.0f, 0.0f);
		b->torque = 0.0f;
	}
}

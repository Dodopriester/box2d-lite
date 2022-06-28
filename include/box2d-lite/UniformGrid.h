#pragma once

#define USE_UGRID

#ifdef USE_UGRID
#define U_GRID_CELL_SIZE 5.0f
#define U_WORLD_WIDTH 100.0f
#define U_WORLD_HEIGHT 70.0f
#define U_GRID_OFFSET_X -50.0f
#define U_GRID_OFFSET_Y -30.0f
//#define DONT_SKIP_SINGLE_BODY_CELLS

#include <vector>
#include <memory>
#include <algorithm>

#include "Body.h"

struct UGridCell {
	std::vector<Body*> bodies;
	float posX, posY; // The bottom left position of the cell

	UGridCell(float posX, float posY);
};

void initializeUGridCells();

void putBodyIntoCell(Body* body);

void flushCells();

extern std::shared_ptr<UGridCell> cells[(unsigned int)(U_WORLD_WIDTH / U_GRID_CELL_SIZE)+1][(unsigned int)(U_WORLD_HEIGHT / U_GRID_CELL_SIZE) + 1];

#endif
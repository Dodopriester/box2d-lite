#pragma once

#include "Base.h"

#if SP_TYPE == 1
#define U_GRID_CELL_SIZE 5.0f
#define U_GRID_OFFSET_X -50.0f
#define U_GRID_OFFSET_Y -30.0f
//#define DONT_SKIP_SINGLE_BODY_CELLS

#include <vector>
#include <memory>
#include <algorithm>

#include "Body.h"

namespace UGrid {

	struct UGridCell {
		std::vector<Body*> bodies;
		float posX, posY; // The bottom left position of the cell

		UGridCell(float posX, float posY);
	};

	void initializeUGridCells();

	void putBodyIntoCell(Body* body);

	// Deletes all object references from the grid cells
	void flushCells();

	// Contains all cells of the grid
	extern std::shared_ptr<UGridCell> cells[(unsigned int)(WORLD_WIDTH / U_GRID_CELL_SIZE) + 1][(unsigned int)(WORLD_HEIGHT / U_GRID_CELL_SIZE) + 1];

}

#endif
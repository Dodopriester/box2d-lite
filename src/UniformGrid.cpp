#include "box2d-lite/UniformGrid.h"

#ifdef USE_UGRID

//std::vector<std::vector<UGridCell*>> cells;
std::shared_ptr<UGridCell> cells[(unsigned int)(U_WORLD_WIDTH / U_GRID_CELL_SIZE) + 1][(unsigned int)(U_WORLD_HEIGHT / U_GRID_CELL_SIZE) + 1];

UGridCell::UGridCell(float posX, float posY) : posX(posX), posY(posY)
{
}

void initializeUGridCells()
{
	int x_size = (int)(U_WORLD_WIDTH / U_GRID_CELL_SIZE) + 1;
	int y_size = (int)(U_WORLD_HEIGHT / U_GRID_CELL_SIZE) + 1;
	
	for (int x = 0; x < x_size; x++) {
		for (int y = 0; y < y_size; y++) {
			cells[x][y] = std::make_shared<UGridCell>(UGridCell(U_GRID_OFFSET_X + (float)(U_GRID_CELL_SIZE * x), U_GRID_OFFSET_Y + (float)(U_GRID_CELL_SIZE * y)));
		}
	}
}

void putBodyIntoCell(Body* body)
{
	// Get vertices
	Mat22 R(body->rotation);
	Vec2 x = body->position;
	Vec2 h = 0.5f * body->width;

	Vec2 v1 = x + R * Vec2(-h.x, -h.y);
	Vec2 v2 = x + R * Vec2(h.x, -h.y);
	Vec2 v3 = x + R * Vec2(h.x, h.y);
	Vec2 v4 = x + R * Vec2(-h.x, h.y);

	int iX1, iY1;
	int iX2, iY2;
	int iX3, iY3;
	int iX4, iY4;

	// Find and insert the body into the cell where each vertex is in
	iX1 = (int)(Abs(v1.x - U_GRID_OFFSET_X) / U_GRID_CELL_SIZE);
	iY1 = (int)(Abs(v1.y - U_GRID_OFFSET_Y) / U_GRID_CELL_SIZE);

	iX2 = (int)(Abs(v2.x - U_GRID_OFFSET_X) / U_GRID_CELL_SIZE);
	iY2 = (int)(Abs(v2.y - U_GRID_OFFSET_Y) / U_GRID_CELL_SIZE);

	iX3 = (int)(Abs(v3.x - U_GRID_OFFSET_X) / U_GRID_CELL_SIZE);
	iY3 = (int)(Abs(v3.y - U_GRID_OFFSET_Y) / U_GRID_CELL_SIZE);

	iX4 = (int)(Abs(v4.x - U_GRID_OFFSET_X) / U_GRID_CELL_SIZE);
	iY4 = (int)(Abs(v4.y - U_GRID_OFFSET_Y) / U_GRID_CELL_SIZE);

	int minX = std::min<int>({ iX1, iX2, iX3, iX4 });
	int minY = std::min<int>({ iY1, iY2, iY3, iY4 });

	int maxX = std::max<int>({ iX1, iX2, iX3, iX4 });
	int maxY = std::max<int>({ iY1, iY2, iY3, iY4 });

	for (int x = minX; x <= maxX; x++) {
		for (int y = minY; y <= maxY; y++) {

			if(std::find(cells[x][y]->bodies.begin(), cells[x][y]->bodies.end(), body) == cells[x][y]->bodies.end()) // Only add body to cell if it isn't already in it
				cells[x][y]->bodies.push_back(body);
		}
	}

}

void flushCells()
{
	int x_size = (int)(U_WORLD_WIDTH / U_GRID_CELL_SIZE) + 1;
	int y_size = (int)(U_WORLD_HEIGHT / U_GRID_CELL_SIZE) + 1;

	for (int x = 0; x < x_size; x++) {
		for (int y = 0; y < y_size; y++) {
			cells[x][y]->bodies.clear();
		}
	}
}

#endif
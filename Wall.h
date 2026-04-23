#pragma once

#include "Square.h"

class Wall
{
private:
	MultiSquare wall;

public:
	Wall();
	bool intersect(Square obj);
	bool intersect(MultiSquare objs);
	bool intersect(Triangulus obj);
	MultiSquare getter() { return wall; }
};


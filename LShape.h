#pragma once

#include <cmath>

#include "Shape.h"


class LShape : public Shape
{
private:
	double long_side, short_side, long_pro, short_pro;
	int sym_angle;
	Polygon poly;

public:
	LShape();

	Polygon get_poly();
	void update(State3D pos);
	double getRadius();
	int get_symangle();
	MultiSquare get_square();
	bool intersect(Square sq);
	bool intersect_robot(Robot* robot);
	bool intersect_wall(Wall* wall);
};

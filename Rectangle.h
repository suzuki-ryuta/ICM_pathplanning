#pragma once

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/optional.hpp>

#include "Shape.h"
#include "Square.h"


class Rectangle : public Shape
{
private:
	const double short_side, long_side;
//	const int sym_angle;
	Polygon poly;

public:
	Rectangle();

	Polygon get_poly();
	void update(State3D pos);
	double getRadius();
//	int get_symangle();
	MultiSquare get_square();
	bool intersect(Square sq);
	bool intersect_robot(Robot* robot);
	bool intersect_wall(Wall* wall);
};


double read_shortside();
double read_longside();
//Polygon calc_polygon(State3D pos, double ss, double ls);

#pragma once

#include "Square.h"
#include "Robot.h"
#include "Wall.h"

class Polygon
{
private:
	std::vector<Point2D> vert;

public:
	Polygon();
	Polygon(std::vector<Point2D> vert);

	std::vector<Point2D> getter();
	Point2D get(int i);
	void add(Point2D pt);
};


class Shape
{
public:
	virtual Polygon get_poly() = 0;
	virtual void update(State3D pos) = 0;
	virtual double getRadius() = 0;
//	virtual int get_symangle() = 0;
	virtual bool intersect(Square sq) = 0;
	virtual bool intersect_robot(Robot* robot) = 0;
	virtual bool intersect_wall(Wall* wall) = 0;
};


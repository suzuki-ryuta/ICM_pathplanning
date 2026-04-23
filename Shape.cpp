#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/optional.hpp>

#include "Rectangle.h"
#include "LShape.h"

namespace bp = boost::property_tree;

#include "Shape.h"
#include <iostream>

Polygon::Polygon()
	:vert()
{
}

Polygon::Polygon(std::vector<Point2D> _vert)
	:vert(_vert)
{
}

std::vector<Point2D> Polygon::getter()
{
	return vert;
}


Point2D Polygon::get(int i)
{
	return vert[i];
}


void Polygon::add(Point2D pt)
{
	vert.push_back(pt);
}



#include "Rectangle.h"

namespace bp = boost::property_tree;

Rectangle::Rectangle()
	: short_side(70),
	  long_side(100)
//	  sym_angle(180)
{
}


void Rectangle::update(State3D pos)
{
	double radAngle = deg_to_rad(pos.th);
	double aspect = atan2(short_side, long_side);

	Point2D refPoint(pos.x - 0.5 * std::sqrt(short_side * short_side + long_side * long_side) * std::sin(aspect - radAngle),
		pos.y - 0.5 * std::sqrt(short_side * short_side + long_side * long_side) * std::cos(aspect - radAngle));
	Point2D p2(refPoint.x + short_side * std::cos(radAngle), refPoint.y + short_side * std::sin(radAngle));
	Point2D p3(p2.x - long_side * std::sin(radAngle), p2.y + long_side * std::cos(radAngle));
	Point2D p4(p3.x - short_side * std::cos(radAngle), p3.y - short_side * std::sin(radAngle));
	
	std::vector<Point2D> vert = { refPoint, p2, p3, p4 };
	poly = Polygon(vert);
}


MultiSquare Rectangle::get_square()
{
	return MultiSquare(poly.getter());
}

Polygon Rectangle::get_poly()
{
	return poly;
}

double Rectangle::getRadius()
{
	return sqrt(short_side * short_side + long_side * long_side);
}


//int Rectangle::get_symangle()
//{
//	return sym_angle;
//}

bool Rectangle::intersect(Square sq)
{
	return sq.intersect(Square(poly.getter()));
}


bool Rectangle::intersect_robot(Robot* robot)
{
	return robot->intersect(get_square());
}


bool Rectangle::intersect_wall(Wall* wall)
{
	return wall->intersect(get_square());
}


double read_shortside()
{
	bp::ptree pt;
	read_ini("config/ObjectParameter.ini", pt);
	boost::optional<int> carrier = pt.get_optional<int>("Rectangle.short_side");
	return carrier.get();
}

double read_longside()
{
	bp::ptree pt;
	read_ini("config/ObjectParameter.ini", pt);
	boost::optional<int> carrier = pt.get_optional<int>("Rectangle.long_side");
	return carrier.get();
}

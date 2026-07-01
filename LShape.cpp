#include "LShape.h"

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/optional.hpp>


namespace bp = boost::property_tree;

LShape::LShape()
	:long_side(150), short_side(100), long_pro(40), short_pro(40), sym_angle(360)
{
}


Polygon LShape::get_poly()
{
	return poly;
}

double LShape::getRadius()
{
	return sqrt(long_side * long_side + short_side * short_side);
}


int LShape::get_symangle()
{
	return sym_angle;
}


void LShape::update(State3D pos)
{
	double radAngle = deg_to_rad(pos.th);
	double aspect = atan2(long_side, short_side);

	Point2D refPoint(pos.x - 0.5 * std::sqrt(short_side * short_side + long_side * long_side) * std::cos(aspect + radAngle),
		             pos.y - 0.5 * std::sqrt(short_side * short_side + long_side * long_side) * std::sin(aspect + radAngle));
	Point2D p2(refPoint.x + short_side * std::cos(radAngle), refPoint.y + short_side * std::sin(radAngle));
	Point2D p3(p2.x - long_pro * std::sin(radAngle), p2.y + long_pro * std::cos(radAngle));
	Point2D p4(p3.x - (short_side - short_pro) * std::cos(radAngle), p3.y - (short_side - short_pro) * std::sin(radAngle));
	Point2D p5(p4.x - (long_side - long_pro) * std::sin(radAngle), p4.y + (long_side - long_pro) * std::cos(radAngle));
	Point2D p6(p5.x - short_pro * std::cos(radAngle), p5.y - short_pro * std::sin(radAngle));

	std::vector<Point2D> vert = { refPoint, p2, p3, p4, p5, p6 };
	poly = Polygon(vert);
}


MultiSquare LShape::get_square()
{
	Vector2D<double> edge1(Vector2D<double>(poly.get(3)) - Vector2D<double>(poly.get(2)));
	edge1.normalize();
	edge1 = edge1 * short_side;
	Point2D p3 = poly.get(2);
	Point2D p4new = p3 + edge1;
	Square sq1(poly.get(0), poly.get(1), poly.get(2), p4new);
	Square sq2(poly.get(3), poly.get(4), poly.get(5), p4new);
	std::vector<Square> msq = { sq1, sq2 };

	return MultiSquare(msq);
}


bool LShape::intersect(Square sq)
{
	MultiSquare msq = get_square();
	for(int i=0; i<(int)msq.size(); ++i){
		if(sq.intersect(msq.element(i)))	return true;
	}
	return false;
}


bool LShape::intersect_robot(Robot* robot)
{
	return robot->intersect(get_square());
}


bool LShape::intersect_wall(Wall* wall)
{
	return wall->intersect(get_square());
}

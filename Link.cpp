#include "Link.h"
#include "Square.h"


Link::Link(RecSize r, Mat22<int> coord, Point2D btm)
	:rect(r), coordination(coord), bottom(btm), vertices() 
{
	update(0, bottom);
}

void Link::set_bottom(Point2D bottom_center)
{
	bottom = bottom_center;
}

void Link::update(double abs_angle)
{
	double radAngle = deg_to_rad(abs_angle);
	Vector2D<double> trs;

	trs.x = -0.5 * rect.width * std::cos(radAngle);
	trs.y = -0.5 * rect.width * std::sin(radAngle);
	trs = coordination * trs;
	Point2D p1 = bottom + trs;

	trs.x = rect.width * std::cos(radAngle);
	trs.y = rect.width * std::sin(radAngle);
	trs = coordination * trs;
	Point2D p2 = p1 + trs;

	trs.x = -rect.height * std::sin(radAngle);
	trs.y = rect.height * std::cos(radAngle);
	trs = coordination * trs;
	Point2D p3 = p2 + trs;

	trs.x = -rect.width * std::cos(radAngle);
	trs.y = -rect.width * std::sin(radAngle);
	trs = coordination * trs;
	Point2D p4 = p3 + trs;

	calc_top(abs_angle);

	vertices = Square(p1, p2, p3, p4);
}


void Link::update(double abs_angle, Point2D bottom_center)
{
	set_bottom(bottom_center);
	update(abs_angle);
}


bool Link::intersect(Square other)
{
	if (vertices.intersect(other))	return true;
	return false;
}

bool Link::intersect(MultiSquare others)
{
	for (int i = 0; i < (int)others.size(); ++i){
		if (intersect(others.element(i)))	return true;
	}
	return false;
}

bool Link::intersect(Triangulus other)
{
	if(other.intersect(vertices))	return true;
	return false;
}



void Link::calc_top(double abs_angle)
{
	double radAngle = deg_to_rad(abs_angle);

	Vector2D<double> cen_trs(-rect.height * std::sin(radAngle), rect.height * std::cos(radAngle));
	cen_trs = coordination * cen_trs;
	top = bottom + cen_trs;
}

Point2D Link::get_top() const
{
	return top;
}

Square Link::get_square() const
{
	return vertices;
}

Point2D Link::get_center() const
{
	return vertices.get_center();
}

double Link::get_radius() const
{
	return vertices.get_radius();
}

void Link::change_size(RecSize r)
{
	rect = r;
}

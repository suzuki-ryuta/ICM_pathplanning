#pragma once

#include "Square.h"


class Link
{
private:
	RecSize rect;
	const Mat22<int> coordination;

	Point2D bottom, top;
	Square vertices;

public:
	Link(RecSize r, Mat22<int> coord, Point2D btm);

	void set_bottom(Point2D bottom_center);
	void update(double abs_angle);
	void update(double abs_angle, Point2D bottom_center);
	bool intersect(Square other);
	bool intersect(MultiSquare others);
	bool intersect(Triangulus other);

	void calc_top(double abs_angle);
	Point2D get_center() const;
	double get_radius() const;
	Point2D get_top() const;
	Square get_square() const;

	void change_size(RecSize r);
};


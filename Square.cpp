#include <cassert>

#include "Square.h"


OBB::OBB(std::vector<Point2D> poly, Vector2D<double> dir)
	:min(DBL_MAX), max(-DBL_MAX)
{
	assert(fabs(dir.norm() - 1.0) < 0.001);
	for (int i = 0; i < (int)poly.size(); ++i) {
		Vector2D<double> tmp(poly[i]);
		double dot = tmp.dot(dir);
		if (dot < min)	min = dot;
		if (dot > max)  max = dot;
	}
}


OBB::OBB()
	:min(), max()
{}


OBB::OBB(const OBB& obb)
	:min(obb.min), max(obb.max)
{
}


bool OBB::intersect_along_axis(const OBB& oth)
{
	return min > oth.max || max < oth.min;
}



// Definition of Square Class ===========================

Square::Square()
	:vertices()
{
	vertices.resize(4);
}

Square::Square(Point2D p1, Point2D p2, Point2D p3, Point2D p4)
	:vertices()
{
	vertices.resize(4);
	vertices[0] = p1;
	vertices[1] = p2;
	vertices[2] = p3;
	vertices[3] = p4;
}


Square::Square(std::vector<Point2D> v)
	:vertices(v)
{
	assert((int)v.size() == 4);
}


Square::Square(const Square& sq)
	:vertices()
{
	this->vertices = sq.vertices;
}


std::vector<Point2D> Square::get_vertices() const
{
	return vertices;
}


std::vector<Vector2D<double>> Square::get_edge_vector() const
{
	std::vector<Vector2D<double>> axis;
	Vector2D<double> tmp(vertices[1].x - vertices[0].x, vertices[1].y - vertices[0].y);
	tmp.normalize();
	
	Vector2D<double> tmp2(vertices[2].x - vertices[1].x, vertices[2].y - vertices[1].y);
	tmp2.normalize();

	axis.push_back(tmp);	
	axis.push_back(tmp2);

	return axis;
}

Point2D Square::get_center() const
{
	Vector2D<double> trs(vertices[2].x - vertices[0].x,
				         vertices[2].y - vertices[0].y);
	trs /= 2;

	return Point2D(vertices[0].x + trs.x, vertices[0].y + trs.y);
}


double Square::get_radius() const
{
	Vector2D<double> edge1(vertices[1].x - vertices[0].x, vertices[1].y - vertices[0].y);
	Vector2D<double> edge2(vertices[2].x - vertices[1].x, vertices[2].y - vertices[1].y);
	return sqrt(edge1.norm() * edge1.norm() + edge2.norm() * edge2.norm()) / 2;
}


std::vector<OBB> Square::makeOBB(std::vector<Vector2D<double>> axis)
{
	std::vector<OBB> linkobb;
	for (int ind = 0; ind < (int)axis.size(); ++ind) {
		OBB tmp(vertices, axis[ind]);
		linkobb.push_back(tmp);
	}

	return linkobb;
}


bool Square::intersect(Square other)
{
	std::vector<Vector2D<double>> ax1 = get_edge_vector();
	std::vector<Vector2D<double>> ax2 = other.get_edge_vector();
	std::vector<Vector2D<double>> axis = { ax1[0], ax1[1], ax2[0], ax2[1] };


	std::vector<OBB> me = makeOBB(axis);
	std::vector<OBB> you = other.makeOBB(axis);


	// In judge phase, Not intersect -> true	intersect -> false
	// but for return value, true -> intersect, false -> NOT intersect
	bool judge = false;
	for (int ind = 0; ind < (int)me.size(); ++ind) {
		judge = me[ind].intersect_along_axis(you[ind]); //分離軸の定理
		if (judge == true)
			return false;
	}

	return true;
}


bool Square::intersect(MultiSquare others)
{
	return others.intersect(*this);
}


bool Square::intersect(Triangulus other)
{
	return other.intersect(*this);
}


///--------------------------------------------------------------------------
///--------------------------------------------------------------------------


MultiSquare::MultiSquare()
	:msqr()
{
}

MultiSquare::MultiSquare(Square sq)
	: msqr()
{
	msqr.push_back(sq);
}

MultiSquare::MultiSquare(std::vector<Square> sqs)
	: msqr(sqs)
{
}


void MultiSquare::push(Square sq)
{
	msqr.push_back(sq);
}


bool MultiSquare::intersect(Square other)
{
	for (int i = 0; i < (int)msqr.size(); ++i)
	{
		if (msqr[i].intersect(other))	return true;
	}

	return false;
}


bool MultiSquare::intersect(MultiSquare others)
{
	for(int i=0; i<others.size(); ++i)
	{
		if(intersect(others.element(i)))	return true;
	}
	return false;
}


bool MultiSquare::intersect(Triangulus other)
{
	return other.intersect(*this);
}


///--------------------------------------------------------------------------
///--------------------------------------------------------------------------


Triangulus::Triangulus()
	:vertices()
{
	vertices.resize(3);
}


Triangulus::Triangulus(Point2D p1, Point2D p2, Point2D p3)
	:vertices()
{
	vertices.resize(3);
	vertices[0] = p1;
	vertices[1] = p2;
	vertices[2] = p3;
}

Triangulus::Triangulus(std::vector<Point2D> v)
	:vertices(v)
{
}


Triangulus::Triangulus(const Triangulus& tri)
	:vertices()
{
	this->vertices = tri.vertices;
}


std::vector<Point2D> Triangulus::get_vertices() const
{
	return vertices;
}


bool Triangulus::intersect(Square other)
{
	std::vector<Vector2D<double>> ax1 = get_edge_vector();
	std::vector<Vector2D<double>> ax2 = other.get_edge_vector();
	std::vector<Vector2D<double>> axis = { ax1[0], ax1[1], ax1[2], ax2[0], ax2[1] };
	
	std::vector<OBB> me = makeOBB(axis);
	std::vector<OBB> you = other.makeOBB(axis);

	// In judge phase, Not intersect -> true	intersect -> false
	// but for return value, true -> intersect, false -> NOT intersect
	bool judge = false;
	for (int ind = 0; ind < (int)me.size(); ++ind) {
		judge = me[ind].intersect_along_axis(you[ind]);
		if (judge == true)
			return false;
	}

	return true;
}


bool Triangulus::intersect(MultiSquare others)
{
	for(int i=0; i<(int)others.size(); ++i){
		if(intersect(others.element(i)))	return true;
	}
	return false;
}


std::vector<OBB> Triangulus::makeOBB(std::vector<Vector2D<double>> axis)
{
	std::vector<OBB> linkobb;
	for (int ind = 0; ind < (int)axis.size(); ++ind) {
		OBB tmp(vertices, axis[ind]);
		linkobb.push_back(tmp);
	}

	return linkobb;
}


std::vector<Vector2D<double>> Triangulus::get_edge_vector() const
{
	std::vector<Vector2D<double>> axis;
	Vector2D<double> tmp(vertices[1].x - vertices[0].x, vertices[1].y - vertices[0].y);
	tmp.normalize();
	
	Vector2D<double> tmp2(vertices[2].x - vertices[1].x, vertices[2].y - vertices[1].y);
	tmp2.normalize();

	Vector2D<double> tmp3(vertices[0].x - vertices[2].x, vertices[0].y - vertices[2].y);
	tmp3.normalize();

	axis.push_back(tmp);	
	axis.push_back(tmp2);
	axis.push_back(tmp3);

	return axis;
}














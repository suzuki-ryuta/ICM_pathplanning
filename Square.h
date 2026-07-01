#pragma once

#include <vector>
#include <cassert>

#include "icmMath.h"

class RecSize
{
public:
	int height;
	int width;

	RecSize(int h, int w)
	{
		assert(h > 0 && w > 0);
		height = h;
		width = w;
	}
};


struct OBB
{
	double min;
	double max;

	OBB(std::vector<Point2D> poly, Vector2D<double> dir);
	OBB();
	OBB(const OBB& obb);

	bool intersect_along_axis(const OBB& oth);

};


class MultiSquare;
class Triangulus;
class Square
{
protected:
	std::vector<Point2D> vertices;

public:
	Square();
	Square(Point2D p1, Point2D p2, Point2D p3, Point2D p4);
	Square(std::vector<Point2D> v);
	Square(const Square& sq);

	std::vector<Point2D> get_vertices() const;
	Point2D get_center() const;
	double get_radius() const;

	bool intersect(Square other);
	bool intersect(MultiSquare others);
	bool intersect(Triangulus other);

	std::vector<OBB> makeOBB(std::vector<Vector2D<double>> axis);
	std::vector<Vector2D<double>> get_edge_vector() const;
};


class MultiSquare
{
private:
	std::vector<Square> msqr;

public:
	MultiSquare();
	MultiSquare(Square sq);
	MultiSquare(std::vector<Square> sqs);

	inline Square element(int i) { return msqr[i]; }
	inline int size() { return (int)msqr.size(); }
	void push(Square sq);
	bool intersect(Square other);
	bool intersect(MultiSquare others);
	bool intersect(Triangulus other);
};


class Triangulus
{
private:
	std::vector<Point2D> vertices;

public:
	Triangulus();
	Triangulus(Point2D p1, Point2D p2, Point2D p3);
	Triangulus(std::vector<Point2D> v);
	Triangulus(const Triangulus& tri);

	std::vector<Point2D> get_vertices() const;

	bool intersect(Square other);
	bool intersect(MultiSquare others);

	std::vector<OBB> makeOBB(std::vector<Vector2D<double>> axis);
	std::vector<Vector2D<double>> get_edge_vector() const;
};

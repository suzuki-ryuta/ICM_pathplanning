#include "Wall.h"
#include "Triangle.h"


Wall::Wall()
	:wall()
{
	Point2D w1(-300, -5.001);
	Point2D w2(-300, -0.001);
	Point2D w3(300, -0.001);
	Point2D w4(300, -5.001);
	Square wall1(w1, w2, w3, w4);
	wall.push(wall1);

//	Point2D w5(-300, 415.001);
//	Point2D w6(-300, 420.001);
//	Point2D w7(300, 420.001);
//	Point2D w8(300, 415.001);
//	Square wall2(w5, w6, w7, w8);
//	wall.push(wall2);
}


bool Wall::intersect(Square obj)
{
	return wall.intersect(obj);
}

bool Wall::intersect(MultiSquare objs)
{
	for (int i = 0; i < (int)objs.size(); ++i) {
		if (wall.intersect(objs.element(i)))	return true;
	}

	return false;
}

bool Wall::intersect(Triangulus obj)
{
	return obj.intersect(wall);
}

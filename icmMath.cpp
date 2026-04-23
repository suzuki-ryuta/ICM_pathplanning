#include "icmMath.h"

const double PI = 3.14159265;

double deg_to_rad(double deg)
{
	return deg * PI / 180;
}


double rad_to_deg(double rad)
{
	return rad * 180 / PI;
}


double distance(Point2D p1, Point2D p2)
{
	return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
}

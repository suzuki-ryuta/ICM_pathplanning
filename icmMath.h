#pragma once

#include <vector>
#include <math.h>
#include <cmath>
#include <cfloat>
#include <fstream>

class Point2D
{
public:
	double x, y;

	Point2D(double _x, double _y) {
		x = _x;
		y = _y;
	}
	Point2D() {
		x = 0;
		y = 0;
	}
	Point2D(const Point2D& p) {
		this->x = p.x;
		this->y = p.y;
	}
	Point2D& operator=(const Point2D& p) {
		this->x = p.x;	this->y = p.y;	return *this;
	}

	bool operator==(const Point2D& p) {
		if (fabs(x - p.x) < DBL_EPSILON &&
			fabs(y - p.y) < DBL_EPSILON)	return true;
		return false;
	}

	double dist(const Point2D& p) {
		return std::sqrt((this->x - p.x) * (this->x - p.x) + (this->y - p.y) * (this->y - p.y));
	}
	void zeroclear() {
		this->x = 0;	this->y = 0;
	}

};


class State3D
{
public:
	int x, y, th;

	State3D(int _x, int _y, int _th)
	{
		x = _x;	y = _y;	th = _th;
	}
	State3D()
	{
		x = 0;	y = 0;	th = 0;
	}

	State3D& operator=(const State3D& s)
	{
		this->x = s.x;	this->y = s.y;	this->th = s.th;
		return *this;
	}
		double dist(const State3D& other) const {
    	double dx = static_cast<double>(x - other.x);
    	double dy = static_cast<double>(y - other.y);
    	double dth = static_cast<double>(th - other.th);

    	// 角度の差を 0～180 に正規化（360度周期のため）
    	dth = std::fmod(std::abs(dth), 360.0);
    	if (dth > 180.0) dth = 360.0 - dth;

    	return std::sqrt(dx * dx + dy * dy + dth * dth);
	}
};

inline bool operator==(const State3D& s1, const State3D& s2)
{
	if (s1.x == s2.x && s1.y == s2.y && s1.th == s2.th)	return true;
	return false;
}

inline bool operator<(const State3D& s1, const State3D& s2)
{
	if (s1.x != s2.x) return s1.x < s2.x;
	if (s1.y != s2.y) return s1.y < s2.y;
	return s1.th < s2.th;
}

//inline bool operator==(const State3D& s1, const State3D& s2)
//{
//	if (fabs(s1.x - s2.x) < DBL_EPSILON &&
//		fabs(s1.y - s2.y) < DBL_EPSILON &&
//		fabs(s1.th - s2.th) < DBL_EPSILON)	return true;
//	
//	return false;
//}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//   Vector Class  ///////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename T>
class Vector2D
{
public:
	Vector2D(T _x, T _y) {
		x = _x;	y = _y;
	}

	Vector2D() {
		x = 0;  y = 0;
	}

	// to positional Vector
	Vector2D(const Point2D& p) {
		x = p.x;	y = p.y;
	}

	Vector2D operator+(Vector2D oth) {
		return Vector2D(x + oth.x, y + oth.y);
	}

	Vector2D operator-(Vector2D oth) {
		return Vector2D(x - oth.x, y - oth.y);
	}

	Vector2D operator*(double r) {
		return Vector2D(x * r, y * r);
	}

	Vector2D& operator/=(double d) {
		this->x /= d;
		this->y /= d;
		return *this;
	}

	double norm() {
		return std::sqrt(x * x + y * y);
	}

	void normalize() {
		*this /= norm();
	}

	double dot(const Vector2D& vec)	const
	{
		return this->x * vec.x + this->y * vec.y;
	}

	T x;
	T y;
};

template <typename T>
class Vector3D
{
public:
	Vector3D(T _x, T _y, T _z) {
		x = _x;	y = _y;	z = _z;
	}

	Vector3D() {
		x = 0;  y = 0;	z = 0;
	}

	Vector3D(const Vector2D<T>& vec) {
		x = vec.x;	y = vec.y;	z = 1;
	}

	Vector3D(const Point2D& pt) {
		x = pt.x;	y = pt.y;	z = 1;
	}

	Vector3D& operator/=(double d) {
		this->x /= d;
		this->y /= d;
		this->z /= d;
		return *this;
	}

	double norm() {
		return std::sqrt(x * x + y * y + z * z);
	}

	void normalize() {
		*this /= norm();
	}

	double dot(const Vector3D& vec)	const
	{
		return this->x * vec.x + this->y * vec.y + this->z * vec.z;
	}

	T x;
	T y;
	T z;
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//   Matrix Class  ///////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename T>
class Mat22
{
public:
	Mat22(T e1, T e2,
		  T e3, T e4)
	{
		p11 = e1; p12 = e2;
		p21 = e3; p22 = e4;
	}

	Mat22()
	{
		p11 = 0; p12 = 0;
		p21 = 0; p22 = 0;
	}

	T p11, p12,
	  p21, p22;
};

template <typename T>
class Mat33
{
public:
	Mat33(T e1, T e2, T e3,
		  T e4, T e5, T e6,
		  T e7, T e8, T e9)
	{
		p11 = e1;	p12 = e2; p13 = e3;
		p21 = e4; p22 = e5; p23 = e6;
		p31 = e7; p32 = e8; p33 = e9;
	}

	Mat33()
	{
		p11 = 0; p12 = 0; p13 = 0;
		p21 = 0; p22 = 0; p23 = 0;
		p31 = 0; p32 = 0; p33 = 0;
	}

	Mat33(const Mat22<T>& mat)
	{
		p11 = mat.p11; p12 = mat.p12; p13 = 0;
		p21 = mat.p21; p22 = mat.p22; p23 = 0;
		p31 = 0; 	   p32 = 0; 	  p33 = 1;
	}

	Vector3D<T> dot(const Vector3D<T>& vec) const
	{
		T tx = 0, ty = 0, tz = 0;
		tx = p11 * vec.x + p12 * vec.y + p13 * vec.z;
		ty = p21 * vec.x + p22 * vec.y + p23 * vec.z;
		tz = p31 * vec.x + p32 * vec.y + p33 * vec.z;

		return Vector3D<T>(tx, ty, tz);
	}

	T p11, p12, p13,
	  p21, p22, p23,
	  p31, p32, p33;
};

template <typename T1, typename T2>
inline Vector2D<T1> operator*(const Mat22<T2>& mat, const Vector2D<T1>& v) {
	return Vector2D<T1>(mat.p11 * v.x + mat.p12 * v.y, mat.p21 * v.x + mat.p22 * v.y);
}

template <typename T>
inline Point2D operator+(Point2D& p, Vector2D<T>& v) {
	return Point2D(p.x + v.x, p.y + v.y);
}

double deg_to_rad(double deg);
double rad_to_deg(double rad);

double distance(Point2D p1, Point2D p2);

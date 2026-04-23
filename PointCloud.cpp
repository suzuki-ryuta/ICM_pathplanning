#include <fstream>

#include "PointCloud.h"
#include "Shape.h"

PointCloud::PointCloud()
	:elm()
{}

PointCloud::PointCloud(std::vector<State3D> sts)
	: elm(sts)
{}

void PointCloud::push(State3D pos)
{
	elm.push_back(pos);
}


void PointCloud::pop()
{
	elm.pop_back();
}


bool PointCloud::exist(State3D st) const
{
	for (int i = 0; i < size(); ++i) {
		if (elm[i] == st)	return true;
	}

	return false;
}


bool PointCloud::overlap(PointCloud pc) const
{
	for (int i = 0; i < pc.size(); ++i) {
		if (exist(pc.get(i)))	return true;
	}

	return false;
}


bool PointCloud::contain_pfar(Vector2D<int> pfar) const
{
	for (int i = 0; i < (int)elm.size(); ++i) {
		if ((elm[i].x == pfar.x) ||
		    (elm[i].y == pfar.y))  return true;
	}

	return false;
}


//void PointCloud::order()
//{
//
//}


std::ostream& operator<<(std::ostream& out, const PointCloud &pc) 
{
	out << "size: " << pc.size() << std::endl;
	out << "{";
	for(int i=0; i<pc.size(); ++i){
		out << "[" << pc.get(i).x << "," 
			       << pc.get(i).y << ","
				   << pc.get(i).th << "]";
	}
	out << "}";

	return out; 
}

PCMerge::PCMerge(std::vector<PointCloud> _pcs, int symmetry)
	:subject(_pcs), theta_edge(symmetry)
{
}


void PCMerge::merge()
{
	while (1) {
		if (scan_one())	break;
	}
}


bool PCMerge::scan_one()
{
	for (int i = 0; i < (int)subject.size(); ++i) {
		for (int j = 0; j < subject[i].size(); ++j) {
			if (subject[i].elm[j].th == 0 ||
				subject[i].elm[j].th == theta_edge) {
				Point2D sub1(subject[i].elm[j].x, subject[i].elm[j].y);
				if (!scan_others(i, sub1))	return false;

			}
		}
	}
	return true;
}


bool PCMerge::scan_others(int ind, Point2D sub1)
{
	for (int k = ind + 1; k < (int)subject.size(); ++k) {
		for (int l = 0; l < subject[k].size(); ++l) {
			if (subject[k].elm[l].th == 0 || 
				subject[k].elm[l].th == theta_edge) {
				Point2D sub2(subject[k].elm[l].x, subject[k].elm[l].y);
				if (sub1 == sub2) {
					insert(ind, k);
					return false;
				}
			}
		}
	}
	return true;
}


void PCMerge::insert(int left, int right)
{
	for (int i = 0; i < subject[right].size(); ++i) {
		subject[left].push(subject[right].get(i));
	}
	subject.erase(subject.begin() + right);
}


std::vector<PointCloud> PCMerge::getter()
{
	return subject;
}


//bool PCMerge::contain_theta_edge(State3D sub)
//{
//	if (sub.th == 0 || sub.th == theta_edge)	return true;
//	return false;
//}
//
//bool contain_theta_edge(State3D sub, int edge_angle)
//{
//	if (sub.th == 0 || sub.th == edge_angle)	return true;
//	return false;
//}

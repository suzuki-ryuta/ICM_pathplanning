#include <fstream>
#include <set>
#include <algorithm>
#include <cmath>

#include "PointCloud.h"
#include "Shape.h"

namespace {

bool valid_cell(Vector3D<int> cell)
{
	return cell.x > 0 && cell.y > 0 && cell.z > 0;
}

bool axis_overlap(int a, int aw, int b, int bw)
{
	return 2LL * std::llabs((long long)a - b) <= (long long)aw + bw;
}

int theta_delta(int a, int b)
{
	int diff = std::abs(a - b) % 360;
	return diff > 180 ? 360 - diff : diff;
}

bool theta_overlap(int a, int aw, int b, int bw)
{
	return 2LL * theta_delta(a, b) <= (long long)aw + bw;
}

bool cell_overlap(State3D lhs, Vector3D<int> lhs_cell,
                  State3D rhs, Vector3D<int> rhs_cell)
{
	if (!valid_cell(lhs_cell)) lhs_cell = Vector3D<int>(0, 0, 0);
	if (!valid_cell(rhs_cell)) rhs_cell = Vector3D<int>(0, 0, 0);

	return axis_overlap(lhs.x, lhs_cell.x, rhs.x, rhs_cell.x)
	    && axis_overlap(lhs.y, lhs_cell.y, rhs.y, rhs_cell.y)
	    && theta_overlap(lhs.th, lhs_cell.z, rhs.th, rhs_cell.z);
}

double normalized_cell_weight(Vector3D<int> cell, Vector3D<int> unit)
{
	if (!valid_cell(cell) || !valid_cell(unit)) return 1.0;
	const double wx = std::max(1.0, (double)cell.x / unit.x);
	const double wy = std::max(1.0, (double)cell.y / unit.y);
	const double wz = std::max(1.0, (double)cell.z / unit.z);
	return wx * wy * wz;
}

} // namespace

PointCloud::PointCloud()
	:elm(), cell_size(), default_cell_size(), use_default_cell_size(false)
{}

PointCloud::PointCloud(std::vector<State3D> sts)
	: elm(sts), cell_size(sts.size()), default_cell_size(), use_default_cell_size(false)
{}

void PointCloud::push(State3D pos)
{
	elm.push_back(pos);
	cell_size.push_back(use_default_cell_size ? default_cell_size : Vector3D<int>());
}

void PointCloud::push(State3D pos, Vector3D<int> cell)
{
	elm.push_back(pos);
	cell_size.push_back(cell);
}

void PointCloud::push_from(const PointCloud& pc, int num)
{
	push(pc.get(num), pc.get_cell_size(num));
}


void PointCloud::pop()
{
	elm.pop_back();
	if (!cell_size.empty()) cell_size.pop_back();
}

Vector3D<int> PointCloud::get_cell_size(int num) const
{
	if (0 <= num && num < (int)cell_size.size()) return cell_size[num];
	if (use_default_cell_size) return default_cell_size;
	return Vector3D<int>();
}

void PointCloud::set_default_cell_size(Vector3D<int> cell)
{
	default_cell_size = cell;
	use_default_cell_size = valid_cell(cell);
}

void PointCloud::set_cell_size_for_all(Vector3D<int> cell)
{
	set_default_cell_size(cell);
	cell_size.assign(elm.size(), cell);
}

bool PointCloud::has_cell_size() const
{
	if (use_default_cell_size) return true;
	for (const auto& cell : cell_size) {
		if (valid_cell(cell)) return true;
	}
	return false;
}

bool PointCloud::overlaps_cell(State3D st, Vector3D<int> cell) const
{
	for (int i = 0; i < size(); ++i) {
		if (cell_overlap(elm[i], get_cell_size(i), st, cell)) return true;
	}
	return false;
}

double PointCloud::weighted_size(Vector3D<int> unit) const
{
	double total = 0.0;
	for (int i = 0; i < size(); ++i) {
		total += normalized_cell_weight(get_cell_size(i), unit);
	}
	return total;
}

bool PointCloud::exist(State3D st) const
{
	if (has_cell_size()) return overlaps_cell(st, Vector3D<int>());

	for (int i = 0; i < size(); ++i) {
		if (elm[i] == st)	return true;
	}

	return false;
}


bool PointCloud::overlap(PointCloud pc) const
{
	if (has_cell_size() || pc.has_cell_size()) {
		for (int i = 0; i < pc.size(); ++i) {
			if (overlaps_cell(pc.get(i), pc.get_cell_size(i))) return true;
		}
		return false;
	}

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

double PointCloud::iou(const PointCloud& other) const
{
	if (has_cell_size() || other.has_cell_size()) {
		if (size() == 0 && other.size() == 0) return 0.0;

		int lhs_hits = 0;
		for (int i = 0; i < size(); ++i) {
			if (other.overlaps_cell(get(i), get_cell_size(i))) ++lhs_hits;
		}

		int rhs_hits = 0;
		for (int i = 0; i < other.size(); ++i) {
			if (overlaps_cell(other.get(i), other.get_cell_size(i))) ++rhs_hits;
		}

		const double intersection = 0.5 * (lhs_hits + rhs_hits);
		const double union_size = size() + other.size() - intersection;
		return union_size <= 0.0 ? 0.0 : intersection / union_size;
	}

	int intersection = 0;
	int union_size = 0;
	std::set<State3D> union_set;

	// 交差を計算
	for (const auto& pt : elm) {
		if (other.exist(pt)) {
			intersection++;
		}
		union_set.insert(pt);
	}

	// 和を計算
	for (const auto& pt : other.elm) {
		union_set.insert(pt);
	}

	union_size = union_set.size();

	if (union_size == 0) return 0.0;
	return static_cast<double>(intersection) / union_size;
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
		subject[left].push_from(subject[right], i);
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

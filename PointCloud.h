#pragma once

#include <string>
#include <vector>

#include "icmMath.h"

struct PointCloud
{
	std::vector<State3D> elm;
	std::vector<Vector3D<int>> cell_size;
	Vector3D<int> default_cell_size;
	bool use_default_cell_size;

public:
	PointCloud();
	PointCloud(std::vector<State3D> sts);

	void push(State3D st);
	void push(State3D st, Vector3D<int> cell);
	void push_from(const PointCloud& pc, int num);
	void pop();

	// PointCloud.h の public: の中
	inline std::vector<State3D>::iterator begin() { return elm.begin(); }
	inline std::vector<State3D>::iterator end()   { return elm.end(); }
	inline std::vector<State3D>::const_iterator begin() const { return elm.begin(); }
	inline std::vector<State3D>::const_iterator end()   const { return elm.end(); }
	inline State3D& operator[](int i) { return elm[i]; }
	inline const State3D& operator[](int i) const { return elm[i]; }


	inline int size() const { return (int)elm.size(); }
	inline State3D get(int num) const { return elm[num]; }
	Vector3D<int> get_cell_size(int num) const;
	void set_default_cell_size(Vector3D<int> cell);
	void set_cell_size_for_all(Vector3D<int> cell);
	bool has_cell_size() const;
	bool overlaps_cell(State3D st, Vector3D<int> cell) const;
	double weighted_size(Vector3D<int> unit) const;
	inline bool empty() const { return elm.empty(); }  // ← ★これを追加！
	bool exist(State3D st) const;
	bool overlap(PointCloud pc) const;

	bool contain_pfar(Vector2D<int> pfar) const;
	void order();
	double iou(const PointCloud& other) const;
};

std::ostream& operator<<(std::ostream& out, const PointCloud &pc);

struct PointMark
{
	State3D pt;
	bool mk;

	PointMark()
		:pt(), mk(true){}

	PointMark(State3D _pt, bool _mk)
		:pt(_pt), mk(_mk)
	{
	}

	State3D getState() { return pt; }

	void toFalse() {
		mk = false;
	}

	void toTrue() {
		mk = true;
	}

	bool getMK()
	{
		return mk;
	}
	State3D getpt()
	{
		return pt;
	}

};


struct PointMarkCloud
{
	std::vector<PointMark> pmc;

	PointMarkCloud()
		:pmc() {}
	PointMarkCloud(PointCloud pc)
		:pmc()
	{
		int pcnum = (int)pc.size();
		pmc.resize(pcnum);
		for (int i = 0; i < pcnum; ++i) {
			pmc[i] = PointMark(pc.get(i), false);
		}
	}

	int size() { return (int)pmc.size(); }
	void resize(int num) { pmc.resize(num); }

	void assign(PointCloud pc){
		int pcnum = (int)pc.size();
		pmc.resize(pcnum);
		for (int i = 0; i < pcnum; ++i) {
			pmc[i] = PointMark(pc.get(i), false);
		}
	}

	void init()
	{
		pmc.clear();
		pmc.shrink_to_fit();
	}

	PointMark get_pm(int i)
	{
		return pmc[i];
	}
	State3D get_pt(int i)
	{
		return pmc[i].getState();
	}
	bool get_mk(int i)
	{
		return pmc[i].getMK();
	}
	void toFalse(int i)
	{
		pmc[i].toFalse();
	}
	void toTrue(int i)
	{
		pmc[i].toTrue();
	}
	bool valid(int i)
	{
		if (pmc[i].getMK())	return true;
		else                return false;
	}
	void set_pm(int i, PointMark pm)
	{
		pmc[i] = pm;
	}
	void push_back(PointMark pm)
	{
		pmc.push_back(pm);
	}
};


class PCMerge 
{
private:
	std::vector<PointCloud> subject;
	int theta_edge;

	bool contain_theta_edge(State3D sub);

public:
	PCMerge(std::vector<PointCloud> _pcs, int symmetry);
	PCMerge();

	void merge();

	bool scan_one();
	bool scan_others(int ind, Point2D sub1);

	void insert(int left, int right);

	std::vector<PointCloud> getter();

};

bool contain_theta_edge(State3D sub, int edge_angle);

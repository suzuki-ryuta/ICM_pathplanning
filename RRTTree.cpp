#include <cassert>
#include <algorithm>
#include <cmath>
#include <limits>

#include "RRTTree.h"

// Definition of Class Neighbor ======================
NeighborList::NeighborList()
	:list()
{
}


NeighborList::NeighborList(std::vector<int> old)
	:list(old)
{
}


int NeighborList::size()
{
	return (int)list.size();
}


void NeighborList::push_back(int index)
{
	assert(index >= -1);
	list.push_back(index);
}


void NeighborList::pop_back()
{
	list.pop_back();
}


int NeighborList::get(int i)
{
	if (i < 0)	i = (int)list.size() + i;
	return list[i];
}

void NeighborList::set(int index, int value)
{
	if (index < 0)	index = (int)list.size() + index;
	list[index] = value;
}

// make the array which starts with goal index and
// ends with start index
std::vector<int> NeighborList::nodenum_order(int end)
{
	std::vector<int> ord;
	int index = end;
	while (1)
	{
		ord.push_back(index);
		index = list[index];
		if (index == -1)	break;
	}

//	std::reverse(ord.begin(), ord.end());

	return ord;
}

// Definition of Class RRTNode ======================
RRTNode::RRTNode()
	:node(), cfree_obj(), cfree_del(), cluster_cache(), cost(0.0), is_valid(true)
{
}


RRTNode::RRTNode(Node _n)
	:node(_n), cfree_obj(), cfree_del(), cluster_cache(), cost(0.0), is_valid(true)
{
}


RRTNode::RRTNode(Node _n, PointCloud pc)
	: node(_n), cfree_obj({pc}), cfree_del(), cluster_cache(), cost(0.0), is_valid(true)
{
}


RRTNode::RRTNode(Node _n, std::vector<PointCloud> pcs, std::vector<PointCloud> del)
	: node(_n), cfree_obj(pcs), cfree_del(del), cluster_cache(), cost(0.0), is_valid(true)
{
}


double RRTNode::distance(const Node& other)
{
	return node.distance(other);
}


double RRTNode::distance(const RRTNode& other)
{
	return node.distance(other.node);
}


Node RRTNode::getNode()
{
	return node;
}

Node RRTNode::getNode() const
{
	return node;
}

PointCloud RRTNode::pc()
{
	assert(cfree_obj.size() == 1);
	return cfree_obj[0];
}


std::vector<PointCloud> RRTNode::get_cfree_obj()
{
	return cfree_obj;
}


std::vector<PointCloud> RRTNode::get_cfree_del()
{
	return cfree_del;
}


// Definition of Class RRTTree ========================
RRTTree::NodeCloud::NodeCloud(const std::vector<RRTNode>& points)
	: pts(points)
{
}

size_t RRTTree::NodeCloud::kdtree_get_point_count() const
{
	return pts.size();
}

inline double RRTTree::NodeCloud::kdtree_get_pt(const size_t idx, const size_t dim) const
{
	return pts[idx].node.node[dim];
}

template <class BBOX>
bool RRTTree::NodeCloud::kdtree_get_bbox(BBOX& /*bb*/) const
{
	return false;
}

RRTTree::RRTTree(Node ini, int origin)
	:graph(), indexed(), ever_indexed(), children(), nl(), cloud(graph), kdtree(std::make_unique<KDTree>(Node::dof, cloud, nanoflann::KDTreeSingleIndexAdaptorParams(10U)))
{
	graph.push_back(RRTNode(ini));
	indexed.push_back(true);
	ever_indexed.push_back(true);
	children.emplace_back();
	nl.push_back(origin);
	if (0 <= origin && origin < (int)children.size()) {
		children[origin].push_back(0);
	}
	kdtree->addPoints(0, 0);
	assert((int)graph.size() == nl.size());
	assert(indexed.size() == graph.size() && ever_indexed.size() == graph.size() && children.size() == graph.size());
}


RRTTree::RRTTree()
	:graph(), indexed(), ever_indexed(), children(), nl(), cloud(graph), kdtree(std::make_unique<KDTree>(Node::dof, cloud, nanoflann::KDTreeSingleIndexAdaptorParams(10U)))
{
}

void RRTTree::rebuild_kdtree()
{
	kdtree = std::make_unique<KDTree>(Node::dof, cloud, nanoflann::KDTreeSingleIndexAdaptorParams(10U));
	for (size_t i = 0; i < graph.size(); ++i) {
		if (!graph[i].is_valid || !indexed[i]) {
			kdtree->removePoint(i);
		}
	}
}


Node RRTTree::format(Node targ)
{
	int oya = get_nearest_index(targ);
	Node fmt = targ.normalize(graph[oya].getNode());
	push_back(fmt, oya);
	return fmt;
}


Node RRTTree::add(int root_index, Node leaf)
{
	Node fmt = leaf.normalize(graph[root_index].getNode());
	push_back(fmt, root_index);
	return fmt;
}


void RRTTree::push_back(Node targ, int oya)
{
	const int idx = (int)graph.size();
	graph.push_back(RRTNode(targ));
	indexed.push_back(false);
	ever_indexed.push_back(false);
	children.emplace_back();
	nl.push_back(oya);
	if (0 <= oya && oya < (int)children.size()) {
		children[oya].push_back(idx);
	}
	assert((int)graph.size() == nl.size());
	assert(indexed.size() == graph.size() && ever_indexed.size() == graph.size() && children.size() == graph.size());
}


void RRTTree::push_back(RRTNode targ, int oya)
{
	const int idx = (int)graph.size();
	graph.push_back(targ);
	indexed.push_back(false);
	ever_indexed.push_back(false);
	children.emplace_back();
	nl.push_back(oya);
	if (0 <= oya && oya < (int)children.size()) {
		children[oya].push_back(idx);
	}
	if (targ.is_valid && kdtree) {
		const size_t kd_idx = graph.size() - 1;
		kdtree->addPoints(kd_idx, kd_idx);
		indexed[kd_idx] = true;
		ever_indexed[kd_idx] = true;
	}
	assert((int)graph.size() == nl.size());
	assert(indexed.size() == graph.size() && ever_indexed.size() == graph.size() && children.size() == graph.size());
}

void RRTTree::replace(RRTNode targ)
{
	assert(!graph.empty());
	const size_t idx = graph.size() - 1;
	const bool was_indexed = indexed[idx];
	const bool point_changed = graph[idx].node.node != targ.node.node;
	graph[idx] = targ;

	if (!graph[idx].is_valid) {
		if (was_indexed && kdtree) {
			kdtree->removePoint(idx);
		}
		indexed[idx] = false;
	} else if (!was_indexed) {
		indexed[idx] = true;
		if (ever_indexed[idx]) {
			rebuild_kdtree();
		} else if (kdtree) {
			kdtree->addPoints(idx, idx);
		}
		ever_indexed[idx] = true;
	} else if (point_changed) {
		rebuild_kdtree();
	}

	assert((int)graph.size() == nl.size());
	assert(indexed.size() == graph.size() && ever_indexed.size() == graph.size() && children.size() == graph.size());
}


void RRTTree::pop_back()
{
	assert(!graph.empty());
	const size_t idx = graph.size() - 1;
	const bool must_rebuild = ever_indexed[idx];
	if (indexed[idx] && kdtree) {
		kdtree->removePoint(idx);
	}
	graph.pop_back();
	indexed.pop_back();
	ever_indexed.pop_back();
	children.pop_back();
	const int parent = nl.get(-1);
	if (0 <= parent && parent < (int)children.size()) {
		auto& siblings = children[parent];
		siblings.erase(std::remove(siblings.begin(), siblings.end(), (int)idx), siblings.end());
	}
	nl.pop_back();
	if (must_rebuild) {
		rebuild_kdtree();
	}
	assert((int)graph.size() == nl.size());
	assert(indexed.size() == graph.size() && ever_indexed.size() == graph.size() && children.size() == graph.size());
}

void RRTTree::invalidate(int index)
{
	if (index < 0) index = size() + index;
	if (index < 0 || index >= (int)graph.size()) return;
	graph[index].is_valid = false;
	if (indexed[index] && kdtree) {
		kdtree->removePoint(static_cast<size_t>(index));
	}
	indexed[index] = false;
}


int RRTTree::size()
{
	assert((int)graph.size() == nl.size());
	assert(indexed.size() == graph.size() && ever_indexed.size() == graph.size() && children.size() == graph.size());
	return (int)graph.size();
}


RRTNode RRTTree::get_nearest_node(Node targ)
{
	int nearest = get_nearest_index(targ);
	return get_RRTNode(nearest);
}


int RRTTree::get_nearest_index(Node targ)
{
	if (graph.empty() || !kdtree) {
		return -1;
	}

	const auto query_once = [this, &targ]() -> int {
		const size_t num_results = 1;
		size_t ret_index = 0;
		double out_dist_sqr = 0.0;
		nanoflann::KNNResultSet<double> resultSet(num_results);
		resultSet.init(&ret_index, &out_dist_sqr);
		const bool found = kdtree->findNeighbors(resultSet, targ.node.data(), nanoflann::SearchParameters());
		if (!found || resultSet.empty()) {
			return -1;
		}
		if (ret_index < graph.size() && graph[ret_index].is_valid && indexed[ret_index]) {
			return static_cast<int>(ret_index);
		}
		return -1;
	};

	int index = query_once();
	if (index >= 0) {
		return index;
	}

	rebuild_kdtree();
	index = query_once();
	assert(index >= 0);
	return index;
}


RRTNode& RRTTree::get_RRTNode(int index)
{
	if (index < 0)	index = size() + index;
	return graph[index];
}

RRTNode RRTTree::back_RRTNode()
{
	return graph[size() - 1];
}


int RRTTree::get_parent_index(int index)
{
	return nl.get(index);
}

std::vector<int> RRTTree::get_children_indices(int parent)
{
	if (parent < 0 || parent >= (int)children.size()) {
		return {};
	}
	return children[parent];
}


RRTNode RRTTree::get_parentRRTNode(int index) {
	int oya = nl.get(index);
	return graph[oya];
}


RRTNode RRTTree::back_parentRRTNode()
{
	int oya = nl.get(-1);
	return graph[oya];
}


int RRTTree::get_now_index(){
	return size() - 1;
}


NodeList RRTTree::generate_path()
{
	NodeList path;
	int end = size() - 1;
	std::vector<int> ord = nl.nodenum_order(end);
	bool started = false;
	for (const auto& e : ord) {
		if(!graph[e].is_valid){
			if (!started) {
				continue;
			} else {
				break;
			}
		}
		started = true;
		path.push_back(graph[e].getNode());
	}

	path.reverse();
	return path;
}

std::vector<int> RRTTree::get_neighbors(int index, double radius)
{
	std::vector<int> neighbors;
	if (index < 0 || index >= (int)graph.size() || graph.empty() || !kdtree || radius <= 0.0 || !graph[index].is_valid) {
		return neighbors;
	}

	std::vector<nanoflann::ResultItem<size_t, double>> matches;
	const double radius_sqr = radius * radius;
	const double search_radius = std::nextafter(radius_sqr, std::numeric_limits<double>::infinity());
	nanoflann::RadiusResultSet<double, size_t> resultSet(search_radius, matches);
	kdtree->findNeighbors(resultSet, graph[index].node.node.data(), nanoflann::SearchParameters());

	neighbors.reserve(matches.size());
	for (const auto& match : matches) {
		const int neighbor_index = static_cast<int>(match.first);
		if (neighbor_index == index) continue;
		if (0 <= neighbor_index && neighbor_index < (int)graph.size() && graph[neighbor_index].is_valid && indexed[neighbor_index]) {
			neighbors.push_back(neighbor_index);
		}
	}
	return neighbors;
}

void RRTTree::set_parent_index(int index, int new_parent)
{
	if (index < 0) index = size() + index;
	if (index < 0 || index >= (int)graph.size()) return;

	const int old_parent = nl.get(index);
	if (old_parent == new_parent) {
		nl.set(index, new_parent);
		return;
	}

	if (0 <= old_parent && old_parent < (int)children.size()) {
		auto& siblings = children[old_parent];
		siblings.erase(std::remove(siblings.begin(), siblings.end(), index), siblings.end());
	}

	nl.set(index, new_parent);

	if (0 <= new_parent && new_parent < (int)children.size()) {
		children[new_parent].push_back(index);
	}
}


NodeList RRTTree::generate_path(int end_index)
{
	NodeList path;
	std::vector<int> ord = nl.nodenum_order(end_index);

	bool started = false;
	for(const auto& e: ord){
		if (!graph[e].is_valid) {
			if (!started) continue;
			break;
		}
		started = true;
		path.push_back(graph[e].getNode());
	}

	path.reverse();
	return path;
}

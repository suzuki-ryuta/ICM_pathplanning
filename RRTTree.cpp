#include <cassert>
#include <algorithm>
#include <iostream>

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
	if(cfree_obj.size() == 0) {
		throw std::runtime_error("RRTNode::pc(): cfree_obj is empty");
	}
	if(cfree_obj.size() != 1) {
		std::cerr << "Warning: RRTNode::pc() called on node with " << cfree_obj.size() 
		          << " cfree_obj entries. Returning first entry." << std::endl;
	}
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
RRTTree::RRTTree(Node ini, int origin)
	:graph(), nl()
{
	graph.push_back(RRTNode(ini));
	nl.push_back(origin);
	assert((int)graph.size() == nl.size());
}


RRTTree::RRTTree()
	:graph(), nl()
{
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
	graph.push_back(RRTNode(targ));
	nl.push_back(oya);
	assert((int)graph.size() == nl.size());
}


void RRTTree::push_back(RRTNode targ, int oya)
{
	graph.push_back(targ);
	nl.push_back(oya);
	assert((int)graph.size() == nl.size());
}

void RRTTree::replace(RRTNode targ)
{
	graph.back() = targ;
	assert((int)graph.size() == nl.size());
}


void RRTTree::pop_back()
{
	assert(!graph.empty());
	graph.pop_back();
	nl.pop_back();
	assert((int)graph.size() == nl.size());
}


int RRTTree::size()
{
	assert((int)graph.size() == nl.size());
	return (int)graph.size();
}


RRTNode RRTTree::get_nearest_node(Node targ)
{
	int nearest = get_nearest_index(targ);
	return get_RRTNode(nearest);
}


int RRTTree::get_nearest_index(Node targ)
{
	double dist = DBL_MAX;
	int index = -1;
	for (int i = 0; i < (int)graph.size(); ++i) {
		if (!graph[i].is_valid) continue;

		double candidate_dist = graph[i].distance(targ);
		if (candidate_dist < dist) {
			dist = candidate_dist;
			index = i;
		}
	}
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
	// Skip leading invalid nodes (these can appear when the last pushed node
	// was later invalidated). Start building the path from the first valid
	// node encountered, then stop at the next invalid node.
	bool started = false;
	for (const auto& e : ord) {
		if (!graph[e].is_valid) {
			if (!started) {
				// still skipping leading invalid nodes
				continue;
			} else {
				std::cerr << "Warning: generate_path stopped at invalid node index " << e << ".\n";
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
	if (index < 0 || index >= (int)graph.size() || radius <= 0.0) {
		return neighbors;
	}

	for (int neighbor_index = 0; neighbor_index < (int)graph.size(); ++neighbor_index) {
		if (neighbor_index == index || !graph[neighbor_index].is_valid) continue;
		if (graph[neighbor_index].distance(graph[index]) <= radius) {
			neighbors.push_back(neighbor_index);
		}
	}
	return neighbors;
}

void RRTTree::set_parent_index(int index, int new_parent)
{
	nl.set(index, new_parent);
}


NodeList RRTTree::generate_path(int end_index)
{
	NodeList path;
	std::vector<int> ord = nl.nodenum_order(end_index);
	bool started2 = false;
	for (const auto& e : ord) {
		if (!graph[e].is_valid) {
			if (!started2) continue;
			std::cerr << "Warning: generate_path(end_index) stopped at invalid node index " << e << ".\n";
			break;
		}
		started2 = true;
		path.push_back(graph[e].getNode());
	}

	path.reverse();
	return path;
}

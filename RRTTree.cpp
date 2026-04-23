#include <cassert>
#include <algorithm>

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
	:node(), cfree_obj(), cfree_del()
{
}


RRTNode::RRTNode(Node _n)
	:node(_n), cfree_obj(), cfree_del()
{
}


RRTNode::RRTNode(Node _n, PointCloud pc)
	: node(_n), cfree_obj({pc}), cfree_del()
{
}


RRTNode::RRTNode(Node _n, std::vector<PointCloud> pcs, std::vector<PointCloud> del)
	: node(_n), cfree_obj(pcs), cfree_del(del)
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
	graph.pop_back();
	graph.push_back(targ);
	assert((int)graph.size() == nl.size());
}


void RRTTree::pop_back()
{
	assert(get_RRTNode(-1).get_cfree_obj().size() == 0);
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
	for (int i = 0; i < (int)(graph.size()); ++i)
	{
		double tmp = graph[i].distance(targ);
		if (dist > tmp) {
			index = i;
			dist = tmp;
		}
	}

	return index;
}


RRTNode RRTTree::get_RRTNode(int index)
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

	for (const auto& e : ord) {
		path.push_back(graph[e].getNode());
	}
	
	path.reverse();
	return path;
}


NodeList RRTTree::generate_path(int end_index)
{
	NodeList path;
	std::vector<int> ord = nl.nodenum_order(end_index);

	for(const auto& e: ord){
		path.push_back(graph[e].getNode());
	}

	path.reverse();
	return path;
}












#include <cmath>
#include <iostream>
#include <iomanip>
#include <cassert>
#include <fstream>

#include "Node.h"


HalfNode::HalfNode(double ang1, double ang2, double ang3)
{
	hnode.resize(3);
	hnode[0] = ang1;
	hnode[1] = ang2;
	hnode[2] = ang3;
}


double HalfNode::get_element(int num)
{
	double angle = 0;
	for (int i = 0; i <= num; ++i) {
		angle += hnode[i];
	}
	return angle;
}




Node::Node()
	:node()
{
	node = { 0, 0, 0, 0, 0, 0 };
}


Node::Node(double e1, double e2, double e3, double e4, double e5, double e6)
	: node()
{
	node.resize(6);node[0] = e1;	node[1] = e2;	node[2] = e3;
	node[3] = e4;	node[4] = e5;	node[5] = e6;
}


Node::Node(std::vector<double> nd)
	: node(nd)
{
	assert(nd.size() == 6);
}


double Node::get_element(int n) 
{
	return node[n];
}
double Node::get_element(int n) const
{
	return node[n];
}

HalfNode Node::get_langles()
{
	HalfNode hn(node[0], node[1], node[2]);
	return hn;
}


HalfNode Node::get_rangles()
{
	HalfNode hn(node[3], node[4], node[5]);
	return hn;
}


double Node::get_absangle(int index) const
{
	if (index == 0)	return node[0];
	if (index == 1)	return node[0] + node[1];
	if (index == 2)	return node[0] + node[1] + node[2];
	if (index == 3)	return node[3];
	if (index == 4) return node[3] + node[4];
	if (index == 5)	return node[3] + node[4] + node[5];
	return 999;
}


double Node::distance(const Node& other)
{
	double dist = 0.0;
	for (int i = 0; i < 6; ++i) {
		dist += (this->node[i] - other.node[i]) * (this->node[i] - other.node[i]);
	}
	return std::sqrt(dist);
}


// otherÇ∆ÇÃãóó£Ç™1Ç…Ç»ÇÈÇÊÇ§Ç…*thisÇìÆÇ©Ç∑ÅCotherÇÕå≈íË
Node Node::normalize(const Node& other)
{
	double dist = distance(other);
	std::vector<double> new_node(6);
	for (int i = 0; i < Node::dof; ++i) {
		double trans = (node[i] - other.node[i]) / dist;
		new_node[i] = other.node[i] + trans;
	}

	return Node(new_node);
}


Node Node::unitmove(const Node& other)
{
	double dist = distance(other);
	if (dist < 1.0)	return *this;

	std::vector<double> new_node(6);
	for (int i = 0; i < Node::dof; ++i) {
		double trans = (other.node[i] - node[i]) / dist;
		new_node[i] = node[i] + trans;
	}

	return new_node;
}


Node Node::operator+(const Node& other)	const
{
	return Node(node[0] + other.node[0],
			    node[1] + other.node[1],
			    node[2] + other.node[2],
			    node[3] + other.node[3],
			    node[4] + other.node[4],
			    node[5] + other.node[5]);
}

Node Node::operator-(const Node& other)	const
{
	return Node(node[0] - other.node[0],
			    node[1] - other.node[1],
			    node[2] - other.node[2],
			    node[3] - other.node[3],
			    node[4] - other.node[4],
			    node[5] - other.node[5]);
}

Node Node::operator*(double r)	const
{
	return Node(r*node[0], r*node[1], r*node[2],
			    r*node[3], r*node[4], r*node[5]);
}

Node operator*(double r, const Node& other)
{
	return Node(r*other.node[0], r*other.node[1], r*other.node[2],
			    r*other.node[3], r*other.node[4], r*other.node[5]);
}

double Node::norm(const Node& other)
{
	double sqr = (node[0] - other.node[0])*(node[0] - other.node[0]) 
				+(node[1] - other.node[1])*(node[1] - other.node[1]) 
				+(node[2] - other.node[2])*(node[2] - other.node[2]) 
				+(node[3] - other.node[3])*(node[3] - other.node[3]) 
				+(node[4] - other.node[4])*(node[4] - other.node[4]) 
				+(node[5] - other.node[5])*(node[5] - other.node[5]);
	return std::sqrt(sqr);
}






NodeList::NodeList()
	:elm()
{}

void NodeList::push_back(Node add_node)
{
	elm.push_back(add_node);
}

void NodeList::printIO()
{
	std::cout << std::fixed;
	for (int i = 0; i < (int)elm.size(); ++i) {
		for (int dof = 0; dof < 6; ++dof) {
			std::cout << std::setprecision(5) << elm[i].get_element(dof) << ", ";
		}
		std::cout << std::endl;
	}
}

void NodeList::print_file(std::string fn)
{
	std::string fp = "../ICM_Log/path/" + fn + ".csv";
	std::ofstream file(fp, std::ios::app);
	for (int i = 0; i < (int)elm.size(); ++i) {
		for (int dof = 0; dof < 6; ++dof) {
			file << elm[i].get_element(dof) << ", ";
		}
		file << std::endl;
	}
	file.close();
}

void NodeList::reverse()
{
	const std::vector<Node> copy(elm);
	const int nlsize = (int)copy.size();

	for (int i = 0; i < nlsize; ++i) {
		elm[i] = copy[nlsize - 1 - i];
	}
}


void NodeList::concat(const NodeList& other)
{
	for (int i = 0; i < other.size(); ++i) {
		elm.push_back(other.get(i));
	}
}

std::ostream& operator<<(std::ostream& out, const Node &nd) 
{ 
	out << "["  
		<< nd.get_element(0) << ", " << nd.get_element(1) << ", " 
	    << nd.get_element(2) << ", " << nd.get_element(3) << ", " 
	    << nd.get_element(4) << ", " << nd.get_element(5) << "]"; 
	return out; 
}

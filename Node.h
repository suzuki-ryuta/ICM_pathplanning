#pragma once

#include <vector>
#include <string>
#include "PointCloud.h" // 


class HalfNode
{
private:
	std::vector<double> hnode;

public:
	const static int dof = 3;
	HalfNode(double ang1, double ang2, double ang3);

	double get_element(int n);
};


struct Node
{
	std::vector<double> node;

	const static int dof = 6;

	Node();
	Node(double e1, double e2, double e3, double e4, double e5, double e6);
	Node(std::vector<double> nd);
	Node interpolate(const Node& other, double t) const;//

	double get_element(int n); 
	double get_element(int n) const;
	HalfNode get_langles();
	HalfNode get_rangles();
	double get_absangle(int index) const;

	double distance(const Node& other);
	Node normalize(const Node& other);		// �����̂�*this
	Node unitmove(const Node& other);	// from *this to other

	double& operator [](int dof) { return node[dof]; }
	Node operator+(const Node& other)	const;
	Node operator-(const Node& other)	const;
	Node operator*(double r)	const;
	friend Node operator*(double r, const Node& other);
	double norm(const Node& other);
	PointCloud getCloud() const;  // 
};

std::ostream& operator<<(std::ostream& out, const Node &nd);

struct NodeList
{
	std::vector<Node> elm;

	NodeList();

	void push_back(Node add_node);
	void reverse();
	void printIO();
	void print_file(std::string fn);

	int size() const { return (int)elm.size(); }
	Node get(int i) const { return elm[i]; }
	Node& operator[](int i)	{ return elm[i]; }

	void concat(const NodeList& other);	// this �̌�� other
};

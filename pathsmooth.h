// #include <string>

// #include "PointCloud.h"
// #include "Node.h"

// pathshortcut.h
// #pragma once

// #include <vector>
// #include <string>

// #include "Node.h"
// #include "PointCloud.h"

// class PathShortcut {
// private:
//     NodeList path;
//     int trials;

//     bool robot_update(const Node& newnode);
//     bool caging_valid(const PointCloud& prev, const Node& now, const Node& next);

// public:
//     PathShortcut(const NodeList& input_path, int num_trials = 100);
//     NodeList shortcut();

//     void set_trials(int t) { trials = t; }
// };







#include <string>

#include "PointCloud.h"
#include "Node.h"

class PathSmooth{
private:
	NodeList orig_path;
	const double alpha = 0.1, beta = 0.5;

	bool robot_update(Node newnode);
	//bool caging_valid(PointCloud prev, Node now, Node aft);
public:
	PathSmooth(NodeList path);

	NodeList smooth();
	bool debug();

};


NodeList csv_to_nodelist(std::string fn);

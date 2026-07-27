#pragma once

#include <climits>
#include <vector>

#include "Node.h"

struct GoalExpansionConfig
{
	double step_mm = 1.0;
	int max_steps = 80;
	double finite_difference_deg = 0.001;
	double max_abs_joint_deg = 140.0;
	double local_goal_x_window_mm = 80.0;
	bool require_closed_cluster = true;
	bool verbose = true;
};

struct GoalLineProbe
{
	bool found = false;
	int count = 0;
	int min_x = INT_MAX;
	int max_x = INT_MIN;
};

struct GoalExpansionResult
{
	bool found = false;
	bool ik_failed = false;
	int steps = 0;
	double expanded_mm = 0.0;
	double score = 1000.0;
	GoalLineProbe goal_line;
	Node node;
	std::vector<Node> trajectory;
	std::vector<double> scores;
	std::vector<GoalLineProbe> goal_line_probes;
};

class GoalExpansionSearch
{
private:
	GoalExpansionConfig config;

	double tip_x(const Node& node, int link_index) const;
	GoalLineProbe probe_goal_line(const Node& node) const;
	bool expand_once(Node& node) const;
	bool expand_hand_once(Node& node, int first_joint, int tip_link, double target_dx_mm) const;
	void clamp_node(Node& node) const;

public:
	GoalExpansionSearch();
	explicit GoalExpansionSearch(const GoalExpansionConfig& cfg);

	GoalExpansionResult search(const Node& start) const;
};

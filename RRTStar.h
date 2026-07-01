
#pragma once

#include <cassert>

#include "RRTTree.h"
#include "Planner.h"
#include "CFree.h"
#include "RRT.h"	// GoalJudgeなどを参照

class RRTStar : public Planner
{
private:
	RRTTree tree;
	std::vector<int> garound;
	std::vector<int> goal_indices;
	CFO* strategy;
	int threshold;
	double cluster_distance_threshold;

	// RRTStar specific parameters
	int max_iterations;
	double gamma;
	double eta;
	int d;
	// double w_x, w_y, w_theta;

	void set_strategy(CFO* cfo);
	bool initialize(Node ini);
	bool config_valid(Node newnode);
	bool dfsconfig_valid(Node newnode);

	Node sampling(Node Rand);
	void add_garound();
	void add_goal_candidate();
	Node format_around(Node rand);

	GoalJudge goal_judge(State3D goal);

	int allowed_drop; // *** 追加 ***

	double calculate_cluster_distance(const PointCloud& old_pc, const PointCloud& new_pc);
	bool cluster_extent_heuristic(const PointCloud& child_region, const PointCloud& parent_region, int eps);
	bool cluster_number_judge(const PointCloud& child_region, const PointCloud& parent_region, int eps);

	// RRTStar methods
	// void load_parameters();
	// double weighted_distance(const Node& q1, const Node& q2);
	std::vector<int> get_neighbors_weighted(int index, double radius);
	bool choose_parent(int q_new_index, const std::vector<int>& q_near);
	bool rewire(int q_new_index, const std::vector<int>& q_near);
	void update_subtree_after_rewire(int index);
	std::vector<int> get_children_indices(int parent);
	void repair_failed_branch(int root_index);
	bool validate_edge(int parent_index, const Node& child_node, RRTNode& out_node, int child_index = -1);
	std::vector<PointCloud> extract_cfree(int parent_index, const Node& child_node);
	int select_min_goal_index();
	std::string make_goal_cost_report(int selected_index);

public:
	RRTStar();
	NodeList plan(Node ini, Node fin, State3D goal);
	bool debug();
};



class RevRRTStar : public Planner
{
private:
	RRTTree tree;
	std::vector<int> garound;
	std::vector<int> goal_indices;
	CFO* strategy;
	double cluster_distance_threshold;
	
	// RRTStar specific parameters
	int max_iterations;
	double gamma;
	double eta;
	int d;
	// double w_x, w_y, w_theta;

	bool initialize(Node fin);

//	bool config_valid(Node newnode);
	bool dfsconfig_valid(Node newnode);
	bool continuous_check(PointCloud prev, PointCloud curr);

	void set_strategy(CFO* cfo);
	Node sampling(Node Rand);
	void add_garound();
	void add_goal_candidate();
	Node format_around(Node rand);

	GoalJudge goal_judge(std::vector<PointCloud> pcs);

	double calculate_cluster_distance(const PointCloud& old_pc, const PointCloud& new_pc);
	bool cluster_extent_heuristic(const PointCloud& child_region, const PointCloud& parent_region, int eps);
	bool cluster_number_judge(const PointCloud& child_region, const PointCloud& parent_region, int eps);
	int allowed_drop; // *** 追加 ***
	
	// RRTStar* methods for reverse direction
	// void load_parameters();
	// double weighted_distance(const Node& q1, const Node& q2);
	std::vector<int> get_neighbors_weighted(int index, double radius);
	bool choose_parent(int q_new_index, const std::vector<int>& q_near);
	bool rewire(int q_new_index, const std::vector<int>& q_near);
	void update_subtree_after_rewire(int index);
	std::vector<int> get_children_indices(int parent);
	void repair_failed_branch(int root_index);
	bool validate_edge(int parent_index, const Node& child_node, RRTNode& out_node, int child_index = -1);
	std::vector<PointCloud> extract_cfree(int parent_index, const Node& child_node);
	int select_min_goal_index();
	std::string make_goal_cost_report(int selected_index);
	
public:
	RevRRTStar();
	NodeList plan(Node ini, Node fin, State3D goal);

};



class RRTStarConnect : public Planner //Plannerの継承
{
private:
	struct GoalCandidate
	{
		GoalJudge flag;
		int s_index;
		int g_index;
		double cost;
	};

	RRTTree s_tree, g_tree;
	std::vector<GoalCandidate> goal_candidates;
	int s_threshold;
	double cluster_distance_threshold;
	CFO* strategy;

	// RRTStar specific parameters
	int max_iterations;
	double gamma;
	double eta;
	int d;
	// double w_x, w_y, w_theta;

	bool initialize(Node ini, Node fin);
	bool sconf_update();
	bool gconf_update();
	GoalJudge sconf_goaljudge(State3D goal, RRTNode bef, RRTNode aft);
	GoalJudge gconf_goaljudge(std::vector<PointCloud> cfo, RRTNode bef, RRTNode aft);

	bool caging_validation_sconf(Node node);
	bool caging_validation_gconf(Node node);

	double calculate_cluster_distance(const PointCloud& old_pc, const PointCloud& new_pc);

	GoalJudge goal_sconf(State3D goal);
	GoalJudge goal_connect(RRTNode bef, RRTNode aft);
	GoalJudge goal_gconf(std::vector<PointCloud> cfo);

	bool cluster_extent_heuristic(const PointCloud& child_region, const PointCloud& parent_region, int eps);
	bool cluster_number_judge(const PointCloud& child_region, const PointCloud& parent_region, int eps);

	NodeList make_path(GoalJudge flag);
	NodeList make_path(GoalJudge flag, int sindex, int gindex);
	bool extend_limit(Node n1, Node n2);

	NodeList path_concat();
	NodeList path_concat(int sindex, int gindex);
	void add_goal_candidate(GoalJudge flag, int sindex, int gindex);
	double goal_candidate_cost(GoalJudge flag, int sindex, int gindex);
	NodeList goal_candidate_path(const GoalCandidate& candidate);
	int select_min_goal_candidate_index();
	std::string make_goal_cost_report(int selected_candidate);

	int allowed_drop; // *** 追加 ***

	// RRTStar methods
	// void load_parameters();
	// double weighted_distance(const Node& q1, const Node& q2);
	std::vector<int> get_neighbors_weighted_s(int index, double radius);
	std::vector<int> get_neighbors_weighted_g(int index, double radius);
	bool choose_parent_s(int q_new_index, const std::vector<int>& q_near);
	bool choose_parent_g(int q_new_index, const std::vector<int>& q_near);
	bool rewire_s(int q_new_index, const std::vector<int>& q_near);
	bool rewire_g(int q_new_index, const std::vector<int>& q_near);
	void update_subtree_after_rewire_s(int index);
	void update_subtree_after_rewire_g(int index);
	std::vector<int> get_children_indices_s(int parent);
	std::vector<int> get_children_indices_g(int parent);
	void repair_failed_branch_s(int root_index);
	void repair_failed_branch_g(int root_index);
	bool validate_edge_s(int parent_index, const Node& child_node, RRTNode& out_node, int child_index = -1);
	bool validate_edge_g(int parent_index, const Node& child_node, RRTNode& out_node, int child_index = -1);
	std::vector<PointCloud> extract_cfree_s(int parent_index, const Node& child_node);
	std::vector<PointCloud> extract_cfree_g(int parent_index, const Node& child_node);


public:
	RRTStarConnect();
	NodeList plan(Node ini, Node fin, State3D goal);
};


void rand_init();
Node generate_newnode();
void print_ICSs(std::vector<PointCloud> pcs);
double calc_dth2(double th, double goalth);
double calc_dist(State3D st, State3D goal);
bool contain_xyth(PointCloud pc, State3D goal);
bool contain_yth(PointCloud pc, State3D goal);
int read_threshold();
bool duplicate_check(PointCloud subject, std::vector<PointCloud> db);

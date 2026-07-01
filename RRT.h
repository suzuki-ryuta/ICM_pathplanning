#pragma once

#include <cassert>

#include "RRTTree.h"
#include "Planner.h"
#include "CFree.h"


enum class GoalJudge{
	NotGoal,		
	MiddleGoal,		// when middle goal is satisfied
	Goal,

	Connect,	// below 3 fields are used for RRTConnect
	SGoal,
	GGoal
};
	

class RRT : public Planner
{
private:
	RRTTree tree;
	std::vector<int> garound;
	CFO* strategy;
	int threshold;
	double cluster_distance_threshold;
	double iou_threshold = 0.7;

	void set_strategy(CFO* cfo);
	bool initialize(Node ini);
	bool config_valid(Node newnode);
	bool dfsconfig_valid(Node newnode);

	Node sampling(Node Rand);
	void add_garound();
	Node format_around(Node rand);

	GoalJudge goal_judge(State3D goal);

	int allowed_drop; // *** 追加 ***

	double calculate_cluster_distance(const PointCloud& old_pc, const PointCloud& new_pc);
	bool cluster_extent_heuristic(const PointCloud& child_region, const PointCloud& parent_region, int eps);
	bool cluster_number_judge(const PointCloud& child_region, const PointCloud& parent_region, int eps);

public:
	RRT();
	NodeList plan(Node ini, Node fin, State3D goal);
	bool debug();
};



class RevRRT : public Planner
{
private:
	RRTTree tree;
	CFO* strategy;
	double cluster_distance_threshold;
	double iou_threshold = 0.7;

	bool initialize(Node fin);

//	bool config_valid(Node newnode);
	bool dfsconfig_valid(Node newnode);
	bool continuous_check(PointCloud prev, PointCloud curr);

	void set_strategy(CFO* cfo);
	Node sampling(Node Rand);

	GoalJudge goal_judge(std::vector<PointCloud> pcs);

	double calculate_cluster_distance(const PointCloud& old_pc, const PointCloud& new_pc);
	bool cluster_extent_heuristic(const PointCloud& child_region, const PointCloud& parent_region, int eps);
	bool cluster_number_judge(const PointCloud& child_region, const PointCloud& parent_region, int eps);
	int allowed_drop; // *** 追加 ***
	
public:
	RevRRT();
	NodeList plan(Node ini, Node fin, State3D goal);

};



class RRTConnect : public Planner //Plannerの継承
{
private:
	RRTTree s_tree, g_tree;
	int s_threshold;
	double cluster_distance_threshold;
	CFO* strategy;

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

	int allowed_drop; // *** 追加 ***


public:
	RRTConnect();
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

#pragma once

#include <climits>
#include <cstddef>
#include <string>
#include <vector>

#include "icmMath.h"
#include "CFreeICS.h"
#include "Controller.h"
#include "Problem.h"
#include "RRT.h"
#include "TaskSet.h"

#define WIDTH 20.0
#define th_max 140.0

double caging_func(Node node);

enum class PsoObjectiveMode
{
	Legacy,
	Fast
};

enum class PsoOptimizerKind
{
	BaselinePSO,
	ConstrictionLBestPSO,
	CLPSO,
	JADEDE,
	HHO,
	HybridJADEPattern,
	CompareAll
};

struct PsoConfig
{
	PsoObjectiveMode objective_mode = PsoObjectiveMode::Legacy;
	PsoOptimizerKind optimizer_kind = PsoOptimizerKind::BaselinePSO;
	int repeat_times = 200;
	int particle_nums = 100;
	int max_evaluations = 0;
	unsigned int seed = 0;
	double diffusion_width = WIDTH;
	bool verbose = true;
	bool progress = true;
	bool log_csv = false;
	bool compare_legacy_fast = false;
	std::string csv_path = "pso_runs.csv";
};

struct PsoRunResult
{
	Node best_node;
	double best_score = 1000.0;
	int evaluations = 0;
	double elapsed_sec = 0.0;
	std::string optimizer_name;
	std::string objective_name;
};

class CSpaceConfig;

class FastCagingObjective
{
private:
	State3D goal;
	Controller* controller;
	CSpaceConfig* conf;
	State3D bottom;
	State3D top;
	Vector3D<int> range;
	int numx;
	int numy;
	int numth;
	std::size_t total_size;
	std::vector<unsigned int> visited_stamp;
	unsigned int current_stamp;
	int evaluations;
	bool progress;

	int coord_to_axis_index(int value, int origin, int step, int count) const;
	std::size_t coord_to_flat_index(int ix, int iy, int ith) const;
	State3D axis_to_state(int ix, int iy, int ith) const;
	bool mark_visited(std::size_t index);
	bool is_free_state(const State3D& state);

	struct ComponentStats
	{
		bool has_free_state = false;
		bool touches_workspace_edge = false;
		bool contains_goal_line = false;
		int min_goal_x = INT_MAX;
		int max_goal_x = INT_MIN;
		double max_ytheta_dist = 0.0;
	};

	ComponentStats explore_component(int seed_ix, int seed_iy, int seed_ith);

public:
	FastCagingObjective(bool _progress = false);

	double evaluate(Node node);
	int get_evaluations() const { return evaluations; }
};

class PSO
{
private:
	PsoConfig config;

public:
	PSO(int _repeat_times, int _particle_nums);
	PSO();

	void set_config(const PsoConfig& cfg);
	PsoRunResult optimize_result(Node ini);

	// Compatibility entry point used by main.cpp.
	std::vector<double> optimize(Node ini);
};

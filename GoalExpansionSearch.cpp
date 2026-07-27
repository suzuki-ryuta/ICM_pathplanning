#include "GoalExpansionSearch.h"

#include <algorithm>
#include <cmath>
#include <iostream>

#include "PSO.h"
#include "Robot.h"
#include "CSpace.h"
#include "icmMath.h"

GoalExpansionSearch::GoalExpansionSearch()
	:config()
{
}

GoalExpansionSearch::GoalExpansionSearch(const GoalExpansionConfig& cfg)
	:config(cfg)
{
}

double GoalExpansionSearch::tip_x(const Node& node, int link_index) const
{
	Robot robot;
	robot.update(node);
	return robot.get_link(link_index).get_top().x;
}

GoalLineProbe GoalExpansionSearch::probe_goal_line(const Node& node) const
{
	GoalLineProbe probe;
	State3D goal = read_goal();
	CSpaceConfig* conf = CSpaceConfig::get_instance();
	Controller* controller = Controller::get_instance();

	controller->robot_update(node);
	if(controller->RintersectR(node)) return probe;
	if(controller->RintersectW(node)) return probe;
	controller->robot_update(node);

	const State3D bottom = conf->getbottom();
	const State3D top = conf->gettop();
	const Vector3D<int> range = conf->getrange();
	const int xmin = static_cast<int>(std::ceil(goal.x - config.local_goal_x_window_mm));
	const int xmax = static_cast<int>(std::floor(goal.x + config.local_goal_x_window_mm));

	if(goal.y < bottom.y || goal.y > top.y) return probe;
	if(range.y == 0 || (goal.y - bottom.y) % range.y != 0) return probe;

	for(int x=bottom.x; x<=top.x; x+=range.x){
		if(x < xmin || x > xmax) continue;
		for(int th=bottom.th; th<=top.th; th+=range.z){
			if(calc_dth2(th, goal.th) != 0.0) continue;
			State3D state(x, goal.y, th);
			controller->shape_update(state);
			if(controller->RintersectS()) continue;
			if(controller->WintersectS()) continue;

			probe.found = true;
			++probe.count;
			if(x < probe.min_x) probe.min_x = x;
			if(x > probe.max_x) probe.max_x = x;
		}
	}

	return probe;
}

void GoalExpansionSearch::clamp_node(Node& node) const
{
	for(int i=0; i<Node::dof; ++i){
		node[i] = std::max(-config.max_abs_joint_deg,
		                   std::min(config.max_abs_joint_deg, node[i]));
	}
}

bool GoalExpansionSearch::expand_hand_once(
	Node& node, int first_joint, int tip_link, double target_dx_mm) const
{
	const double eps_deg = config.finite_difference_deg;
	const double eps_rad = deg_to_rad(eps_deg);
	if(eps_rad <= 0.0) return false;

	double jacobian[3] = {0.0, 0.0, 0.0};
	for(int local=0; local<3; ++local){
		const int joint = first_joint + local;
		Node plus = node;
		Node minus = node;
		plus[joint] += eps_deg;
		minus[joint] -= eps_deg;

		const double x_plus = tip_x(plus, tip_link);
		const double x_minus = tip_x(minus, tip_link);
		jacobian[local] = (x_plus - x_minus) / (2.0 * eps_rad);
	}

	double denom = 0.0;
	for(double value : jacobian){
		denom += value * value;
	}
	if(denom < 1.0e-12) return false;

	for(int local=0; local<3; ++local){
		const double delta_rad = jacobian[local] * target_dx_mm / denom;
		node[first_joint + local] += rad_to_deg(delta_rad);
	}

	clamp_node(node);
	return true;
}

bool GoalExpansionSearch::expand_once(Node& node) const
{
	Node next = node;
	const bool left_ok = expand_hand_once(next, 0, 3, -config.step_mm);
	const bool right_ok = expand_hand_once(next, 3, 7, config.step_mm);
	if(!left_ok || !right_ok) return false;

	node = next;
	return true;
}

GoalExpansionResult GoalExpansionSearch::search(const Node& start) const
{
	GoalExpansionResult result;
	result.node = start;

	FastCagingObjective objective(false);
	Node current = start;

	for(int step=0; step<=config.max_steps; ++step){
		const GoalLineProbe probe = probe_goal_line(current);
		double score = 1000.0;
		if(probe.found && config.require_closed_cluster){
			score = objective.evaluate(current);
		}
		else if(probe.found){
			score = 0.0;
		}
		result.trajectory.push_back(current);
		result.scores.push_back(score);
		result.goal_line_probes.push_back(probe);

		if(config.verbose){
			std::cout << "[GoalExpansion] step=" << step
			          << " expanded_mm=" << step * config.step_mm
			          << " goal_line_free=" << (probe.found ? 1 : 0)
			          << " goal_line_count=" << probe.count;
			if(probe.found){
				std::cout << " goal_line_x=[" << probe.min_x << "," << probe.max_x << "]";
			}
			std::cout
			          << " score=" << score
			          << " node=" << current << std::endl;
		}

		if(probe.found && (!config.require_closed_cluster || score < 1000.0)){
			result.found = true;
			result.steps = step;
			result.expanded_mm = step * config.step_mm;
			result.score = score;
			result.goal_line = probe;
			result.node = current;
			return result;
		}

		if(step == config.max_steps) break;
		if(!expand_once(current)){
			result.ik_failed = true;
			result.steps = step;
			result.expanded_mm = step * config.step_mm;
			result.score = score;
			result.node = current;
			return result;
		}
	}

	result.steps = config.max_steps;
	result.expanded_mm = config.max_steps * config.step_mm;
	if(!result.scores.empty()) result.score = result.scores.back();
	result.node = current;
	return result;
}

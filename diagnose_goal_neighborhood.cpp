#include "PSO.h"
#include "CSpace.h"
#include "Controller.h"
#include "Problem.h"
#include "RRT.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <random>
#include <string>
#include <vector>

namespace
{
const double kPenaltyScore = 1000.0;
const double kThetaLimit = 140.0;

Node default_probe_node()
{
	return Node(
		68.018359,
		-121.291797,
		50.839844,
		0.142578,
		41.1875,
		-89.026953);
}

Node clamp_node(Node node)
{
	for(int i=0; i<Node::dof; ++i){
		if(node[i] < -kThetaLimit) node[i] = -kThetaLimit;
		if(node[i] > kThetaLimit) node[i] = kThetaLimit;
	}
	return node;
}

double max_abs_delta(const Node& a, const Node& b)
{
	double result = 0.0;
	for(int i=0; i<Node::dof; ++i){
		result = std::max(result, std::abs(a.get_element(i) - b.get_element(i)));
	}
	return result;
}

double l2_delta(const Node& a, const Node& b)
{
	double sum = 0.0;
	for(int i=0; i<Node::dof; ++i){
		const double diff = a.get_element(i) - b.get_element(i);
		sum += diff * diff;
	}
	return std::sqrt(sum);
}

std::vector<int> equivalent_goal_theta_indices(const State3D& goal, CSpaceConfig* space)
{
	std::vector<int> indices;
	const int bottom_th = space->getbottom().th;
	const int step = space->getrange().z;
	if(step == 0) return indices;

	for(int ith=0; ith<space->getnumth(); ++ith){
		const int theta = bottom_th + step * ith;
		if(calc_dth2(theta, goal.th) == 0.0){
			indices.push_back(ith);
		}
	}
	return indices;
}

struct GoalLineProbe
{
	bool robot_self_collision = false;
	bool robot_wall_collision = false;
	int goal_line_states = 0;
	int free_goal_line_states = 0;
	int first_free_x = 0;
	int last_free_x = 0;
};

struct EscapeResult
{
	bool found = false;
	bool truncated = false;
	int free_goal_line_states = 0;
	int visited_states = 0;
	State3D start;
	State3D boundary;
	std::vector<State3D> path;
	std::string boundary_name;
};

std::size_t flat_index(int ix, int iy, int ith, int numy, int numth)
{
	return (static_cast<std::size_t>(ix) * numy + iy) * numth + ith;
}

State3D axis_to_state(int ix, int iy, int ith, CSpaceConfig* space)
{
	return State3D(
		space->getbottom().x + space->getrange().x * ix,
		space->getbottom().y + space->getrange().y * iy,
		space->getbottom().th + space->getrange().z * ith);
}

void flat_to_axis(std::size_t index, int numy, int numth, int& ix, int& iy, int& ith)
{
	ix = static_cast<int>(index / (numy * numth));
	const int rem = static_cast<int>(index % (numy * numth));
	iy = rem / numth;
	ith = rem % numth;
}

std::string boundary_name(int ix, int iy, CSpaceConfig* space)
{
	if(ix == 0) return "x_min";
	if(ix == space->getnumx() - 1) return "x_max";
	if(iy == 0) return "y_min";
	if(iy == space->getnumy() - 1) return "y_max";
	return "none";
}

bool is_xy_boundary(int ix, int iy, CSpaceConfig* space)
{
	return ix == 0 || ix == space->getnumx() - 1 ||
	       iy == 0 || iy == space->getnumy() - 1;
}

bool is_free_state(Controller* controller, const State3D& state)
{
	controller->shape_update(state);
	if(controller->RintersectS()) return false;
	if(controller->WintersectS()) return false;
	return true;
}

GoalLineProbe probe_goal_line(Node node, const State3D& goal)
{
	GoalLineProbe probe;
	Controller* controller = Controller::get_instance();
	CSpaceConfig* space = CSpaceConfig::get_instance();

	node = clamp_node(node);
	controller->robot_update(node);
	probe.robot_self_collision = controller->RintersectR(node);
	probe.robot_wall_collision = controller->RintersectW(node);
	if(probe.robot_self_collision || probe.robot_wall_collision){
		return probe;
	}

	const int goal_y = goal.y;
	const int bottom_x = space->getbottom().x;
	const int bottom_y = space->getbottom().y;
	const int step_x = space->getrange().x;
	const int step_y = space->getrange().y;
	if(step_y == 0 || (goal_y - bottom_y) % step_y != 0){
		return probe;
	}
	const int goal_y_index = (goal_y - bottom_y) / step_y;
	if(goal_y_index < 0 || goal_y_index >= space->getnumy()){
		return probe;
	}

	std::vector<int> goal_theta_indices = equivalent_goal_theta_indices(goal, space);
	for(int ix=0; ix<space->getnumx(); ++ix){
		const int x = bottom_x + step_x * ix;
		for(int theta_index : goal_theta_indices){
			const int theta = space->getbottom().th + space->getrange().z * theta_index;
			State3D state(x, goal_y, theta);
			++probe.goal_line_states;
			controller->shape_update(state);
			if(controller->RintersectS()) continue;
			if(controller->WintersectS()) continue;
			if(probe.free_goal_line_states == 0){
				probe.first_free_x = x;
			}
			probe.last_free_x = x;
			++probe.free_goal_line_states;
		}
	}

	return probe;
}

std::vector<State3D> free_goal_line_states(Node node, const State3D& goal)
{
	std::vector<State3D> states;
	Controller* controller = Controller::get_instance();
	CSpaceConfig* space = CSpaceConfig::get_instance();

	node = clamp_node(node);
	controller->robot_update(node);
	if(controller->RintersectR(node)) return states;
	if(controller->RintersectW(node)) return states;

	const int goal_y = goal.y;
	const int bottom_x = space->getbottom().x;
	const int step_x = space->getrange().x;
	std::vector<int> goal_theta_indices = equivalent_goal_theta_indices(goal, space);
	for(int ix=0; ix<space->getnumx(); ++ix){
		const int x = bottom_x + step_x * ix;
		for(int theta_index : goal_theta_indices){
			State3D state = axis_to_state(ix, (goal_y - space->getbottom().y) / space->getrange().y, theta_index, space);
			state.x = x;
			state.y = goal_y;
			if(is_free_state(controller, state)){
				states.push_back(state);
			}
		}
	}

	return states;
}

EscapeResult find_escape_path(Node node, const State3D& goal, int max_visits)
{
	EscapeResult result;
	Controller* controller = Controller::get_instance();
	CSpaceConfig* space = CSpaceConfig::get_instance();
	node = clamp_node(node);
	controller->robot_update(node);
	if(controller->RintersectR(node) || controller->RintersectW(node)){
		return result;
	}

	if(space->getrange().y == 0 || (goal.y - space->getbottom().y) % space->getrange().y != 0){
		return result;
	}
	const int goal_iy = (goal.y - space->getbottom().y) / space->getrange().y;
	if(goal_iy < 0 || goal_iy >= space->getnumy()){
		return result;
	}

	std::vector<int> goal_theta_indices = equivalent_goal_theta_indices(goal, space);
	struct Seed
	{
		int ix;
		int iy;
		int ith;
		State3D state;
	};
	std::vector<Seed> seeds;
	for(int ix=0; ix<space->getnumx(); ++ix){
		for(int ith : goal_theta_indices){
			State3D state = axis_to_state(ix, goal_iy, ith, space);
			if(is_free_state(controller, state)){
				seeds.push_back({ix, goal_iy, ith, state});
			}
		}
	}
	result.free_goal_line_states = static_cast<int>(seeds.size());
	if(seeds.empty()) return result;

	std::sort(seeds.begin(), seeds.end(), [&](const Seed& a, const Seed& b){
		return std::abs(a.state.x - goal.x) < std::abs(b.state.x - goal.x);
	});

	const int numx = space->getnumx();
	const int numy = space->getnumy();
	const int numth = space->getnumth();
	const std::size_t total = static_cast<std::size_t>(numx) * numy * numth;

	for(const Seed& seed : seeds){
		std::vector<int> parent(total, -1);
		std::deque<std::size_t> queue;
		const std::size_t start_index = flat_index(seed.ix, seed.iy, seed.ith, numy, numth);
		parent[start_index] = static_cast<int>(start_index);
		queue.push_back(start_index);
		result.start = seed.state;

		while(!queue.empty()){
			const std::size_t current_index = queue.front();
			queue.pop_front();
			++result.visited_states;

			int ix = 0;
			int iy = 0;
			int ith = 0;
			flat_to_axis(current_index, numy, numth, ix, iy, ith);
			if(is_xy_boundary(ix, iy, space)){
				result.found = true;
				result.boundary = axis_to_state(ix, iy, ith, space);
				result.boundary_name = boundary_name(ix, iy, space);

				std::vector<State3D> reversed;
				std::size_t trace = current_index;
				while(true){
					int tx = 0;
					int ty = 0;
					int tt = 0;
					flat_to_axis(trace, numy, numth, tx, ty, tt);
					reversed.push_back(axis_to_state(tx, ty, tt, space));
					if(parent[trace] == static_cast<int>(trace)) break;
					trace = static_cast<std::size_t>(parent[trace]);
				}
				result.path.assign(reversed.rbegin(), reversed.rend());
				return result;
			}

			if(max_visits > 0 && result.visited_states >= max_visits){
				result.truncated = true;
				return result;
			}

			for(int dx=-1; dx<=1; ++dx){
				for(int dy=-1; dy<=1; ++dy){
					for(int dt=-1; dt<=1; ++dt){
						if(dx == 0 && dy == 0 && dt == 0) continue;
						const int nx = ix + dx;
						const int ny = iy + dy;
						int nth = ith + dt;
						if(nx < 0 || nx >= numx || ny < 0 || ny >= numy) continue;
						if(nth < 0 || nth >= numth){
							if(dx != 0 || dy != 0) continue;
							nth = nth < 0 ? numth - 1 : 0;
						}

						const std::size_t next_index = flat_index(nx, ny, nth, numy, numth);
						if(parent[next_index] != -1) continue;
						State3D next_state = axis_to_state(nx, ny, nth, space);
						if(!is_free_state(controller, next_state)) continue;
						parent[next_index] = static_cast<int>(current_index);
						queue.push_back(next_index);
					}
				}
			}
		}
	}

	return result;
}

void write_states_csv(const std::string& path, const std::vector<State3D>& states)
{
	std::ofstream file(path.c_str());
	file << "index,x,y,th\n";
	for(std::size_t i=0; i<states.size(); ++i){
		file << i << ','
		     << states[i].x << ','
		     << states[i].y << ','
		     << states[i].th << '\n';
	}
}

void write_polyline(std::ofstream& file, const std::string& kind, int id, const std::vector<Point2D>& points)
{
	for(std::size_t i=0; i<points.size(); ++i){
		file << kind << ','
		     << id << ','
		     << i << ','
		     << points[i].x << ','
		     << points[i].y << '\n';
	}
	if(!points.empty()){
		file << kind << ','
		     << id << ','
		     << points.size() << ','
		     << points[0].x << ','
		     << points[0].y << '\n';
	}
}

void export_collision_report(const std::string& prefix, Node node, const State3D& state)
{
	Controller* controller = Controller::get_instance();
	node = clamp_node(node);
	controller->robot_update(node);
	controller->shape_update(state);

	std::ofstream summary((prefix + "_collision_summary.txt").c_str());
	summary << std::fixed << std::setprecision(6);
	summary << "node=";
	for(int i=0; i<Node::dof; ++i){
		if(i > 0) summary << ',';
		summary << node.get_element(i);
	}
	summary << '\n';
	summary << "state=" << state.x << ',' << state.y << ',' << state.th << '\n';
	summary << "robot_self_collision=" << (controller->RintersectR(node) ? 1 : 0) << '\n';
	summary << "robot_wall_collision=" << (controller->RintersectW(node) ? 1 : 0) << '\n';
	summary << "object_robot_collision=" << (controller->RintersectS() ? 1 : 0) << '\n';
	summary << "object_wall_collision=" << (controller->WintersectS() ? 1 : 0) << '\n';
	for(int link=0; link<8; ++link){
		controller->shape_update(state);
		const bool hit = controller->LintersectS(link);
		summary << "link_" << link << "_collision=" << (hit ? 1 : 0) << '\n';
	}

	std::ofstream geom((prefix + "_geometry.csv").c_str());
	geom << "kind,id,vertex,x,y\n";
	for(int link=0; link<8; ++link){
		write_polyline(geom, "link", link, controller->get_robot()->get_link(link).get_square().get_vertices());
	}
	write_polyline(geom, "shape", 0, controller->get_shape()->get_poly().getter());
}

void export_escape(const std::string& prefix, Node node, const State3D& goal, int max_visits)
{
	EscapeResult escape = find_escape_path(node, goal, max_visits);
	std::vector<State3D> goal_line = free_goal_line_states(node, goal);

	write_states_csv(prefix + "_goal_line_free.csv", goal_line);
	write_states_csv(prefix + "_escape_path.csv", escape.path);

	std::ofstream summary((prefix + "_summary.txt").c_str());
	summary << std::fixed << std::setprecision(6);
	summary << "node=";
	for(int i=0; i<Node::dof; ++i){
		if(i > 0) summary << ',';
		summary << node.get_element(i);
	}
	summary << '\n';
	summary << "goal=" << goal.x << ',' << goal.y << ',' << goal.th << '\n';
	summary << "free_goal_line_states=" << escape.free_goal_line_states << '\n';
	summary << "escape_found=" << (escape.found ? 1 : 0) << '\n';
	summary << "truncated=" << (escape.truncated ? 1 : 0) << '\n';
	summary << "visited_states=" << escape.visited_states << '\n';
	summary << "boundary=" << escape.boundary_name << '\n';
	summary << "start=" << escape.start.x << ',' << escape.start.y << ',' << escape.start.th << '\n';
	summary << "boundary_state=" << escape.boundary.x << ',' << escape.boundary.y << ',' << escape.boundary.th << '\n';
	summary << "path_length=" << escape.path.size() << '\n';
}

void export_theta_slices(const std::string& prefix, Node node, const State3D& goal)
{
	Controller* controller = Controller::get_instance();
	CSpaceConfig* space = CSpaceConfig::get_instance();
	node = clamp_node(node);
	controller->robot_update(node);

	std::vector<int> theta_indices = equivalent_goal_theta_indices(goal, space);
	for(int theta_index : theta_indices){
		const int theta = space->getbottom().th + space->getrange().z * theta_index;
		std::ofstream file((prefix + "_slice_th_" + std::to_string(theta) + ".csv").c_str());
		file << "x,y,th,free,boundary,goal_y\n";
		for(int ix=0; ix<space->getnumx(); ++ix){
			for(int iy=0; iy<space->getnumy(); ++iy){
				State3D state = axis_to_state(ix, iy, theta_index, space);
				const bool free = is_free_state(controller, state);
				file << state.x << ','
				     << state.y << ','
				     << state.th << ','
				     << (free ? 1 : 0) << ','
				     << (is_xy_boundary(ix, iy, space) ? 1 : 0) << ','
				     << (state.y == goal.y ? 1 : 0) << '\n';
			}
		}
	}
}

int count_free_goal_line_for_theta(Node node, int theta, const State3D& goal)
{
	Controller* controller = Controller::get_instance();
	CSpaceConfig* space = CSpaceConfig::get_instance();

	node = clamp_node(node);
	controller->robot_update(node);
	if(controller->RintersectR(node)) return 0;
	if(controller->RintersectW(node)) return 0;

	const int goal_y = goal.y;
	const int bottom_x = space->getbottom().x;
	const int step_x = space->getrange().x;
	int free_count = 0;
	for(int ix=0; ix<space->getnumx(); ++ix){
		const int x = bottom_x + step_x * ix;
		State3D state(x, goal_y, theta);
		controller->shape_update(state);
		if(controller->RintersectS()) continue;
		if(controller->WintersectS()) continue;
		++free_count;
	}
	return free_count;
}

std::vector<Node> make_candidates(const Node& center, double radius, int random_samples, std::mt19937& rng)
{
	std::vector<Node> candidates;
	candidates.push_back(center);
	if(radius <= 0.0) return candidates;

	for(int dim=0; dim<Node::dof; ++dim){
		for(int sign=-1; sign<=1; sign += 2){
			Node node = center;
			node[dim] += sign * radius;
			candidates.push_back(clamp_node(node));
		}
	}

	std::uniform_real_distribution<double> dist(-radius, radius);
	for(int sample=0; sample<random_samples; ++sample){
		Node node = center;
		for(int dim=0; dim<Node::dof; ++dim){
			node[dim] += dist(rng);
		}
		candidates.push_back(clamp_node(node));
	}

	return candidates;
}

void print_node_csv(const Node& node)
{
	for(int i=0; i<Node::dof; ++i){
		if(i > 0) std::cout << ';';
		std::cout << node.get_element(i);
	}
}
} // namespace

int main(int argc, char* argv[])
{
	int random_samples = 6;
	bool quick_only = false;
	double max_radius = 40.0;
	int max_escape_visits = 2000000;
	std::string export_prefix;
	std::string export_slices_prefix;
	std::string collision_prefix;
	State3D collision_state;
	bool has_collision_state = false;
	std::vector<double> node_values;

	for(int i=1; i<argc; ++i){
		std::string arg(argv[i]);
		if(arg == "--samples" && i + 1 < argc){
			random_samples = std::max(0, std::atoi(argv[++i]));
		}
		else if(arg == "--quick-only"){
			quick_only = true;
		}
		else if(arg == "--max-radius" && i + 1 < argc){
			max_radius = std::atof(argv[++i]);
		}
		else if(arg == "--export-prefix" && i + 1 < argc){
			export_prefix = argv[++i];
		}
		else if(arg == "--export-slices-prefix" && i + 1 < argc){
			export_slices_prefix = argv[++i];
		}
		else if(arg == "--collision-prefix" && i + 1 < argc){
			collision_prefix = argv[++i];
		}
		else if(arg == "--collision-state" && i + 3 < argc){
			const int x = std::atoi(argv[++i]);
			const int y = std::atoi(argv[++i]);
			const int th = std::atoi(argv[++i]);
			collision_state = State3D(x, y, th);
			has_collision_state = true;
		}
		else if(arg == "--max-escape-visits" && i + 1 < argc){
			max_escape_visits = std::max(1, std::atoi(argv[++i]));
		}
		else{
			node_values.push_back(std::atof(argv[i]));
		}
	}

	Node center = node_values.size() == Node::dof ? Node(node_values) : default_probe_node();
	center = clamp_node(center);
	State3D goal = read_goal();
	CSpaceConfig* space = CSpaceConfig::get_instance();
	std::vector<int> theta_indices = equivalent_goal_theta_indices(goal, space);

	if(!export_prefix.empty()){
		export_escape(export_prefix, center, goal, max_escape_visits);
	}
	if(!export_slices_prefix.empty()){
		export_theta_slices(export_slices_prefix, center, goal);
	}
	if(!collision_prefix.empty()){
		export_collision_report(collision_prefix, center, has_collision_state ? collision_state : goal);
	}
	if(!export_prefix.empty() || !export_slices_prefix.empty() || !collision_prefix.empty()){
		return 0;
	}

	std::cout << std::fixed << std::setprecision(6);
	std::cout << "center,";
	print_node_csv(center);
	std::cout << "\n";
	std::cout << "goal_y," << goal.y << "\n";
	std::cout << "goal_th," << goal.th << "\n";
	std::cout << "equivalent_goal_theta_count," << theta_indices.size() << "\n";
	for(int theta_index : theta_indices){
		const int theta = space->getbottom().th + space->getrange().z * theta_index;
		std::cout << "center_free_goal_line_theta_" << theta << ","
		          << count_free_goal_line_for_theta(center, theta, goal) << "\n";
	}
	std::cout << "random_samples_per_radius," << random_samples << "\n";
	std::cout << "quick_only," << (quick_only ? 1 : 0) << "\n";
	std::cout << "max_radius," << max_radius << "\n";

	FastCagingObjective objective(false);
	std::mt19937 rng(1);
	const std::vector<double> radii = {0.0, 0.5, 1.0, 2.0, 3.0, 5.0, 8.0, 10.0, 15.0, 20.0, 30.0, 40.0};

	std::cout
		<< "radius,candidates,robot_invalid,goal_line_hits,nearest_goal_line_hit_max_abs,"
		<< "max_free_goal_line_states,full_checked,full_valid,best_score,best_max_abs,best_l2,best_node\n";

	for(double radius : radii){
		if(radius > max_radius) continue;
		std::vector<Node> candidates = make_candidates(center, radius, random_samples, rng);
		int robot_invalid = 0;
		int goal_line_hits = 0;
		double nearest_goal_line_hit = std::numeric_limits<double>::infinity();
		int max_free_goal_line_states = 0;
		int full_checked = 0;
		int full_valid = 0;
		double best_score = kPenaltyScore;
		Node best_node = center;
		const int max_full_checks = 20;

		for(const Node& candidate : candidates){
			GoalLineProbe probe = probe_goal_line(candidate, goal);
			if(probe.robot_self_collision || probe.robot_wall_collision){
				++robot_invalid;
				continue;
			}

			if(probe.free_goal_line_states > 0){
				++goal_line_hits;
				nearest_goal_line_hit = std::min(nearest_goal_line_hit, max_abs_delta(candidate, center));
				max_free_goal_line_states = std::max(max_free_goal_line_states, probe.free_goal_line_states);

				if(!quick_only && full_checked < max_full_checks){
					++full_checked;
					const double score = objective.evaluate(candidate);
					if(score < kPenaltyScore){
						++full_valid;
					}
					if(score < best_score){
						best_score = score;
						best_node = candidate;
					}
				}
			}
		}

		std::cout << radius << ','
		          << candidates.size() << ','
		          << robot_invalid << ','
		          << goal_line_hits << ',';
		if(std::isfinite(nearest_goal_line_hit)) std::cout << nearest_goal_line_hit;
		else std::cout << "inf";
		std::cout << ','
		          << max_free_goal_line_states << ','
		          << full_checked << ','
		          << full_valid << ','
		          << best_score << ','
		          << max_abs_delta(best_node, center) << ','
		          << l2_delta(best_node, center) << ',';
		print_node_csv(best_node);
		std::cout << "\n";
	}

	return 0;
}

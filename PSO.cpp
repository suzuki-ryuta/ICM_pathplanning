#include "PSO.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <numeric>
#include <random>
#include <sstream>

#include <boost/optional.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/property_tree/ptree.hpp>

namespace
{
const double kPenaltyScore = 1000.0;
const double kC1Baseline = 0.7;
const double kC2Baseline = 0.2;
const double kInertiaHigh = 1.2;
const double kInertiaLow = 0.3;
const double kConstrictionChi = 0.7298437881283576;
const double kConstrictionC = 2.05;

double clamp_double(double value, double low, double high)
{
	if(value < low) return low;
	if(value > high) return high;
	return value;
}

Node clamp_node(Node node)
{
	for(int i=0; i<Node::dof; ++i){
		node[i] = clamp_double(node[i], -th_max, th_max);
	}
	return node;
}

std::vector<double> node_to_vector(const Node& node)
{
	std::vector<double> values(Node::dof);
	for(int i=0; i<Node::dof; ++i){
		values[i] = node.get_element(i);
	}
	return values;
}

Node vector_to_node(const std::vector<double>& values)
{
	std::vector<double> clipped(Node::dof, 0.0);
	for(int i=0; i<Node::dof; ++i){
		clipped[i] = clamp_double(values[i], -th_max, th_max);
	}
	return Node(clipped);
}

std::string objective_name(PsoObjectiveMode mode)
{
	return mode == PsoObjectiveMode::Fast ? "fast" : "legacy";
}

std::string optimizer_name(PsoOptimizerKind kind)
{
	switch(kind){
	case PsoOptimizerKind::BaselinePSO: return "baseline_pso";
	case PsoOptimizerKind::ConstrictionLBestPSO: return "constriction_lbest_pso";
	case PsoOptimizerKind::CLPSO: return "clpso";
	case PsoOptimizerKind::JADEDE: return "jade_de";
	case PsoOptimizerKind::HHO: return "hho";
	case PsoOptimizerKind::HybridJADEPattern: return "hybrid_jade_pattern";
	case PsoOptimizerKind::CompareAll: return "compare_all";
	}
	return "baseline_pso";
}

PsoObjectiveMode parse_objective(const std::string& text, PsoObjectiveMode fallback)
{
	if(text == "fast" || text == "Fast" || text == "FAST") return PsoObjectiveMode::Fast;
	if(text == "legacy" || text == "Legacy" || text == "LEGACY") return PsoObjectiveMode::Legacy;
	return fallback;
}

PsoOptimizerKind parse_optimizer(const std::string& text, PsoOptimizerKind fallback)
{
	if(text == "baseline_pso" || text == "pso" || text == "PSO") return PsoOptimizerKind::BaselinePSO;
	if(text == "constriction_lbest_pso" || text == "lbest_pso") return PsoOptimizerKind::ConstrictionLBestPSO;
	if(text == "clpso" || text == "CLPSO") return PsoOptimizerKind::CLPSO;
	if(text == "jade_de" || text == "JADE" || text == "jade") return PsoOptimizerKind::JADEDE;
	if(text == "hho" || text == "HHO") return PsoOptimizerKind::HHO;
	if(text == "hybrid_jade_pattern" || text == "hybrid") return PsoOptimizerKind::HybridJADEPattern;
	if(text == "compare_all" || text == "comparison") return PsoOptimizerKind::CompareAll;
	return fallback;
}

PsoConfig load_config_from_ini(PsoConfig defaults)
{
	boost::property_tree::ptree pt;
	try{
		read_ini("config/ProblemDefine.ini", pt);
	}
	catch(...){
		return defaults;
	}

	if(boost::optional<std::string> value = pt.get_optional<std::string>("PSO.objective")){
		defaults.objective_mode = parse_objective(value.get(), defaults.objective_mode);
	}
	if(boost::optional<std::string> value = pt.get_optional<std::string>("PSO.optimizer")){
		defaults.optimizer_kind = parse_optimizer(value.get(), defaults.optimizer_kind);
	}
	if(boost::optional<int> value = pt.get_optional<int>("PSO.repeat_times")){
		defaults.repeat_times = std::max(1, value.get());
	}
	if(boost::optional<int> value = pt.get_optional<int>("PSO.particle_nums")){
		defaults.particle_nums = std::max(4, value.get());
	}
	if(boost::optional<int> value = pt.get_optional<int>("PSO.max_evaluations")){
		defaults.max_evaluations = std::max(0, value.get());
	}
	if(boost::optional<unsigned int> value = pt.get_optional<unsigned int>("PSO.seed")){
		defaults.seed = value.get();
	}
	if(boost::optional<double> value = pt.get_optional<double>("PSO.diffusion_width")){
		defaults.diffusion_width = std::max(0.0, value.get());
	}
	if(boost::optional<int> value = pt.get_optional<int>("PSO.verbose")){
		defaults.verbose = (value.get() != 0);
	}
	if(boost::optional<int> value = pt.get_optional<int>("PSO.progress")){
		defaults.progress = (value.get() != 0);
	}
	if(boost::optional<int> value = pt.get_optional<int>("PSO.log_csv")){
		defaults.log_csv = (value.get() != 0);
	}
	if(boost::optional<int> value = pt.get_optional<int>("PSO.compare_legacy_fast")){
		defaults.compare_legacy_fast = (value.get() != 0);
	}
	if(boost::optional<std::string> value = pt.get_optional<std::string>("PSO.csv_path")){
		defaults.csv_path = value.get();
	}

	return defaults;
}

unsigned int seed_or_clock(unsigned int seed)
{
	if(seed != 0) return seed;
	return static_cast<unsigned int>(
		std::chrono::high_resolution_clock::now().time_since_epoch().count());
}

std::vector<Node> initial_population(Node center, int count, double width, std::mt19937& rng)
{
	std::vector<Node> population;
	population.reserve(count);
	std::uniform_real_distribution<double> dist(-width / 2.0, width / 2.0);
	for(int i=0; i<count; ++i){
		std::vector<double> values(Node::dof);
		for(int d=0; d<Node::dof; ++d){
			values[d] = center.get_element(d) + dist(rng);
		}
		population.push_back(vector_to_node(values));
	}
	return population;
}

bool is_goal_line_state(const State3D& state, const State3D& goal)
{
	return state.y == goal.y && calc_dth2(state.th, goal.th) == 0.0;
}

std::vector<int> equivalent_goal_theta_indices(const State3D& goal, int bottom_th, int th_step, int count)
{
	std::vector<int> indices;
	if(th_step == 0) return indices;

	for(int ith=0; ith<count; ++ith){
		const int theta = bottom_th + th_step * ith;
		if(calc_dth2(theta, goal.th) == 0.0){
			indices.push_back(ith);
		}
	}

	return indices;
}

struct ObjectiveContext
{
	PsoConfig config;
	std::unique_ptr<FastCagingObjective> fast;
	int evaluations = 0;

	explicit ObjectiveContext(const PsoConfig& cfg)
		:config(cfg), fast(), evaluations(0)
	{
		if(config.objective_mode == PsoObjectiveMode::Fast || config.compare_legacy_fast){
			fast.reset(new FastCagingObjective(config.progress));
		}
	}

	bool can_evaluate() const
	{
		return config.max_evaluations <= 0 || evaluations < config.max_evaluations;
	}

	double evaluate(Node node)
	{
		if(!can_evaluate()) return kPenaltyScore;
		node = clamp_node(node);
		++evaluations;
		if(config.progress){
			std::cout << "[PSO eval] " << evaluations << "/" << config.max_evaluations
			          << " objective=" << objective_name(config.objective_mode)
			          << " node=" << node << std::endl;
		}
		double score = kPenaltyScore;
		if(config.compare_legacy_fast){
			const double legacy_score = caging_func(node);
			const double fast_score = fast->evaluate(node);
			if(config.verbose || config.progress){
				std::cout << "[PSO compare] " << node
				          << " legacy=" << legacy_score
				          << " fast=" << fast_score << std::endl;
			}
			score = config.objective_mode == PsoObjectiveMode::Fast ? fast_score : legacy_score;
		}
		else if(config.objective_mode == PsoObjectiveMode::Fast){
			score = fast->evaluate(node);
		}
		else{
			score = caging_func(node);
		}
		if(config.progress){
			std::cout << "[PSO eval] " << evaluations << "/" << config.max_evaluations
			          << " score=" << score << std::endl;
		}
		return score;
	}
};

struct TimedResult
{
	PsoRunResult result;
	std::chrono::steady_clock::time_point start;

	TimedResult(const PsoConfig& cfg)
		:result(), start(std::chrono::steady_clock::now())
	{
		result.optimizer_name = optimizer_name(cfg.optimizer_kind);
		result.objective_name = objective_name(cfg.objective_mode);
	}

	void finish(int evaluations)
	{
		result.evaluations = evaluations;
		const auto end = std::chrono::steady_clock::now();
		result.elapsed_sec = std::chrono::duration<double>(end - start).count();
	}
};

void print_result(const PsoRunResult& result)
{
	std::cout << "[PSO] optimizer=" << result.optimizer_name
	          << " objective=" << result.objective_name
	          << " evaluations=" << result.evaluations
	          << " elapsed_sec=" << std::fixed << std::setprecision(3) << result.elapsed_sec
	          << " best_score=" << result.best_score
	          << " best_node=" << result.best_node
	          << std::endl;
}

void append_result_csv(const PsoConfig& config, const PsoRunResult& result)
{
	if(!config.log_csv) return;

	const bool exists = static_cast<bool>(std::ifstream(config.csv_path.c_str()));
	std::ofstream ofs(config.csv_path.c_str(), std::ios::app);
	if(!ofs) return;
	if(!exists){
		ofs << "optimizer,objective,seed,particles,repeat_times,max_evaluations,evaluations,elapsed_sec,best_score";
		for(int i=0; i<Node::dof; ++i) ofs << ",th" << (i + 1);
		ofs << "\n";
	}
	ofs << result.optimizer_name << ","
	    << result.objective_name << ","
	    << config.seed << ","
	    << config.particle_nums << ","
	    << config.repeat_times << ","
	    << config.max_evaluations << ","
	    << result.evaluations << ","
	    << std::fixed << std::setprecision(6) << result.elapsed_sec << ","
	    << result.best_score;
	for(int i=0; i<Node::dof; ++i){
		ofs << "," << result.best_node.get_element(i);
	}
	ofs << "\n";
}

PsoRunResult run_baseline_pso(Node ini, const PsoConfig& config)
{
	ObjectiveContext objective(config);
	TimedResult timed(config);
	std::mt19937 rng(seed_or_clock(config.seed));
	std::uniform_real_distribution<double> unit(0.0, 1.0);

	std::vector<Node> particles = initial_population(ini, config.particle_nums, config.diffusion_width, rng);
	std::vector<Node> velocity(config.particle_nums);
	std::vector<Node> personal_best = particles;
	std::vector<double> personal_score(config.particle_nums, kPenaltyScore);

	for(int i=0; i<config.particle_nums && objective.can_evaluate(); ++i){
		personal_score[i] = objective.evaluate(personal_best[i]);
		if(config.verbose){
			std::cout << personal_best[i] << " : " << personal_score[i] << std::endl;
		}
	}

	int global_index = 0;
	for(int i=1; i<config.particle_nums; ++i){
		if(personal_score[i] < personal_score[global_index]) global_index = i;
	}
	Node global_best = personal_best[global_index];
	double global_score = personal_score[global_index];

	for(int iter=0; iter<config.repeat_times && objective.can_evaluate(); ++iter){
		const double w = kInertiaHigh - (kInertiaHigh - kInertiaLow) * (iter + 1) / config.repeat_times;
		if(config.verbose) std::cout << iter << ": " << std::endl;
		for(int i=0; i<config.particle_nums && objective.can_evaluate(); ++i){
			const double r1 = unit(rng);
			const double r2 = unit(rng);
			velocity[i] = velocity[i] * w
				+ r1 * kC1Baseline * (personal_best[i] - particles[i])
				+ r2 * kC2Baseline * (global_best - particles[i]);
			particles[i] = clamp_node(particles[i] + velocity[i]);
			const double score = objective.evaluate(particles[i]);
			if(score < personal_score[i]){
				personal_score[i] = score;
				personal_best[i] = particles[i];
				if(score < global_score){
					global_score = score;
					global_best = particles[i];
				}
			}
			if(config.verbose){
				std::cout << i << ":" << personal_best[i] << " -> " << score << std::endl;
			}
		}
		if(config.verbose || config.progress){
			std::cout << "\nglobal best -> " << global_best << " : " << global_score << std::endl << std::endl;
		}
	}

	timed.result.best_node = global_best;
	timed.result.best_score = global_score;
	timed.finish(objective.evaluations);
	return timed.result;
}

PsoRunResult run_constriction_lbest_pso(Node ini, const PsoConfig& config)
{
	ObjectiveContext objective(config);
	TimedResult timed(config);
	std::mt19937 rng(seed_or_clock(config.seed));
	std::uniform_real_distribution<double> unit(0.0, 1.0);

	std::vector<Node> particles = initial_population(ini, config.particle_nums, config.diffusion_width, rng);
	std::vector<Node> velocity(config.particle_nums);
	std::vector<Node> personal_best = particles;
	std::vector<double> personal_score(config.particle_nums, kPenaltyScore);

	for(int i=0; i<config.particle_nums && objective.can_evaluate(); ++i){
		personal_score[i] = objective.evaluate(personal_best[i]);
	}

	int global_index = 0;
	for(int i=1; i<config.particle_nums; ++i){
		if(personal_score[i] < personal_score[global_index]) global_index = i;
	}
	Node global_best = personal_best[global_index];
	double global_score = personal_score[global_index];

	for(int iter=0; iter<config.repeat_times && objective.can_evaluate(); ++iter){
		for(int i=0; i<config.particle_nums && objective.can_evaluate(); ++i){
			int best_neighbor = i;
			for(int offset=-2; offset<=2; ++offset){
				const int idx = (i + offset + config.particle_nums) % config.particle_nums;
				if(personal_score[idx] < personal_score[best_neighbor]) best_neighbor = idx;
			}

			const double r1 = unit(rng);
			const double r2 = unit(rng);
			velocity[i] = kConstrictionChi * (velocity[i]
				+ r1 * kConstrictionC * (personal_best[i] - particles[i])
				+ r2 * kConstrictionC * (personal_best[best_neighbor] - particles[i]));
			particles[i] = clamp_node(particles[i] + velocity[i]);

			const double score = objective.evaluate(particles[i]);
			if(score < personal_score[i]){
				personal_score[i] = score;
				personal_best[i] = particles[i];
				if(score < global_score){
					global_score = score;
					global_best = particles[i];
				}
			}
		}
		if(config.verbose || config.progress){
			std::cout << "[PSO lbest] iter=" << iter
			          << " best=" << global_best << " score=" << global_score << std::endl;
		}
	}

	timed.result.best_node = global_best;
	timed.result.best_score = global_score;
	timed.finish(objective.evaluations);
	return timed.result;
}

std::vector< std::vector<int> > make_clpso_exemplars(
	const std::vector<double>& personal_score,
	std::mt19937& rng)
{
	const int n = static_cast<int>(personal_score.size());
	std::uniform_int_distribution<int> pick(0, n - 1);
	std::vector< std::vector<int> > exemplars(n, std::vector<int>(Node::dof, 0));
	for(int i=0; i<n; ++i){
		bool all_self = true;
		for(int d=0; d<Node::dof; ++d){
			int a = pick(rng);
			int b = pick(rng);
			const int winner = personal_score[a] < personal_score[b] ? a : b;
			exemplars[i][d] = winner;
			if(winner != i) all_self = false;
		}
		if(all_self && n > 1){
			exemplars[i][pick(rng) % Node::dof] = (i + 1) % n;
		}
	}
	return exemplars;
}

PsoRunResult run_clpso(Node ini, const PsoConfig& config)
{
	ObjectiveContext objective(config);
	TimedResult timed(config);
	std::mt19937 rng(seed_or_clock(config.seed));
	std::uniform_real_distribution<double> unit(0.0, 1.0);

	std::vector<Node> particles = initial_population(ini, config.particle_nums, config.diffusion_width, rng);
	std::vector<Node> velocity(config.particle_nums);
	std::vector<Node> personal_best = particles;
	std::vector<double> personal_score(config.particle_nums, kPenaltyScore);
	std::vector<int> stagnation(config.particle_nums, 0);

	for(int i=0; i<config.particle_nums && objective.can_evaluate(); ++i){
		personal_score[i] = objective.evaluate(personal_best[i]);
	}

	std::vector< std::vector<int> > exemplars = make_clpso_exemplars(personal_score, rng);

	int global_index = 0;
	for(int i=1; i<config.particle_nums; ++i){
		if(personal_score[i] < personal_score[global_index]) global_index = i;
	}
	Node global_best = personal_best[global_index];
	double global_score = personal_score[global_index];

	for(int iter=0; iter<config.repeat_times && objective.can_evaluate(); ++iter){
		const double w = 0.9 - 0.5 * iter / std::max(1, config.repeat_times - 1);
		for(int i=0; i<config.particle_nums && objective.can_evaluate(); ++i){
			if(stagnation[i] >= 7){
				exemplars = make_clpso_exemplars(personal_score, rng);
				stagnation[i] = 0;
			}
			std::vector<double> vel = node_to_vector(velocity[i]);
			std::vector<double> pos = node_to_vector(particles[i]);
			for(int d=0; d<Node::dof; ++d){
				const int exemplar = exemplars[i][d];
				vel[d] = w * vel[d] + 1.5 * unit(rng)
					* (personal_best[exemplar].get_element(d) - particles[i].get_element(d));
				pos[d] += vel[d];
			}
			velocity[i] = Node(vel);
			particles[i] = vector_to_node(pos);
			const double score = objective.evaluate(particles[i]);
			if(score < personal_score[i]){
				personal_score[i] = score;
				personal_best[i] = particles[i];
				stagnation[i] = 0;
				if(score < global_score){
					global_score = score;
					global_best = particles[i];
				}
			}
			else{
				++stagnation[i];
			}
		}
		if(config.verbose || config.progress){
			std::cout << "[CLPSO] iter=" << iter
			          << " best=" << global_best << " score=" << global_score << std::endl;
		}
	}

	timed.result.best_node = global_best;
	timed.result.best_score = global_score;
	timed.finish(objective.evaluations);
	return timed.result;
}

int random_index_except(int size, const std::vector<int>& forbidden, std::mt19937& rng)
{
	std::uniform_int_distribution<int> pick(0, size - 1);
	for(int attempt=0; attempt<100; ++attempt){
		const int idx = pick(rng);
		if(std::find(forbidden.begin(), forbidden.end(), idx) == forbidden.end()){
			return idx;
		}
	}
	for(int idx=0; idx<size; ++idx){
		if(std::find(forbidden.begin(), forbidden.end(), idx) == forbidden.end()){
			return idx;
		}
	}
	return 0;
}

Node jade_mutation(
	int i,
	const std::vector<Node>& population,
	const std::vector<double>& scores,
	const std::vector<Node>& archive,
	double f,
	double p,
	std::mt19937& rng)
{
	const int n = static_cast<int>(population.size());
	std::vector<int> order(n);
	std::iota(order.begin(), order.end(), 0);
	std::sort(order.begin(), order.end(), [&](int a, int b){ return scores[a] < scores[b]; });
	const int p_count = std::max(2, static_cast<int>(std::ceil(p * n)));
	std::uniform_int_distribution<int> pick_pbest(0, std::min(n, p_count) - 1);
	const int pbest = order[pick_pbest(rng)];
	const int r1 = random_index_except(n, std::vector<int>{i, pbest}, rng);

	const int union_size = n + static_cast<int>(archive.size());
	const int r2_union = random_index_except(union_size, std::vector<int>{i, pbest, r1}, rng);
	Node r2 = r2_union < n ? population[r2_union] : archive[r2_union - n];

	std::vector<double> values(Node::dof);
	for(int d=0; d<Node::dof; ++d){
		values[d] = population[i].get_element(d)
			+ f * (population[pbest].get_element(d) - population[i].get_element(d))
			+ f * (population[r1].get_element(d) - r2.get_element(d));
	}
	return vector_to_node(values);
}

PsoRunResult run_jade_de(Node ini, const PsoConfig& config)
{
	ObjectiveContext objective(config);
	TimedResult timed(config);
	std::mt19937 rng(seed_or_clock(config.seed));
	std::uniform_real_distribution<double> unit(0.0, 1.0);
	std::normal_distribution<double> normal(0.0, 0.1);
	std::cauchy_distribution<double> cauchy(0.0, 0.1);

	std::vector<Node> population = initial_population(ini, config.particle_nums, config.diffusion_width, rng);
	std::vector<double> scores(config.particle_nums, kPenaltyScore);
	std::vector<Node> archive;
	archive.reserve(config.particle_nums);

	for(int i=0; i<config.particle_nums && objective.can_evaluate(); ++i){
		scores[i] = objective.evaluate(population[i]);
	}

	int best_index = 0;
	for(int i=1; i<config.particle_nums; ++i){
		if(scores[i] < scores[best_index]) best_index = i;
	}

	double mu_f = 0.5;
	double mu_cr = 0.5;
	const double p = 0.1;
	const double learning_rate = 0.1;

	for(int iter=0; iter<config.repeat_times && objective.can_evaluate(); ++iter){
		std::vector<double> success_f;
		std::vector<double> success_cr;
		for(int i=0; i<config.particle_nums && objective.can_evaluate(); ++i){
			double f = mu_f + cauchy(rng);
			int guard = 0;
			while(f <= 0.0 && guard++ < 16) f = mu_f + cauchy(rng);
			f = clamp_double(f, 0.05, 1.0);
			const double cr = clamp_double(mu_cr + normal(rng), 0.0, 1.0);

			Node mutant = jade_mutation(i, population, scores, archive, f, p, rng);
			std::vector<double> trial_values = node_to_vector(population[i]);
			std::uniform_int_distribution<int> pick_dim(0, Node::dof - 1);
			const int forced_dim = pick_dim(rng);
			for(int d=0; d<Node::dof; ++d){
				if(unit(rng) < cr || d == forced_dim){
					trial_values[d] = mutant.get_element(d);
				}
			}
			Node trial = vector_to_node(trial_values);
			const double trial_score = objective.evaluate(trial);
			if(trial_score <= scores[i]){
				archive.push_back(population[i]);
				population[i] = trial;
				scores[i] = trial_score;
				success_f.push_back(f);
				success_cr.push_back(cr);
				if(trial_score < scores[best_index]) best_index = i;
			}
			if(static_cast<int>(archive.size()) > config.particle_nums){
				std::uniform_int_distribution<int> pick_archive(0, static_cast<int>(archive.size()) - 1);
				archive.erase(archive.begin() + pick_archive(rng));
			}
		}

		if(!success_f.empty()){
			double num = 0.0;
			double den = 0.0;
			for(double f : success_f){
				num += f * f;
				den += f;
			}
			const double lehmer_f = den > 0.0 ? num / den : mu_f;
			const double mean_cr = std::accumulate(success_cr.begin(), success_cr.end(), 0.0) / success_cr.size();
			mu_f = (1.0 - learning_rate) * mu_f + learning_rate * lehmer_f;
			mu_cr = (1.0 - learning_rate) * mu_cr + learning_rate * mean_cr;
		}

		if(config.verbose || config.progress){
			std::cout << "[JADE-DE] iter=" << iter
			          << " best=" << population[best_index] << " score=" << scores[best_index]
			          << " muF=" << mu_f << " muCR=" << mu_cr << std::endl;
		}
	}

	timed.result.best_node = population[best_index];
	timed.result.best_score = scores[best_index];
	timed.finish(objective.evaluations);
	return timed.result;
}

double levy_step(std::mt19937& rng)
{
	static const double beta = 1.5;
	static const double sigma = 0.6965745025576968;
	std::normal_distribution<double> normal(0.0, 1.0);
	const double u = normal(rng) * sigma;
	const double v = normal(rng);
	return u / std::pow(std::abs(v) + 1e-12, 1.0 / beta);
}

Node hho_candidate(
	const Node& hawk,
	const Node& rabbit,
	const Node& random_hawk,
	const Node& mean_hawk,
	double escaping_energy,
	double jump_strength,
	double q,
	double r,
	std::mt19937& rng)
{
	std::uniform_real_distribution<double> unit(0.0, 1.0);
	std::vector<double> values(Node::dof);
	if(std::abs(escaping_energy) >= 1.0){
		if(q < 0.5){
			for(int d=0; d<Node::dof; ++d){
				values[d] = random_hawk.get_element(d)
					- unit(rng) * std::abs(random_hawk.get_element(d) - 2.0 * unit(rng) * hawk.get_element(d));
			}
		}
		else{
			for(int d=0; d<Node::dof; ++d){
				const double random_bound = -th_max + unit(rng) * (2.0 * th_max);
				values[d] = rabbit.get_element(d) - mean_hawk.get_element(d)
					- unit(rng) * random_bound;
			}
		}
		return vector_to_node(values);
	}

	if(r >= 0.5 && std::abs(escaping_energy) >= 0.5){
		for(int d=0; d<Node::dof; ++d){
			values[d] = rabbit.get_element(d)
				- escaping_energy * std::abs(jump_strength * rabbit.get_element(d) - hawk.get_element(d));
		}
	}
	else if(r >= 0.5){
		for(int d=0; d<Node::dof; ++d){
			values[d] = rabbit.get_element(d)
				- escaping_energy * std::abs(rabbit.get_element(d) - hawk.get_element(d));
		}
	}
	else{
		for(int d=0; d<Node::dof; ++d){
			const double base = rabbit.get_element(d)
				- escaping_energy * std::abs(jump_strength * rabbit.get_element(d) - hawk.get_element(d));
			values[d] = base + 0.01 * levy_step(rng) * (base - hawk.get_element(d));
		}
	}
	return vector_to_node(values);
}

PsoRunResult run_hho(Node ini, const PsoConfig& config)
{
	ObjectiveContext objective(config);
	TimedResult timed(config);
	std::mt19937 rng(seed_or_clock(config.seed));
	std::uniform_real_distribution<double> unit(0.0, 1.0);

	std::vector<Node> hawks = initial_population(ini, config.particle_nums, config.diffusion_width, rng);
	std::vector<double> scores(config.particle_nums, kPenaltyScore);
	for(int i=0; i<config.particle_nums && objective.can_evaluate(); ++i){
		scores[i] = objective.evaluate(hawks[i]);
	}

	int rabbit = 0;
	for(int i=1; i<config.particle_nums; ++i){
		if(scores[i] < scores[rabbit]) rabbit = i;
	}

	for(int iter=0; iter<config.repeat_times && objective.can_evaluate(); ++iter){
		std::vector<double> mean_values(Node::dof, 0.0);
		for(const Node& hawk : hawks){
			for(int d=0; d<Node::dof; ++d) mean_values[d] += hawk.get_element(d);
		}
		for(int d=0; d<Node::dof; ++d) mean_values[d] /= hawks.size();
		Node mean_hawk(mean_values);

		std::uniform_int_distribution<int> pick_hawk(0, config.particle_nums - 1);
		for(int i=0; i<config.particle_nums && objective.can_evaluate(); ++i){
			const double e0 = 2.0 * unit(rng) - 1.0;
			const double escaping_energy = 2.0 * e0 * (1.0 - static_cast<double>(iter) / config.repeat_times);
			const double jump_strength = 2.0 * (1.0 - unit(rng));
			Node candidate = hho_candidate(
				hawks[i],
				hawks[rabbit],
				hawks[pick_hawk(rng)],
				mean_hawk,
				escaping_energy,
				jump_strength,
				unit(rng),
				unit(rng),
				rng);
			const double candidate_score = objective.evaluate(candidate);
			if(candidate_score < scores[i]){
				hawks[i] = candidate;
				scores[i] = candidate_score;
				if(candidate_score < scores[rabbit]) rabbit = i;
			}
		}
		if(config.verbose || config.progress){
			std::cout << "[HHO] iter=" << iter
			          << " best=" << hawks[rabbit] << " score=" << scores[rabbit] << std::endl;
		}
	}

	timed.result.best_node = hawks[rabbit];
	timed.result.best_score = scores[rabbit];
	timed.finish(objective.evaluations);
	return timed.result;
}

PsoRunResult run_hybrid_jade_pattern(Node ini, const PsoConfig& config)
{
	PsoConfig global_config = config;
	global_config.optimizer_kind = PsoOptimizerKind::JADEDE;
	if(config.max_evaluations > 0){
		global_config.max_evaluations = std::max(config.particle_nums, static_cast<int>(config.max_evaluations * 0.8));
	}

	PsoRunResult global = run_jade_de(ini, global_config);
	ObjectiveContext objective(config);
	TimedResult timed(config);
	objective.evaluations = global.evaluations;

	Node best = global.best_node;
	double best_score = global.best_score;
	double step = std::max(0.1, config.diffusion_width / 4.0);

	while(step >= 0.1 && objective.can_evaluate()){
		if(config.verbose || config.progress){
			std::cout << "[Hybrid pattern] step=" << step
			          << " best=" << best << " score=" << best_score << std::endl;
		}
		bool improved = false;
		for(int d=0; d<Node::dof && objective.can_evaluate(); ++d){
			for(int sign=-1; sign<=1 && objective.can_evaluate(); sign += 2){
				std::vector<double> values = node_to_vector(best);
				values[d] += sign * step;
				Node candidate = vector_to_node(values);
				const double score = objective.evaluate(candidate);
				if(score < best_score){
					best = candidate;
					best_score = score;
					improved = true;
				}
			}
		}
		if(!improved) step *= 0.5;
	}

	timed.result.best_node = best;
	timed.result.best_score = best_score;
	timed.finish(objective.evaluations);
	timed.result.elapsed_sec += global.elapsed_sec;
	return timed.result;
}

PsoRunResult run_selected_optimizer(Node ini, const PsoConfig& config)
{
	switch(config.optimizer_kind){
	case PsoOptimizerKind::BaselinePSO:
		return run_baseline_pso(ini, config);
	case PsoOptimizerKind::ConstrictionLBestPSO:
		return run_constriction_lbest_pso(ini, config);
	case PsoOptimizerKind::CLPSO:
		return run_clpso(ini, config);
	case PsoOptimizerKind::JADEDE:
		return run_jade_de(ini, config);
	case PsoOptimizerKind::HHO:
		return run_hho(ini, config);
	case PsoOptimizerKind::HybridJADEPattern:
		return run_hybrid_jade_pattern(ini, config);
	case PsoOptimizerKind::CompareAll:
		break;
	}
	return run_baseline_pso(ini, config);
}

PsoRunResult run_comparison(Node ini, const PsoConfig& config)
{
	std::vector<PsoOptimizerKind> kinds;
	kinds.push_back(PsoOptimizerKind::BaselinePSO);
	kinds.push_back(PsoOptimizerKind::ConstrictionLBestPSO);
	kinds.push_back(PsoOptimizerKind::CLPSO);
	kinds.push_back(PsoOptimizerKind::JADEDE);
	kinds.push_back(PsoOptimizerKind::HHO);
	kinds.push_back(PsoOptimizerKind::HybridJADEPattern);

	PsoRunResult best;
	best.best_score = std::numeric_limits<double>::max();
	for(std::size_t i=0; i<kinds.size(); ++i){
		PsoConfig child = config;
		child.optimizer_kind = kinds[i];
		child.seed = config.seed == 0 ? 0 : config.seed + static_cast<unsigned int>(1009 * i);
		PsoRunResult result = run_selected_optimizer(ini, child);
		print_result(result);
		append_result_csv(child, result);
		if(result.best_score < best.best_score){
			best = result;
		}
	}
	return best;
}

} // namespace

double caging_func(Node node)
{
	State3D goal = read_goal();
	Controller* controller = Controller::get_instance();
	controller->robot_update(node);
	CFreeICS ics(node);

	if (controller->RintersectR(node)) {
		return 1000;
	}
	if (controller->RintersectW(node)) {
		return 1000;
	}
	
	std::vector<PointCloud> cfics = ics.extract();
	if(cfics.size() == 0)	return 1000;
	std::vector<PointCloud> cfree_objs;
	for(int i=0; i<(int)cfics.size(); ++i){
		for(int j=0; j<cfics[i].size(); ++j){
			if(is_goal_line_state(cfics[i].get(j), goal)){
				cfree_objs.push_back(cfics[i]);
				break;
			}
		}
	}
	if(cfree_objs.size() != 1)	return 1000;
	PointCloud cfree_obj = cfree_objs[0];
	
	std::vector<int> x_list;
	for(int i=0; i<cfree_obj.size(); ++i){
		if(is_goal_line_state(cfree_obj.get(i), goal)){
			x_list.push_back(cfree_obj.get(i).x);
		}
	}
	std::sort(x_list.begin(), x_list.end());
	int len = (int)x_list.size();
	int index = len/2;

	int gx = x_list[index];
	goal.x = gx;

	int xmin = INT_MAX, xmax = INT_MIN;
	double max_dist = DBL_MIN;
	for(int i=0; i<cfree_obj.size(); ++i){
		double dist = calc_dist(cfree_obj.get(i), goal);
		int xtmp = cfree_obj.get(i).x;
        if (xtmp < xmin) xmin = xtmp;
        if (xtmp > xmax) xmax = xtmp;
	    int delta_x = (xmax - xmin) / 2;
	    dist = std::sqrt(dist * dist + delta_x * delta_x);
        if (max_dist < dist)   max_dist = dist;
	}

	return max_dist;
}

FastCagingObjective::FastCagingObjective(bool _progress)
	:goal(read_goal()),
	 controller(Controller::get_instance()),
	 conf(CSpaceConfig::get_instance()),
	 bottom(conf->getbottom()),
	 top(conf->gettop()),
	 range(conf->getrange()),
	 numx(conf->getnumx()),
	 numy(conf->getnumy()),
	 numth(conf->getnumth()),
	 total_size(static_cast<std::size_t>(numx) * numy * numth),
	 visited_stamp(total_size, 0),
	 current_stamp(0),
	 evaluations(0),
	 progress(_progress)
{
}

int FastCagingObjective::coord_to_axis_index(int value, int origin, int step, int count) const
{
	if(step == 0) return -1;
	const int delta = value - origin;
	if(delta % step != 0) return -1;
	const int index = delta / step;
	if(index < 0 || index >= count) return -1;
	return index;
}

std::size_t FastCagingObjective::coord_to_flat_index(int ix, int iy, int ith) const
{
	return (static_cast<std::size_t>(ix) * numy + iy) * numth + ith;
}

State3D FastCagingObjective::axis_to_state(int ix, int iy, int ith) const
{
	return State3D(
		bottom.x + range.x * ix,
		bottom.y + range.y * iy,
		bottom.th + range.z * ith);
}

bool FastCagingObjective::mark_visited(std::size_t index)
{
	if(visited_stamp[index] == current_stamp) return false;
	visited_stamp[index] = current_stamp;
	return true;
}

bool FastCagingObjective::is_free_state(const State3D& state)
{
	controller->shape_update(state);
	if(controller->RintersectS()) return false;
	if(controller->WintersectS()) return false;
	return true;
}

FastCagingObjective::ComponentStats FastCagingObjective::explore_component(
	int seed_ix, int seed_iy, int seed_ith)
{
	ComponentStats stats;
	const std::size_t seed_index = coord_to_flat_index(seed_ix, seed_iy, seed_ith);
	if(!mark_visited(seed_index)) return stats;

	const State3D seed_state = axis_to_state(seed_ix, seed_iy, seed_ith);
	if(!is_free_state(seed_state)) return stats;

	struct AxisIndex
	{
		int x;
		int y;
		int th;
	};

	std::deque<AxisIndex> queue;
	queue.push_back({seed_ix, seed_iy, seed_ith});
	stats.has_free_state = true;
	int explored_free_states = 0;

	while(!queue.empty()){
		const AxisIndex current = queue.front();
		queue.pop_front();
		const State3D state = axis_to_state(current.x, current.y, current.th);
		++explored_free_states;
		if(progress && explored_free_states % 1000000 == 0){
			std::cout << "[Fast objective] component seed_x=" << seed_ix
			          << " explored_free_states=" << explored_free_states
			          << " queue=" << queue.size() << std::endl;
		}

		if(current.x == 0 || current.x == numx - 1 ||
		   current.y == 0 || current.y == numy - 1){
			stats.touches_workspace_edge = true;
		}

		if(is_goal_line_state(state, goal)){
			stats.contains_goal_line = true;
			if(state.x < stats.min_goal_x) stats.min_goal_x = state.x;
			if(state.x > stats.max_goal_x) stats.max_goal_x = state.x;
		}

		const double dist = calc_dist(state, goal);
		if(dist > stats.max_ytheta_dist) stats.max_ytheta_dist = dist;

		for(int dx=-1; dx<=1; ++dx){
			for(int dy=-1; dy<=1; ++dy){
				for(int dt=-1; dt<=1; ++dt){
					if(dx == 0 && dy == 0 && dt == 0) continue;
					const int nx = current.x + dx;
					const int ny = current.y + dy;
					int nth = current.th + dt;
					if(nx < 0 || nx >= numx || ny < 0 || ny >= numy) continue;

					if(nth < 0 || nth >= numth){
						if(dx != 0 || dy != 0) continue;
						nth = nth < 0 ? numth - 1 : 0;
					}

					const std::size_t next_index = coord_to_flat_index(nx, ny, nth);
					if(!mark_visited(next_index)) continue;
					const State3D next_state = axis_to_state(nx, ny, nth);
					if(!is_free_state(next_state)) continue;
					queue.push_back({nx, ny, nth});
				}
			}
		}
	}

	return stats;
}

double FastCagingObjective::evaluate(Node node)
{
	++evaluations;
	if(progress){
		std::cout << "[Fast objective] eval=" << evaluations
		          << " start node=" << node << std::endl;
	}
	controller->robot_update(node);
	if(controller->RintersectR(node)) {
		if(progress) std::cout << "[Fast objective] eval=" << evaluations << " robot-self collision -> 1000" << std::endl;
		return kPenaltyScore;
	}
	if(controller->RintersectW(node)) {
		if(progress) std::cout << "[Fast objective] eval=" << evaluations << " robot-wall collision -> 1000" << std::endl;
		return kPenaltyScore;
	}
	controller->robot_update(node);

	const int goal_y_index = coord_to_axis_index(goal.y, bottom.y, range.y, numy);
	const int goal_th_index = coord_to_axis_index(goal.th, bottom.th, range.z, numth);
	std::vector<int> goal_th_indices = equivalent_goal_theta_indices(goal, bottom.th, range.z, numth);
	if(goal_y_index < 0 || (goal_th_index < 0 && goal_th_indices.empty())) {
		if(progress) std::cout << "[Fast objective] eval=" << evaluations << " goal is outside C-space grid -> 1000" << std::endl;
		return kPenaltyScore;
	}

	++current_stamp;
	if(current_stamp == 0){
		std::fill(visited_stamp.begin(), visited_stamp.end(), 0);
		++current_stamp;
	}

	int valid_components = 0;
	double valid_score = kPenaltyScore;
	const int x_progress_interval = std::max(1, numx / 10);
	for(int ix=0; ix<numx; ++ix){
		if(progress && ix > 0 && ix % x_progress_interval == 0){
			std::cout << "[Fast objective] eval=" << evaluations
			          << " seed_x=" << ix << "/" << numx
			          << " valid_components=" << valid_components << std::endl;
		}
		for(int goal_theta_index : goal_th_indices){
			const std::size_t seed_index = coord_to_flat_index(ix, goal_y_index, goal_theta_index);
			if(visited_stamp[seed_index] == current_stamp) continue;
			ComponentStats stats = explore_component(ix, goal_y_index, goal_theta_index);
			if(!stats.has_free_state || !stats.contains_goal_line) continue;
			if(stats.touches_workspace_edge) continue;
			if(stats.min_goal_x == INT_MAX || stats.max_goal_x == INT_MIN) continue;

			++valid_components;
			const int delta_x = (stats.max_goal_x - stats.min_goal_x) / 2;
			valid_score = std::sqrt(stats.max_ytheta_dist * stats.max_ytheta_dist
				+ static_cast<double>(delta_x * delta_x));
			if(progress){
				std::cout << "[Fast objective] eval=" << evaluations
				          << " valid_component=" << valid_components
				          << " score=" << valid_score << std::endl;
			}
			if(valid_components > 1) {
				if(progress) std::cout << "[Fast objective] eval=" << evaluations << " multiple valid components -> 1000" << std::endl;
				return kPenaltyScore;
			}
		}
	}

	if(valid_components != 1) {
		if(progress) std::cout << "[Fast objective] eval=" << evaluations << " valid_components=" << valid_components << " -> 1000" << std::endl;
		return kPenaltyScore;
	}
	if(progress){
		std::cout << "[Fast objective] eval=" << evaluations
		          << " done score=" << valid_score << std::endl;
	}
	return valid_score;
}

PSO::PSO(int _repeat_times, int _particle_nums)
	:config()
{
	config.repeat_times = _repeat_times;
	config.particle_nums = _particle_nums;
}

PSO::PSO()
	:config()
{
}

void PSO::set_config(const PsoConfig& cfg)
{
	config = cfg;
}

PsoRunResult PSO::optimize_result(Node ini)
{
	PsoConfig effective = load_config_from_ini(config);
	if(effective.max_evaluations == 0){
		effective.max_evaluations = effective.particle_nums * (effective.repeat_times + 1);
	}

	if(effective.verbose || effective.progress){
		std::cout << "[PSO] optimizer=" << optimizer_name(effective.optimizer_kind)
		          << " objective=" << objective_name(effective.objective_mode)
		          << " particles=" << effective.particle_nums
		          << " repeat_times=" << effective.repeat_times
		          << " max_evaluations=" << effective.max_evaluations
		          << " seed=" << effective.seed
		          << " verbose=" << effective.verbose
		          << " progress=" << effective.progress
		          << std::endl;
	}

	PsoRunResult result;
	if(effective.optimizer_kind == PsoOptimizerKind::CompareAll){
		result = run_comparison(ini, effective);
	}
	else{
		result = run_selected_optimizer(ini, effective);
		print_result(result);
		append_result_csv(effective, result);
	}
	return result;
}

std::vector<double> PSO::optimize(Node ini)
{
	PsoRunResult result = optimize_result(ini);
	return node_to_vector(result.best_node);
}

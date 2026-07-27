#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include "CFree.h"
#include "CFreeICS.h"
#include "Controller.h"
#include "Node.h"
#include "PointCloud.h"

namespace {

struct Args
{
	std::string csv_path = "../ICM_Log/path/Rev検証用.csv";
	int start_cluster = -1;
	int goal_cluster = -1;
	std::string detail_csv = "../ICM_Log/path/Rev検証用_cluster_check.csv";
	bool ics_only = false;
	bool forward_only = false;
	bool reverse_only = false;
	bool enforce_continuous = true;
};

struct ForwardStep
{
	int edge = -1;
	int result_count = 0;
	std::vector<int> result_sizes;
};

struct ForwardTrace
{
	int start_cluster = -1;
	bool valid = false;
	int first_bad_edge = -1;
	std::vector<PointCloud> active;
	std::vector<ForwardStep> steps;
};

struct ReverseStep
{
	int edge = -1;
	int raw_count = 0;
	int kept_count = 0;
	int rejected_count = 0;
};

struct ReverseTrace
{
	int goal_cluster = -1;
	bool valid = false;
	int first_bad_edge = -1;
	std::vector<std::vector<PointCloud>> candidates;
	std::vector<ReverseStep> steps;
};

std::string join_sizes(const std::vector<PointCloud>& clouds)
{
	std::ostringstream oss;
	for(int i = 0; i < (int)clouds.size(); ++i){
		if(i > 0) oss << '|';
		oss << clouds[i].size();
	}
	return oss.str();
}

std::string join_sizes(const std::vector<int>& sizes)
{
	std::ostringstream oss;
	for(int i = 0; i < (int)sizes.size(); ++i){
		if(i > 0) oss << '|';
		oss << sizes[i];
	}
	return oss.str();
}

bool parse_int_value(const std::string& value, int& out)
{
	char* end = nullptr;
	long parsed = std::strtol(value.c_str(), &end, 10);
	if(end == value.c_str() || *end != '\0') return false;
	if(parsed < std::numeric_limits<int>::min() || parsed > std::numeric_limits<int>::max()) return false;
	out = (int)parsed;
	return true;
}

Args parse_args(int argc, char** argv)
{
	Args args;
	for(int i = 1; i < argc; ++i){
		std::string arg = argv[i];
		if(arg == "--start-cluster" && i + 1 < argc){
			std::string value = argv[++i];
			if(value == "all") args.start_cluster = -1;
			else if(!parse_int_value(value, args.start_cluster)){
				throw std::runtime_error("invalid --start-cluster value: " + value);
			}
		}
		else if(arg == "--goal-cluster" && i + 1 < argc){
			std::string value = argv[++i];
			if(value == "all") args.goal_cluster = -1;
			else if(!parse_int_value(value, args.goal_cluster)){
				throw std::runtime_error("invalid --goal-cluster value: " + value);
			}
		}
		else if(arg == "--detail-csv" && i + 1 < argc){
			args.detail_csv = argv[++i];
		}
		else if(arg == "--ics-only"){
			args.ics_only = true;
		}
		else if(arg == "--forward-only"){
			args.forward_only = true;
		}
		else if(arg == "--reverse-only"){
			args.reverse_only = true;
		}
		else if(arg == "--no-continuous-check"){
			args.enforce_continuous = false;
		}
		else if(arg == "--help" || arg == "-h"){
			std::cout
				<< "Usage: ./VerifyRevClusterTransition [path.csv]\n"
				<< "       [--start-cluster N|all] [--goal-cluster N|all]\n"
				<< "       [--detail-csv output.csv]\n"
				<< "       [--ics-only] [--forward-only] [--reverse-only]\n"
				<< "       [--no-continuous-check]\n";
			std::exit(EXIT_SUCCESS);
		}
		else if(arg.rfind("--", 0) == 0){
			throw std::runtime_error("unknown option: " + arg);
		}
		else{
			args.csv_path = arg;
		}
	}
	return args;
}

std::vector<Node> read_path_csv(const std::string& path)
{
	std::ifstream file(path);
	if(!file.is_open()){
		throw std::runtime_error("cannot open CSV: " + path);
	}

	std::vector<Node> nodes;
	std::string line;
	int line_no = 0;
	while(std::getline(file, line)){
		++line_no;
		if(line.empty()) continue;
		std::replace(line.begin(), line.end(), ',', ' ');
		std::istringstream iss(line);
		std::vector<double> values;
		double value = 0.0;
		while(iss >> value){
			values.push_back(value);
		}
		if(values.empty()) continue;
		if(values.size() != Node::dof){
			std::ostringstream oss;
			oss << "line " << line_no << " has " << values.size()
			    << " values, expected " << Node::dof;
			throw std::runtime_error(oss.str());
		}
		nodes.emplace_back(values);
	}

	if(nodes.size() < 2){
		throw std::runtime_error("path must contain at least two nodes");
	}
	return nodes;
}

std::vector<int> selected_indices(int selected, int count)
{
	if(selected >= 0){
		if(selected >= count){
			std::ostringstream oss;
			oss << "cluster index " << selected << " is out of range 0.." << (count - 1);
			throw std::runtime_error(oss.str());
		}
		return {selected};
	}

	std::vector<int> result;
	for(int i = 0; i < count; ++i){
		result.push_back(i);
	}
	return result;
}

bool has_overlap(const PointCloud& target, const std::vector<PointCloud>& db)
{
	for(const auto& existing: db){
		if(existing.overlap(target)) return true;
	}
	return false;
}

bool continuous_check(const PointCloud& prev, const PointCloud& curr)
{
	if(prev.empty() || curr.empty()) return false;

	double rate = 0.0;
	if(prev.size() < curr.size()){
		rate = (double)curr.size() / (double)prev.size();
	}
	else{
		rate = (double)prev.size() / (double)curr.size();
	}
	return rate < 3.0;
}

std::vector<PointCloud> extract_unique_from_set(const std::vector<PointCloud>& previous, const Node& node)
{
	DfsCFO dfs;
	std::vector<PointCloud> result;
	for(const auto& prev: previous){
		std::vector<PointCloud> extracted = dfs.extract(prev, node);
		for(const auto& cluster: extracted){
			if(has_overlap(cluster, result)) continue;
			result.push_back(cluster);
		}
	}
	return result;
}

ForwardTrace build_forward_trace(const std::vector<Node>& nodes, const PointCloud& initial, int start_cluster)
{
	ForwardTrace trace;
	trace.start_cluster = start_cluster;
	trace.active.reserve(nodes.size());
	trace.active.push_back(initial);
	trace.valid = true;

	for(int i = 1; i < (int)nodes.size(); ++i){
		DfsCFO dfs;
		std::vector<PointCloud> extracted = dfs.extract(trace.active.back(), nodes[i]);

		ForwardStep step;
		step.edge = i - 1;
		step.result_count = (int)extracted.size();
		for(const auto& cluster: extracted){
			step.result_sizes.push_back(cluster.size());
		}
		trace.steps.push_back(step);

		if(extracted.size() != 1){
			trace.valid = false;
			trace.first_bad_edge = i - 1;
			break;
		}
		trace.active.push_back(extracted[0]);
	}

	return trace;
}

ReverseTrace build_reverse_trace(
	const std::vector<Node>& nodes,
	const PointCloud& goal_cluster,
	int goal_cluster_index,
	bool enforce_continuous)
{
	ReverseTrace trace;
	trace.goal_cluster = goal_cluster_index;
	trace.candidates.resize(nodes.size());
	trace.candidates.back().push_back(goal_cluster);
	trace.valid = true;

	Controller* controller = Controller::get_instance();

	for(int i = (int)nodes.size() - 2; i >= 0; --i){
		std::vector<PointCloud> raw = extract_unique_from_set(trace.candidates[i + 1], nodes[i]);
		std::vector<PointCloud> kept;
		std::vector<PointCloud> rejected;

		controller->robot_update(nodes[i + 1]);
		for(const auto& candidate: raw){
			DfsCFO dfs;
			std::vector<PointCloud> back = dfs.extract(candidate, nodes[i + 1]);
			if(back.size() == 1 && (!enforce_continuous || continuous_check(back[0], candidate))){
				kept.push_back(candidate);
			}
			else{
				rejected.push_back(candidate);
			}
		}
		controller->robot_update(nodes[i]);

		trace.candidates[i] = kept;

		ReverseStep step;
		step.edge = i;
		step.raw_count = (int)raw.size();
		step.kept_count = (int)kept.size();
		step.rejected_count = (int)rejected.size();
		trace.steps.push_back(step);

		if(kept.empty()){
			trace.valid = false;
			trace.first_bad_edge = i;
			break;
		}
	}

	std::reverse(trace.steps.begin(), trace.steps.end());
	return trace;
}

int overlap_count(const PointCloud& cluster, const std::vector<PointCloud>& candidates)
{
	int count = 0;
	for(const auto& candidate: candidates){
		if(cluster.overlap(candidate)) ++count;
	}
	return count;
}

void write_detail_csv(
	const std::string& path,
	const std::vector<ForwardTrace>& forward_traces,
	const std::vector<ReverseTrace>& reverse_traces)
{
	std::ofstream file(path);
	if(!file.is_open()){
		std::cerr << "[WARN] cannot write detail CSV: " << path << std::endl;
		return;
	}

	file << "kind,start_cluster,goal_cluster,node_index,edge_index,"
	     << "forward_size,reverse_candidate_count,reverse_candidate_sizes,"
	     << "overlap_count,result_count,result_sizes\n";

	for(const auto& fwd: forward_traces){
		for(int i = 0; i < (int)fwd.steps.size(); ++i){
			const ForwardStep& step = fwd.steps[i];
			file << "forward," << fwd.start_cluster << ",," << (i + 1) << ','
			     << step.edge << ",,,," << ',' << step.result_count
			     << ',' << join_sizes(step.result_sizes) << '\n';
		}
	}

	for(const auto& rev: reverse_traces){
		for(int i = 0; i < (int)rev.candidates.size(); ++i){
			file << "reverse,," << rev.goal_cluster << ',' << i << ",,"
			     << ',' << rev.candidates[i].size() << ','
			     << join_sizes(rev.candidates[i]) << ",,,\n";
		}
	}

	for(const auto& fwd: forward_traces){
		if(!fwd.valid) continue;
		for(const auto& rev: reverse_traces){
			if(!rev.valid) continue;
			for(int i = 0; i < (int)fwd.active.size(); ++i){
				int count = overlap_count(fwd.active[i], rev.candidates[i]);
				file << "overlap," << fwd.start_cluster << ',' << rev.goal_cluster
				     << ',' << i << ",," << fwd.active[i].size() << ','
				     << rev.candidates[i].size() << ','
				     << join_sizes(rev.candidates[i]) << ','
				     << count << ",,\n";
			}
		}
	}
}

void print_ics_summary(const std::string& label, const std::vector<PointCloud>& clusters)
{
	std::cout << label << " clusters: " << clusters.size() << '\n';
	for(int i = 0; i < (int)clusters.size(); ++i){
		std::cout << "  " << label << "[" << i << "] size=" << clusters[i].size() << '\n';
	}
}

} // namespace

int main(int argc, char** argv)
{
	try{
		Args args = parse_args(argc, argv);
		std::vector<Node> nodes = read_path_csv(args.csv_path);

		std::cout << "[INFO] path: " << args.csv_path << '\n';
		std::cout << "[INFO] nodes: " << nodes.size() << '\n';
		std::cout << "[INFO] first node: " << nodes.front() << '\n';
		std::cout << "[INFO] last node : " << nodes.back() << '\n';

		if(!Controller::get_instance()->RintersectR(nodes.front())){
			std::cout << "[INFO] first node robot self-intersection: no\n";
		}

		CFreeICS start_ics(nodes.front());
		std::vector<PointCloud> start_clusters = start_ics.extract();
		CFreeICS goal_ics(nodes.back());
		std::vector<PointCloud> goal_clusters = goal_ics.extract();

		print_ics_summary("start", start_clusters);
		print_ics_summary("goal", goal_clusters);
		std::cout << std::flush;

		if(args.ics_only){
			return EXIT_SUCCESS;
		}

		std::vector<int> start_indices = selected_indices(args.start_cluster, (int)start_clusters.size());
		std::vector<int> goal_indices = selected_indices(args.goal_cluster, (int)goal_clusters.size());

		std::vector<ForwardTrace> forward_traces;
		if(!args.reverse_only){
			for(int start_index: start_indices){
				forward_traces.push_back(build_forward_trace(nodes, start_clusters[start_index], start_index));
			}
		}

		std::vector<ReverseTrace> reverse_traces;
		if(!args.forward_only){
			for(int goal_index: goal_indices){
				reverse_traces.push_back(build_reverse_trace(
					nodes, goal_clusters[goal_index], goal_index, args.enforce_continuous));
			}
		}

		std::cout << "\n[FORWARD]\n";
		for(const auto& trace: forward_traces){
			std::cout << "  start_cluster=" << trace.start_cluster
			          << " status=" << (trace.valid ? "OK" : "BAD")
			          << " reached_nodes=" << trace.active.size();
			if(!trace.valid){
				const ForwardStep& bad = trace.steps.back();
				std::cout << " first_bad_edge=" << trace.first_bad_edge
				          << " result_count=" << bad.result_count
				          << " result_sizes=" << join_sizes(bad.result_sizes);
			}
			std::cout << '\n';
		}

		std::cout << "\n[REVERSE]\n";
		for(const auto& trace: reverse_traces){
			int multi_nodes = 0;
			int max_candidates = 0;
			for(const auto& candidates: trace.candidates){
				if((int)candidates.size() > 1) ++multi_nodes;
				max_candidates = std::max(max_candidates, (int)candidates.size());
			}
			std::cout << "  goal_cluster=" << trace.goal_cluster
			          << " status=" << (trace.valid ? "OK" : "BAD")
			          << " multi_candidate_nodes=" << multi_nodes
			          << " max_candidates=" << max_candidates;
			if(!trace.valid){
				std::cout << " first_bad_edge=" << trace.first_bad_edge;
			}
			std::cout << '\n';
		}

		if(!args.forward_only && !args.reverse_only){
			std::cout << "\n[OVERLAP]\n";
			for(const auto& fwd: forward_traces){
				if(!fwd.valid) continue;
				for(const auto& rev: reverse_traces){
					if(!rev.valid) continue;
					int zero = 0;
					int one = 0;
					int multi = 0;
					int first_zero = -1;
					int first_multi = -1;
					for(int i = 0; i < (int)nodes.size(); ++i){
						int count = overlap_count(fwd.active[i], rev.candidates[i]);
						if(count == 0){
							if(first_zero < 0) first_zero = i;
							++zero;
						}
						else if(count == 1){
							++one;
						}
						else{
							if(first_multi < 0) first_multi = i;
							++multi;
						}
					}

					std::cout << "  start_cluster=" << fwd.start_cluster
					          << " goal_cluster=" << rev.goal_cluster
					          << " exact_one_nodes=" << one << '/' << nodes.size()
					          << " zero_nodes=" << zero
					          << " multi_nodes=" << multi;
					if(first_zero >= 0) std::cout << " first_zero_node=" << first_zero;
					if(first_multi >= 0) std::cout << " first_multi_node=" << first_multi;
					std::cout << '\n';
				}
			}
		}

		write_detail_csv(args.detail_csv, forward_traces, reverse_traces);
		std::cout << "\n[INFO] detail CSV: " << args.detail_csv << '\n';
		return EXIT_SUCCESS;
	}
	catch(const std::exception& e){
		std::cerr << "[ERROR] " << e.what() << std::endl;
		return EXIT_FAILURE;
	}
}

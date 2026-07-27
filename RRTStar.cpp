#include<algorithm>
#include<array>
#include<iostream>
#include<chrono>
#include<climits>
#include<cmath>
#include<fstream>
#include<iomanip>
#include<limits>
#include<sstream>

#include "RRTStar.h"
#include "CFreeICS.h"
#include "Robot.h"
#include "CFree.h"
#include "pathsmooth.h"

extern int origin;
extern std::string g_path_cost_report;
using namespace std::chrono;

namespace {

struct RRTStarCostConfig
{
	double omega;
	double time_loss;
	std::array<double, Node::dof> weights;
};

const RRTStarCostConfig& rrtstar_cost_config()
{
	static const RRTStarCostConfig config = []{
		bp::ptree pt_prob;
		read_ini("config/ProblemDefine.ini", pt_prob);
		return RRTStarCostConfig{
			pt_prob.get<double>("RRTStar.omega"),
			pt_prob.get<double>("RRTStar.Timeloss"),
			// {2.0, 1.0, 1.0, 2.0, 1.0, 1.0}
			{1.0, 1.0, 1.0, 1.0, 1.0, 1.0}
		};
	}();
	return config;
}

bool rrtstar_verbose()
{
	static const bool verbose = []{
		bp::ptree pt_prob;
		try{
			read_ini("config/ProblemDefine.ini", pt_prob);
			return pt_prob.get<int>("RRTStar.verbose", 0) != 0;
		}
		catch(const boost::property_tree::ptree_error&){
			return false;
		}
	}();
	return verbose;
}

bool same_pointcloud(const PointCloud& lhs, const PointCloud& rhs)
{
	if(lhs.size() != rhs.size()) return false;

	for(int i = 0; i < lhs.size(); ++i){
		if(!rhs.exist(lhs.get(i))) return false;
	}

	return true;
}

void load_parameters(int& max_iterations, double& eta, double& gamma, int& d)
{
	try{
    bp::ptree pt_prob, pt_space;
    read_ini("config/ProblemDefine.ini", pt_prob);
    // read_ini("config/SpaceConfig.ini", pt_space);

    // Read max_iterations
    max_iterations = pt_prob.get<int>("RRTStar.max_iterations");
    eta = pt_prob.get<double>("RRTStar.eta");
	d = pt_prob.get<int>("RRTStar.d");

    // Read weights
    // w_x = pt_prob.get<double>("RRTStar.w_x");
    // w_y = pt_prob.get<double>("RRTStar.w_y");
    // w_theta = pt_prob.get<double>("RRTStar.w_theta");

    // Calculate V_free
    // double x_top = pt_space.get<double>("top.x");
    // double x_bottom = pt_space.get<double>("bottom.x");
    // double y_top = pt_space.get<double>("top.y");
    // double y_bottom = pt_space.get<double>("bottom.y");
    // double th_top = pt_space.get<double>("top.th");
    // double th_bottom = pt_space.get<double>("bottom.th");

    // double V_free = std::abs(x_top - x_bottom)*std::abs(y_top - y_bottom)*std::abs(th_top - th_bottom);
	double V_free = 13621.1;//280deg*pi/180deg^6=4.88692^6

    // double V_ball = 4.0 / 3.0 * M_PI;
	double V_ball = 5.1677;//6次元空間の単位球の体積（pi^3/6）
    gamma = 2.0 * std::pow(1.0 + 1.0 / d, 1.0 / d) * std::pow(V_free / V_ball, 1.0 / d);//2*1.02602*3.71666=7.6267349864
	}
	catch (const boost::property_tree::ptree_error& e) {
    	std::cerr << "INI parameter error: " << e.what() << std::endl;
    	std::exit(EXIT_FAILURE);
	}
}

// bool same_cfree_obj(const RRTNode& lhs, const RRTNode& rhs)
// {
// 	if(lhs.cfree_obj.size() != rhs.cfree_obj.size()) return false;

// 	for(int i = 0; i < (int)lhs.cfree_obj.size(); ++i){
// 		if(!same_pointcloud(lhs.cfree_obj[i], rhs.cfree_obj[i])) return false;
// 	}

// 	return true;
// }

double weighted_distance(const Node& q1, const Node& q2)
{
    // double dx = q1.node[0] - q2.node[0];
    // double dy = q1.node[1] - q2.node[1];
    // double dth = calc_dth2(q1.node[2], q2.node[2]);
    // return std::sqrt(w_x * dx * dx + w_y * dy * dy + w_theta * dth * dth);
	double max_diff = 0.0;
	const RRTStarCostConfig& config = rrtstar_cost_config();

	//6次元
	// 重みの設定 (0と3が根本の関節)
	for(int i=0; i<Node::dof; ++i){
		double diff = config.weights[i] * std::abs(q1.node[i] - q2.node[i]);
		if(diff > max_diff) max_diff = diff;
	}
	max_diff = max_diff / config.omega + config.time_loss;
	return max_diff;
}

void append_selected_path_log(const std::string& planner_name, int node_count, double total_cost)
{
	std::ofstream log("../ICM_Log/icm.log", std::ios::app);
	log << std::fixed << std::setprecision(6);
	log << planner_name << " selected path nodes : " << node_count << std::endl;
	log << planner_name << " selected path cost : " << total_cost << std::endl;
}

double nodelist_cost(const NodeList& path)
{
	double cost = 0.0;
	for(int i = 1; i < path.size(); ++i){
		cost += weighted_distance(path.get(i - 1), path.get(i));
	}
	return cost;
}

//-----------------------------
// double max_time;//関節移動時間
// double axit = ;//加速度
// double velo = ;//速度
// if(pow(velo,2) / axit > max_diff){
// 	max_time = 2 * pow(max_diff/axit,1/2);
// }
// else{
// 	max_time =2 * velo / axit + (max_diff - pow(velo,2)/axit)/velo;
// }
// return max_time;
//------------------------------
} // namespace

std::vector<int> RRTStar::get_neighbors_weighted(int index, double radius)
{
	std::vector<int> neighbors = tree.get_neighbors(index, radius);
	if(0 <= index && index < tree.size() && tree.get_RRTNode(index).is_valid && radius >= 0.0){
		neighbors.push_back(index);
	}

	return neighbors;
}

// bool RRTStar::cluster_number_judge(const PointCloud& child_region, const PointCloud& parent_region, int eps)
// {
// 	if (child_region.empty() || parent_region.empty()) return false;

// 	int point_number_child = child_region.size();
// 	int point_number_parent = parent_region.size();
// 	int dif = point_number_parent - point_number_child;
// 	std::cout << "前姿勢から現姿勢へのポイント数減少量は " << dif << std::endl;
// 	return dif > eps;
// }

void RRTStar::set_strategy(CFO* cfo)
{
	strategy = cfo;
}


bool RRTStar::initialize(Node ini)
{
	tree.push_back(ini, origin);
	// tree.get_RRTNode(0).cost=0.0;
	CFreeICS ics(ini);
	std::cout << ini << std::endl;
	if (!robot_update(ini))	return false;
	std::vector<PointCloud> init_CFree = ics.extract();
	if(init_CFree.size() == 0)	return false;
	print_ICSs(init_CFree);

	std::cout << "Select the cluster:";
	int index = 0;
	std::cin >> index;
	assert(0 <= index && index < (int)init_CFree.size());

	// tree.replace(RRTNode(ini, init_CFree[index]));
	RRTNode init_node(ini, init_CFree[index]);
	init_node.cost = 0.0;
	init_node.is_valid = true;
	tree.replace(init_node);
	return true;
}


bool RRTStar::dfsconfig_valid(Node newnode)
{
	std::vector<PointCloud> cfo = strategy->extract(tree.back_parentRRTNode().pc(), newnode);

	if((int)cfo.size() == 1){
		RRTNode validnode(newnode, cfo[0]);
		tree.replace(validnode);
		return true;
	}
	else {
		return false;
	}

}


Node RRTStar::sampling(Node Rand)
{
	static int loop = 0;
	loop++;
	Node newnode;

	if (garound.size() < 10)	newnode = tree.format(Rand);
	else {
		if (loop % 10 == 3 || loop % 10 == 7)	newnode = tree.format(Rand);
		else                                    newnode = format_around(Rand);
	}
	if(rrtstar_verbose()){
		std::cout << loop << ": ";
		std::cout << newnode << std::endl;
	}

	return newnode;
}


void RRTStar::add_garound()
{
	garound.push_back(tree.size() - 1);
	if(rrtstar_verbose()) std::cout << "around goal: " << garound.size() << std::endl;
}

void RRTStar::add_goal_candidate()
{
	int index = tree.get_now_index();
	for(int existing : goal_indices){
		if(existing == index) return;
	}
	goal_indices.push_back(index);
	std::cout << "[goal] candidate " << goal_indices.size()
		  << ": node index = " << index
		  << ", cost = " << tree.get_RRTNode(index).cost << std::endl;
}


Node RRTStar::format_around(Node rand)
{
	double dist = DBL_MAX;
	int index = -1;
	for (int i = 0; i < (int)garound.size(); ++i) {
		double tmp = tree.get_RRTNode(garound[i]).distance(rand);
		if (dist > tmp) {
			//index = tree.getParentIndex(garound[i]);
			index = garound[i];	// Change above line to this one (10/17)
			dist = tmp;
		}
	}

	Node fmt = rand.normalize(tree.get_RRTNode(index).node);
	tree.push_back(fmt, index);
	return fmt;
}


GoalJudge RRTStar::goal_judge(State3D goal)
{
	PointCloud pc = tree.back_RRTNode().pc();
    const double around_rate = 1.5;
    double max_dist = DBL_MIN;
    int xmin = INT_MAX, xmax = INT_MIN;

    for (int i = 0; i < pc.size(); ++i) {
        double dist = calc_dist(pc.get(i), goal);
        if (dist > threshold * around_rate)  return GoalJudge::NotGoal;

        int xtmp = pc.get(i).x;
        if (xtmp < xmin) xmin = xtmp;
        if (xtmp > xmax) xmax = xtmp;

        if (max_dist < dist)   max_dist = dist;
    }

    int delta_x = (xmax - xmin) / 2;
    max_dist = std::sqrt(max_dist * max_dist + delta_x * delta_x);
    if (max_dist > threshold * around_rate) return GoalJudge::NotGoal;

    if(rrtstar_verbose()) std::cout << "max distance:" << max_dist << std::endl;

//    if (!contain_yth(pc, goal)) {
//        std::cout << "epsilon is satisfied but doesn't contain goal state." << std::endl;
//        return GoalJudge::NotGoal;
//    }
    if (max_dist > threshold)    return GoalJudge::MiddleGoal;
    else                         return GoalJudge::Goal;

}


bool RRTStar::config_valid(Node newnode)
{
	std::string fn = "log.txt";
	std::ofstream ofs(fn, std::ios::app);

	set_strategy(new DfsCFO());
	std::vector<PointCloud> cfo = strategy->extract(tree.back_parentRRTNode().pc(), newnode);
	for(int i=0; i<(int)cfo.size(); ++i){
		ofs << cfo[i] << std::endl;
	}
	set_strategy(new RasterCFO());
	std::vector<PointCloud> cfo2 = strategy->extract(tree.back_parentRRTNode().pc(), newnode);
	for(int i=0; i<(int)cfo2.size(); ++i){
		ofs << cfo2[i] << std::endl;
	}

	std::cout << cfo.size() << ", " << cfo2.size() << std::endl;
	assert(cfo.size() == cfo2.size());

	if((int)cfo.size() == 1){
		std::cout << cfo[0].size() << ", " << cfo2[0].size() << std::endl;

		assert(cfo[0].size() == cfo2[0].size());
		RRTNode validnode(newnode, cfo[0]);
		tree.replace(validnode);
		return true;
	}
	else {
		return false;
	}

}


RRTStar::RRTStar()
	:tree(), garound(), goal_indices(), strategy(new DfsCFO()),
  	threshold(read_threshold())
{
	load_parameters(max_iterations, eta, gamma, d);
	std::ofstream log("../ICM_Log/icm.log", std::ios::app);
	log << "Forward epsilon : " << threshold << std::endl;
}


NodeList RRTStar::plan(Node ini, Node fin, State3D goal)
{
	rand_init();
	goal_indices.clear();
	g_path_cost_report.clear();

	if (!initialize(ini)) {
		std::cout << "Invalid initial value was given." << std::endl;
		return NodeList();
	}
	auto start = std::chrono::system_clock::now();

	int iteration = 0;
	while (iteration < max_iterations || goal_indices.empty())
	{
		++iteration;
		// Random sampling and format
		Node Rand = generate_newnode();
		Node newnode = sampling(Rand);

		// Validation
		if (!robot_update(newnode)){
			tree.pop_back();
			continue;
		}
		if (!dfsconfig_valid(newnode)){
			tree.pop_back();
			continue;
		}

		// RRTStar: Choose Parent
		int q_new_index = tree.size() - 1;
		double r = std::min(gamma * std::pow(std::log(tree.size()) / tree.size(), 1.0 / d), eta);
		std::vector<int> q_near = get_neighbors_weighted(q_new_index, r);
		if (!choose_parent(q_new_index, q_near)) {
			// No valid parent, discard q_new
			tree.pop_back();
			continue;
		}

		// Rewire
		rewire(q_new_index, q_near);

		// Post process
		GoalJudge flag = goal_judge(goal);
		if (flag == GoalJudge::MiddleGoal)	add_garound();
		if (flag == GoalJudge::Goal) {
			add_goal_candidate();
			if (iteration >= max_iterations)	break;
		}
	}

	auto end = std::chrono::system_clock::now();
	auto dur = end - start;
	auto sec = std::chrono::duration_cast<std::chrono::seconds>(dur).count();
	std::cout << "Elapsed time [s] :";	std::cout << sec << std::endl;

	int selected_goal_index = select_min_goal_index();
	g_path_cost_report = make_goal_cost_report(selected_goal_index);
	if(selected_goal_index >= 0){
		NodeList selected_path = tree.generate_path(selected_goal_index);
		append_selected_path_log("RRTStar", selected_path.size(), nodelist_cost(selected_path));
		return selected_path;
	}
	NodeList selected_path = tree.generate_path();
	append_selected_path_log("RRTStar", selected_path.size(), nodelist_cost(selected_path));
	return selected_path;
}


// ===============================================================================

int RRTStar::select_min_goal_index()
{
	int best_index = -1;
	double best_cost = std::numeric_limits<double>::infinity();

	for(int index : goal_indices){
		if(index < 0 || index >= tree.size()) continue;
		RRTNode& candidate = tree.get_RRTNode(index);
		if(!candidate.is_valid) continue;
		if(candidate.cost < best_cost){
			best_cost = candidate.cost;
			best_index = index;
		}
	}

	return best_index;
}

std::string RRTStar::make_goal_cost_report(int selected_index)
{
	std::ostringstream oss;
	oss << std::fixed << std::setprecision(6);

	if(goal_indices.empty()){
		oss << "[INFO] RRTStar goal cost summary: Goal判定まで到達した経路はありません．\n";
		oss << "[INFO] CSVには従来通り tree.generate_path() の経路を保存しました．\n";
		return oss.str();
	}

	oss << "[INFO] RRTStar goal cost summary: ゴールにたどり着いた経路は "
	    << goal_indices.size() << " 本です．\n";
	for(int i = 0; i < (int)goal_indices.size(); ++i){
		int index = goal_indices[i];
		if(index < 0 || index >= tree.size()){
			oss << "[INFO]   path " << (i + 1)
			    << ": node数 = invalid"
			    << ", cost = invalid\n";
			continue;
		}
		RRTNode& candidate = tree.get_RRTNode(index);
		int path_node_count = tree.generate_path(index).size();
		oss << "[INFO]   path " << (i + 1)
		    << ": node数 = " << path_node_count
		    << ", cost = " << candidate.cost;
		if(index == selected_index) oss << "  <-- selected";
		oss << '\n';
	}

	if(selected_index >= 0){
		oss << "[INFO] 今回の探索では最小コスト "
		    << tree.get_RRTNode(selected_index).cost
		    << " の経路を採用しました．\n";
	}
	else{
		oss << "[INFO] 有効なGoal候補が残っていないため、従来通り tree.generate_path() の経路を保存しました．\n";
	}

	return oss.str();
}


// ===============================================================================


void RevRRTStar::set_strategy(CFO* cfo)
{
	strategy = cfo;
}

bool RevRRTStar::initialize(Node fin)
{
	tree.push_back(fin, origin);
	CFreeICS ics(fin);

	for(int i=0; i<6;++i){
		std::cout << fin.get_element(i) << ", ";
	} std::cout << std::endl;
	if (!robot_update(fin))	return false;

	std::vector<PointCloud> fin_CFree = ics.extract();
	if(fin_CFree.size() == 0)	return false;
	print_ICSs(fin_CFree);

	std::cout << "Select the cluster:";
	int index = -1;
	std::cin >> index;
	assert(0 <= index && index < (int)fin_CFree.size());
	PointCloud selected = fin_CFree[index];
	fin_CFree.erase(fin_CFree.begin() + index);
	RRTNode init_node(fin, {selected}, fin_CFree);
	init_node.cost = 0.0;
	init_node.is_valid = true;
	tree.replace(init_node);

	return true;
}

bool RevRRTStar::dfsconfig_valid(Node newnode)
{
	Controller* controller = Controller::get_instance();

	std::vector<PointCloud> prev_cfree_obj = tree.back_parentRRTNode().get_cfree_obj();
	//std::vector<PointCloud> prev_cfree_del = tree.back_parentRRTNode().get_cfree_del();

	std::vector<PointCloud> cfree_obj;
	std::vector<PointCloud> del_list;

	int debug = 0;
	if(debug == 1)	set_strategy(new RasterCFO());
	for(const auto& eo: prev_cfree_obj){
		std::vector<PointCloud> cfree_obj_tmp = strategy->extract(eo, newnode);

		for(const auto& e : cfree_obj_tmp){
			//if(duplicate_check(e, cfree_obj))	continue;
			bool flag = false;
			for(int i=0; i<(int)cfree_obj.size(); ++i){
				if(cfree_obj[i].overlap(e)){
					flag = true;
					break;
				}
			}
			if(flag)	continue;
			cfree_obj.push_back(e);
		}
	}

	Node parent = tree.back_parentRRTNode().node;
	controller->robot_update(parent);
	for(auto it = cfree_obj.begin(); it != cfree_obj.end(); ){
		std::vector<PointCloud> prev_real_cfree = strategy->extract(*it, parent);
		if(prev_real_cfree.size() != 1){
			del_list.push_back(*it);
			it = cfree_obj.erase(it);
		}
		else{
			if(continuous_check(prev_real_cfree[0], *it)){
				++it;
			}
			else{
				del_list.push_back(*it);
				it = cfree_obj.erase(it);
			}
		}
	}
	controller->robot_update(newnode);

	if((int)cfree_obj.size() == 0)	return false;
	else{
		RRTNode validnode(newnode, cfree_obj, del_list);
		tree.replace(validnode);
		return true;
	}
}


bool RevRRTStar::continuous_check(PointCloud prev, PointCloud curr)
{
	double rate = 0.0;
	const double continue_rate = 3.0;

	if(prev.size() < curr.size()){
		rate = (double)curr.size()/prev.size();
	}
	else{
		rate = (double)prev.size()/curr.size();
	}

	return rate < continue_rate ? true : false;
}

Node RevRRTStar::sampling(Node Rand)
{
	static int loop = 0;
	loop++;
	Node newnode = tree.format(Rand);
	if(rrtstar_verbose()) std::cout << loop << ": " << newnode << std::endl;

	return newnode;
}

void RevRRTStar::add_goal_candidate()
{
	int index = tree.get_now_index();
	for(int existing : goal_indices){
		if(existing == index) return;
	}
	goal_indices.push_back(index);
	std::cout << "[goal] reverse candidate " << goal_indices.size()
		  << ": node index = " << index
		  << ", cost = " << tree.get_RRTNode(index).cost << std::endl;
}


GoalJudge RevRRTStar::goal_judge(std::vector<PointCloud> pcs)
{
//    int minx = INT_MAX, miny = INT_MAX, mint = INT_MAX;
//    int maxx = INT_MIN, maxy = INT_MIN, maxt = INT_MIN;
//    bool flag = false;
//
//    for (int i = 0; i < (int)pcs.size(); ++i) {
//        for (int j = 0; j < (int)pcs[i].size(); ++j) {
//            /*if (minx > pcs[i].get(j).x)  minx = pcs[i].get(j).x;
//            if (maxx < pcs[i].get(j).x)  maxx = pcs[i].get(j).x;*/
//            if (miny > pcs[i].get(j).y)  miny = pcs[i].get(j).y;
//            if (maxy < pcs[i].get(j).y)  maxy = pcs[i].get(j).y;
//            if (mint > pcs[i].get(j).th)  mint = pcs[i].get(j).th;
//            if (maxt < pcs[i].get(j).th)  maxt = pcs[i].get(j).th;
//            if (pcs[i].get(j).th == 0)   flag = true;
//        }
//        int widy = (maxy - miny) / 10;
//        int widt = (maxt - mint) / 5;
//
//        if (flag) {
//            int pmint = INT_MAX, pmaxt = INT_MIN;
//            for (int j = 0; j < (int)pcs[i].size(); ++j) {
//                if (pcs[i].get(j).th < 180) {
//                    if (pcs[i].get(j).th > pmaxt)    pmaxt = pcs[i].get(j).th;
//                }
//                if (pcs[i].get(j).th >= 180) {
//                    if (pcs[i].get(j).th < pmint)    pmint = pcs[i].get(j).th;
//                }
//            }
//            widt = (pmaxt + (360 - pmint)) / 5;
//        }
//
//        std::cout << "y width: " << widy << std::endl;
//        std::cout << "theta width: " << widt << std::endl;
//        if (widy < 8)      return GoalJudge::NotGoal;
//        if (widt < 25)      return GoalJudge::NotGoal;
//    }
//    return GoalJudge::Goal;

	static int maxi = 0;
	int nu = 1000;
	for(int i=0; i<(int)pcs.size(); ++i){
		if(maxi < pcs[i].size())	maxi = pcs[i].size();
		if(pcs[i].size() > nu){
			std::cout << "num: " << pcs[i].size() << std::endl;

			return GoalJudge::Goal;
		}
	}
	if(rrtstar_verbose()) std::cout << "Maxi: " << maxi << std::endl;
	return GoalJudge::NotGoal;
}

std::vector<int> RevRRTStar::get_neighbors_weighted(int index, double radius)
{
	std::vector<int> neighbors = tree.get_neighbors(index, radius);
	if(0 <= index && index < tree.size() && tree.get_RRTNode(index).is_valid && radius >= 0.0){
		neighbors.push_back(index);
	}

	return neighbors;
}


RevRRTStar::RevRRTStar()
	:tree(), garound(), goal_indices(), strategy(new DfsCFO()), cluster_distance_threshold(0.0)
{
	load_parameters(max_iterations, eta, gamma, d);
	std::ofstream log("../ICM_Log/icm.log", std::ios::app);
	log << "Reverse RRTStar initialization" << std::endl;
}


NodeList RevRRTStar::plan(Node ini, Node fin, State3D goal)
{
	rand_init();
	goal_indices.clear();
	g_path_cost_report.clear();

	if(!initialize(fin)){
		std::cout << "Invalid initial value was given." << std::endl;
		return NodeList();
	}

	auto start = std::chrono::system_clock::now();

	int iteration = 0;
	while (iteration < max_iterations || goal_indices.empty())
	{
		++iteration;
		// Random sampling and format
		Node Rand = generate_newnode();
		Node newnode = sampling(Rand);

		// Validation
		if (!robot_update(newnode)){
			tree.pop_back();
			continue;
		}
		if (!dfsconfig_valid(newnode)){
			tree.pop_back();
			continue;
		}

		// RRTStar: Choose Parent (Reverse version)
		int q_new_index = tree.size() - 1;
		double r = std::min(gamma * std::pow(std::log(tree.size()) / tree.size(), 1.0 / d), eta);
		std::vector<int> q_near = get_neighbors_weighted(q_new_index, r);
		if (!choose_parent(q_new_index, q_near)) {
			// No valid parent, discard q_new
			tree.pop_back();
			continue;
		}

		// Rewire
		rewire(q_new_index, q_near);

		// Post process
		GoalJudge flag = goal_judge(tree.back_RRTNode().get_cfree_obj());
		if (flag == GoalJudge::Goal) add_goal_candidate();
	}

	auto end = std::chrono::system_clock::now();
	auto dur = end - start;
	auto sec = std::chrono::duration_cast<std::chrono::seconds>(dur).count();
	std::cout << "Elapsed time [s] :";	std::cout << sec << std::endl;

	int selected_goal_index = select_min_goal_index();
	g_path_cost_report = make_goal_cost_report(selected_goal_index);

	NodeList path = selected_goal_index >= 0 ? tree.generate_path(selected_goal_index) : tree.generate_path();
	path.reverse();

	// If generate_path returned empty (e.g., trailing nodes invalidated),
	// try to build a partial path ending at the last valid node.
	if (path.size() == 0) {
		int last_valid = -1;
		for (int idx = tree.size() - 1; idx >= 0; --idx) {
			if (tree.get_RRTNode(idx).is_valid && !tree.get_RRTNode(idx).get_cfree_obj().empty()) {
				last_valid = idx;
				break;
			}
		}
		if (last_valid >= 0) {
			std::cerr << "[INFO] generate_path returned empty; using partial path to index " << last_valid << ".\n";
			path = tree.generate_path(last_valid);
			path.reverse();
		}
	}

	append_selected_path_log("RevRRTStar", path.size(), nodelist_cost(path));

	std::cout << "Debug time!\n";
	// Find the path head node in the tree and use its actual cfree_obj for debug verification.
	PointCloud pre_cfo;
	bool found_pre = false;
	if (path.size() > 0) {
		const Node start_node = path.get(0);
		for (int idx = 0; idx < tree.size(); ++idx) {
			RRTNode& candidate = tree.get_RRTNode(idx);
			if (!candidate.is_valid) continue;
			if (candidate.getNode().node == start_node.node) {
				auto cfo_list = candidate.get_cfree_obj();
				if (!cfo_list.empty()) {
					pre_cfo = cfo_list[0];
					found_pre = true;
					break;
				}
			}
		}
	}
	if (!found_pre) {
		for(int idx = tree.size() - 1; idx >= 0; --idx){
			if(tree.get_RRTNode(idx).is_valid){
				auto cfo_list = tree.get_RRTNode(idx).get_cfree_obj();
				if(!cfo_list.empty()){
					pre_cfo = cfo_list[0];
					found_pre = true;
					break;
				}
			}
		}
	}
	if(!found_pre){
		std::cerr << "Debug: no valid cfree_obj found, returning path." << std::endl;
		return path;
	}

	for(int i=1; i<path.size(); ++i){
		Node check = path.get(i);
		std::cout << check << std::endl;
		if(!robot_update(check)){
			std::cerr << "[WARN] Debug robot_update failed at path index " << i << ". Returning current path.\n";
			return path;
		}

		DfsCFO dfs;
		std::vector<PointCloud> cfo_now = dfs.extract(pre_cfo, check);
		if((int)cfo_now.size() != 1){
			std::cerr << "[WARN] Debug extract returned " << cfo_now.size() << " clusters at path index " << i << ". Returning current path.\n";
			return path;
		}
		pre_cfo = cfo_now[0];
	}

	return path;
}


int RevRRTStar::select_min_goal_index()
{
	int best_index = -1;
	double best_cost = std::numeric_limits<double>::infinity();

	for(int index : goal_indices){
		if(index < 0 || index >= tree.size()) continue;
		RRTNode& candidate = tree.get_RRTNode(index);
		if(!candidate.is_valid) continue;
		if(candidate.cost < best_cost){
			best_cost = candidate.cost;
			best_index = index;
		}
	}

	return best_index;
}

std::string RevRRTStar::make_goal_cost_report(int selected_index)
{
	std::ostringstream oss;
	oss << std::fixed << std::setprecision(6);

	if(goal_indices.empty()){
		oss << "[INFO] RevRRTStar goal cost summary: Goal判定まで到達した経路はありません．\n";
		oss << "[INFO] CSVには従来通り tree.generate_path() の経路を保存しました．\n";
		return oss.str();
	}

	oss << "[INFO] RevRRTStar goal cost summary: ゴールにたどり着いた経路は "
	    << goal_indices.size() << " 本です．\n";
	for(int i = 0; i < (int)goal_indices.size(); ++i){
		int index = goal_indices[i];
		if(index < 0 || index >= tree.size()){
			oss << "[INFO]   path " << (i + 1)
			    << ": node数 = invalid"
			    << ", cost = invalid\n";
			continue;
		}
		RRTNode& candidate = tree.get_RRTNode(index);
		int path_node_count = tree.generate_path(index).size();
		oss << "[INFO]   path " << (i + 1)
		    << ": node数 = " << path_node_count
		    << ", cost = " << candidate.cost;
		if(index == selected_index) oss << "  <-- selected";
		oss << '\n';
	}

	if(selected_index >= 0){
		oss << "[INFO] 今回の探索では最小コスト "
		    << tree.get_RRTNode(selected_index).cost
		    << " の経路を採用しました．\n";
	}
	else{
		oss << "[INFO] 有効なGoal候補が残っていないため、従来通り tree.generate_path() の経路を保存しました．\n";
	}

	return oss.str();
}


//////////////////////////////////////////////////////////////////////////

bool RRTStarConnect::initialize(Node ini, Node fin)
{
	CFreeICS ini_ics(ini), fin_ics(fin);
	s_tree.push_back(ini, origin);
	g_tree.push_back(fin, origin);
	int index1 = -1, index2 = -1;

	// Setting initial configuration
	if(!robot_update(ini))	return false;
	std::vector<PointCloud> ini_CFree = ini_ics.extract();
	if(ini_CFree.size() == 0)	return false;
	print_ICSs(ini_CFree);

	std::cout << "Select a cluster (start configuration):";
	std::cin >> index1;
	assert(0 <= index1 && index1 < (int)ini_CFree.size());
	RRTNode s_init(ini, ini_CFree[index1]);
	s_init.cost = 0.0;
	s_init.is_valid = true;
	s_tree.replace(s_init);

	// Setting goal configuration
	if(!robot_update(fin))	return false;
	std::vector<PointCloud> fin_CFree = fin_ics.extract();
	if(fin_CFree.size() == 0)	return false;
	print_ICSs(fin_CFree);

	std::cout << "Select a cluster (goal configuration):";
	std::cin >> index2;
	assert(0 <= index2 && index2 < (int)fin_CFree.size());
	PointCloud selected = fin_CFree[index2];
	fin_CFree.erase(fin_CFree.begin() + index2);
	RRTNode g_init(fin, {selected}, fin_CFree);
	g_init.cost = 0.0;
	g_init.is_valid = true;
	g_tree.replace(g_init);

	return true;
}


bool RRTStarConnect::sconf_update()
{
	Node newnode = s_tree.back_RRTNode().node; //新しいノードを取得
	if(!robot_update(newnode)){ //干渉判定
		s_tree.pop_back();
		return false;
	}
	if(!caging_validation_sconf(newnode)){ //ケージング成立条件とケージングマニピュレーション可能条件を同時に評価
		s_tree.pop_back();
		return false;
	}

	assert(s_tree.back_RRTNode().get_cfree_obj().size() == 1);
	return true;
}


bool RRTStarConnect::gconf_update()
{
	Node newnode = g_tree.back_RRTNode().node;
	if(!robot_update(newnode)){ //干渉判定
		g_tree.pop_back();
		return false;
	}
	if(!caging_validation_gconf(newnode)){ //ケージング成立条件とケージングマニピュレーション可能条件を同時に評価
		g_tree.pop_back();
		return false;
	}

	return true;
}


GoalJudge RRTStarConnect::sconf_goaljudge(State3D goal, RRTNode bef, RRTNode aft)
{
	// if(goal_sconf(goal) == GoalJudge::SGoal){
	// 	std::ofstream log("../ICM_Log/icm.log", std::ios::app);
	// 	log << "Forward Goal\n";
	// 	std::cout << "Start conf reached to the goal" << std::endl;
	// 	return GoalJudge::SGoal;
	// }
	if(goal_connect(bef, aft) == GoalJudge::Connect){
		std::cout << "Connect goal!" << std::endl;
		return GoalJudge::Connect;
	}

	return GoalJudge::NotGoal;
}


GoalJudge RRTStarConnect::gconf_goaljudge(std::vector<PointCloud> cfo, RRTNode bef, RRTNode aft)
{
	if(goal_gconf(cfo) == GoalJudge::GGoal){
		std::cout << "Goal conf reached to desire start condition." << std::endl;
		return GoalJudge::GGoal;
	}
	if(goal_connect(bef, aft) == GoalJudge::Connect){
		std::cout << "Connect goal." << std::endl;
		return GoalJudge::Connect;
	}

	return GoalJudge::NotGoal;
}


bool RRTStarConnect::caging_validation_sconf(Node node)
{
	std::vector<PointCloud> cfo = strategy->extract(s_tree.back_parentRRTNode().pc(), node); //cfo=c_free_obj.C(t-Δt)とC(t)の共通領域を算出

	if((int)cfo.size() == 1){ //ケージング成立条件とケージングマニピュレーション成立条件を同時に評価．C(t-Δt)とC(t)の共通領域が１つ
		RRTNode validnode(node, cfo[0]);
		s_tree.replace(validnode);
		return true;
	}
	else{
		return false;
	}
}

bool RRTStarConnect::caging_validation_gconf(Node newnode)
{
	Controller* controller = Controller::get_instance();

	std::vector<PointCloud> prev_cfree_obj = g_tree.back_parentRRTNode().get_cfree_obj();
	std::vector<PointCloud> prev_cfree_del = g_tree.back_parentRRTNode().get_cfree_del();

	std::vector<PointCloud> cfree_obj;
	std::vector<PointCloud> del_list;

	for(const auto& eo: prev_cfree_obj){
		std::vector<PointCloud> cfree_obj_tmp = strategy->extract(eo, newnode);

		for(const auto& e : cfree_obj_tmp){
			if(duplicate_check(e, cfree_obj))	continue;
			cfree_obj.push_back(e);
		}
	}

	Node parent = g_tree.back_parentRRTNode().node;//T_reveseを一つ遡り，ノードを取得
	controller->robot_update(parent);
	for(auto it = cfree_obj.begin(); it != cfree_obj.end(); ){
		std::vector<PointCloud> prev_real_cfree = strategy->extract(*it, parent); //C(t)とC(t+Δt)の共通領域を求める
		if(prev_real_cfree.size() != 1){ //ケージング成立条件とケージングマニピュレーション成立条件を同時に評価．C(t)とC(t+Δt)の共通領域が１つ出なければ弾く
			del_list.push_back(*it);
			it = cfree_obj.erase(it); //ここで当該クラスタを弾く
		}
		else{
			++it;
		}
	}
	controller->robot_update(newnode);

	if((int)cfree_obj.size() == 0)	return false;
	else{
		RRTNode validnode(newnode, cfree_obj, del_list);
		g_tree.replace(validnode);
		return true;
	}
}


GoalJudge RRTStarConnect::goal_sconf(State3D goal)
{
	PointCloud pc = s_tree.back_RRTNode().pc();
    double max_dist = DBL_MIN;
    int xmin = INT_MAX, xmax = INT_MIN;

    for (int i = 0; i < pc.size(); ++i) {
        double dist = calc_dist(pc.get(i), goal);
        if (dist > s_threshold)  return GoalJudge::NotGoal;

        int xtmp = pc.get(i).x;
        if (xtmp < xmin) xmin = xtmp;
        if (xtmp > xmax) xmax = xtmp;

        if (max_dist < dist)   max_dist = dist;
    }

    int delta_x = (xmax - xmin) / 2;
    max_dist = std::sqrt(max_dist * max_dist + delta_x * delta_x);
    if (max_dist > s_threshold) return GoalJudge::NotGoal;

    if(rrtstar_verbose()) std::cout << "max distance:" << max_dist << std::endl;

    if (!contain_yth(pc, goal)) {
        if(rrtstar_verbose()) std::cout << "epsilon is satisfied but doesn't contain goal state." << std::endl;
        return GoalJudge::NotGoal;
    }

	return GoalJudge::SGoal;
}


GoalJudge RRTStarConnect::goal_connect(RRTNode bef, RRTNode aft)
{
	double dist = bef.distance(aft);
	if(dist > 1.0)	return GoalJudge::NotGoal;

	std::vector<PointCloud> bef_cfree = bef.get_cfree_obj();
	std::vector<PointCloud> aft_cfree = aft.get_cfree_obj();

	if(bef_cfree.size() != 1) return GoalJudge::NotGoal;

	std::vector<PointCloud> overlap_cfree = strategy->extract(bef_cfree[0], aft.getNode());
	if(overlap_cfree.size() != 1) return GoalJudge::NotGoal;

	for(const auto& afree: aft_cfree){
		if(overlap_cfree[0].overlap(afree)){
			return GoalJudge::Connect;
		}
	}

	return GoalJudge::NotGoal;
}


GoalJudge RRTStarConnect::goal_gconf(std::vector<PointCloud> cfo)
{
//    int minx = INT_MAX, miny = INT_MAX, mint = INT_MAX;
//    int maxx = INT_MIN, maxy = INT_MIN, maxt = INT_MIN;
//    bool flag = false;
//
//    for (int i = 0; i < (int)cfo.size(); ++i) {
//        for (int j = 0; j < (int)cfo[i].size(); ++j) {
//            /*if (minx > cfo[i].get(j).x)  minx = cfo[i].get(j).x;
//            if (maxx < cfo[i].get(j).x)  maxx = cfo[i].get(j).x;*/
//            if (miny > cfo[i].get(j).y)  miny = cfo[i].get(j).y;
//            if (maxy < cfo[i].get(j).y)  maxy = cfo[i].get(j).y;
//            if (mint > cfo[i].get(j).th)  mint = cfo[i].get(j).th;
//            if (maxt < cfo[i].get(j).th)  maxt = cfo[i].get(j).th;
//            if (cfo[i].get(j).th == 0)   flag = true;
//        }
//        int widy = (maxy - miny) / 10;
//        int widt = (maxt - mint) / 5;
//
//        if (flag) {
//            int pmint = INT_MAX, pmaxt = INT_MIN;
//            for (int j = 0; j < (int)cfo[i].size(); ++j) {
//                if (cfo[i].get(j).th < 180) {
//                    if (cfo[i].get(j).th > pmaxt)    pmaxt = cfo[i].get(j).th;
//                }
//                if (cfo[i].get(j).th >= 180) {
//                    if (cfo[i].get(j).th < pmint)    pmint = cfo[i].get(j).th;
//                }
//            }
//            widt = (pmaxt + (360 - pmint)) / 5;
//        }
//
////        std::cout << "y width: " << widy << std::endl;
////        std::cout << "theta width: " << widt << std::endl;
//        if (widy < 8)      return GoalJudge::NotGoal;
//        if (widt < 18)      return GoalJudge::NotGoal;
//    }
//    return GoalJudge::GGoal;

	static int maxi = 0;
	int nu = 1000;
	for(int i=0; i<(int)cfo.size(); ++i){
		if(maxi < cfo[i].size())	maxi = cfo[i].size();
		if(cfo[i].size() > nu){
			return GoalJudge::GGoal;
		}
	}
//	std::cout << "Maxi: " << maxi << std::endl;
	return GoalJudge::NotGoal;

}


NodeList RRTStarConnect:: make_path(GoalJudge flag)
{
	if(flag == GoalJudge::SGoal){
		return s_tree.generate_path();
	}
	else if(flag == GoalJudge::GGoal){
		NodeList path = g_tree.generate_path();
		path.reverse();
		return path;
	}
	else if(flag == GoalJudge::Connect){
		return path_concat();
	}

	assert(true);
	return NodeList();
}


NodeList RRTStarConnect::make_path(GoalJudge flag, int sindex, int gindex)
{
	if(flag == GoalJudge::SGoal){
		return s_tree.generate_path();
	}
	else if(flag == GoalJudge::GGoal){
		NodeList path = g_tree.generate_path();
		path.reverse();
		return path;
	}
	else if(flag == GoalJudge::Connect){
		return path_concat(sindex, gindex);
	}

	assert(true);
	return NodeList();
}


bool RRTStarConnect::extend_limit(Node n1, Node n2)
{
	double threshold = n1.distance(n2);
	if(threshold < 1.0)	return true;
	return false;
}


NodeList RRTStarConnect::path_concat()
{
	NodeList spath, gpath;
	spath = s_tree.generate_path();
	gpath = g_tree.generate_path();
	gpath.reverse();
	spath.concat(gpath);
	return spath;
}

NodeList RRTStarConnect::path_concat(int sindex, int gindex)
{
	NodeList spath, gpath;
	spath = s_tree.generate_path(sindex);
	spath.printIO();
	std::cout << "latter2" << std::endl;
	gpath = g_tree.generate_path(gindex);
	gpath.reverse();
	gpath.printIO();
	spath.concat(gpath);
	return spath;
}

void RRTStarConnect::add_goal_candidate(GoalJudge flag, int sindex, int gindex)
{
	double cost = goal_candidate_cost(flag, sindex, gindex);
	if(!std::isfinite(cost)) return;

	for(const auto& candidate : goal_candidates){
		if(candidate.flag == flag && candidate.s_index == sindex && candidate.g_index == gindex) return;
	}

	goal_candidates.push_back({flag, sindex, gindex, cost});
	std::cout << "[goal] connect candidate " << goal_candidates.size()
		  << ": cost = " << cost << std::endl;
}

double RRTStarConnect::goal_candidate_cost(GoalJudge flag, int sindex, int gindex)
{
	if(flag == GoalJudge::SGoal){
		if(sindex < 0 || sindex >= s_tree.size()) return std::numeric_limits<double>::infinity();
		RRTNode& snode = s_tree.get_RRTNode(sindex);
		if(!snode.is_valid) return std::numeric_limits<double>::infinity();
		return snode.cost;
	}
	if(flag == GoalJudge::GGoal){
		if(gindex < 0 || gindex >= g_tree.size()) return std::numeric_limits<double>::infinity();
		RRTNode& gnode = g_tree.get_RRTNode(gindex);
		if(!gnode.is_valid) return std::numeric_limits<double>::infinity();
		return gnode.cost;
	}
	if(flag == GoalJudge::Connect){
		if(sindex < 0 || sindex >= s_tree.size()) return std::numeric_limits<double>::infinity();
		if(gindex < 0 || gindex >= g_tree.size()) return std::numeric_limits<double>::infinity();
		RRTNode& snode = s_tree.get_RRTNode(sindex);
		RRTNode& gnode = g_tree.get_RRTNode(gindex);
		if(!snode.is_valid || !gnode.is_valid) return std::numeric_limits<double>::infinity();
		return snode.cost + weighted_distance(snode.node, gnode.node) + gnode.cost;
	}
	return std::numeric_limits<double>::infinity();
}

NodeList RRTStarConnect::goal_candidate_path(const GoalCandidate& candidate)
{
	if(candidate.flag == GoalJudge::SGoal){
		return s_tree.generate_path(candidate.s_index);
	}
	if(candidate.flag == GoalJudge::GGoal){
		NodeList path = g_tree.generate_path(candidate.g_index);
		path.reverse();
		return path;
	}
	if(candidate.flag == GoalJudge::Connect){
		NodeList spath = s_tree.generate_path(candidate.s_index);
		NodeList gpath = g_tree.generate_path(candidate.g_index);
		gpath.reverse();
		spath.concat(gpath);
		return spath;
	}
	return NodeList();
}

int RRTStarConnect::select_min_goal_candidate_index()
{
	int best_index = -1;
	double best_cost = std::numeric_limits<double>::infinity();

	for(int i = 0; i < (int)goal_candidates.size(); ++i){
		double cost = goal_candidate_cost(goal_candidates[i].flag, goal_candidates[i].s_index, goal_candidates[i].g_index);
		if(cost < best_cost){
			best_cost = cost;
			best_index = i;
		}
	}

	return best_index;
}

std::string RRTStarConnect::make_goal_cost_report(int selected_candidate)
{
	std::ostringstream oss;
	oss << std::fixed << std::setprecision(6);

	if(goal_candidates.empty()){
		oss << "[INFO] RRTStarConnect goal cost summary: Goal判定まで到達した経路はありません．\n";
		return oss.str();
	}

	oss << "[INFO] RRTStarConnect goal cost summary: ゴールにたどり着いた経路は "
	    << goal_candidates.size() << " 本です．\n";
	for(int i = 0; i < (int)goal_candidates.size(); ++i){
		NodeList path = goal_candidate_path(goal_candidates[i]);
		double cost = goal_candidate_cost(goal_candidates[i].flag, goal_candidates[i].s_index, goal_candidates[i].g_index);
		oss << "[INFO]   path " << (i + 1)
		    << ": node数 = " << path.size()
		    << ", cost = " << cost;
		if(i == selected_candidate) oss << "  <-- selected";
		oss << '\n';
	}

	if(selected_candidate >= 0){
		oss << "[INFO] 今回の探索では最小コスト "
		    << goal_candidate_cost(goal_candidates[selected_candidate].flag,
		                           goal_candidates[selected_candidate].s_index,
		                           goal_candidates[selected_candidate].g_index)
		    << " の経路を採用しました．\n";
	}
	else{
		oss << "[INFO] 有効なGoal候補が残っていないため、経路を保存しませんでした．\n";
	}

	return oss.str();
}


RRTStarConnect::RRTStarConnect()
	:s_tree(), g_tree(), goal_candidates(),
	 s_threshold(read_threshold()),
	 strategy(new DfsCFO())
{
	load_parameters(max_iterations, eta, gamma, d);
	std::ofstream log("../ICM_Log/icm.log", std::ios::app);
	log << "Forward epsilon : " << s_threshold << std::endl;
	log << "RRTStarConnect initialization" << std::endl;
}


NodeList RRTStarConnect::plan(Node ini, Node fin, State3D goal)
{
	rand_init();
	goal_candidates.clear();
	g_path_cost_report.clear();

	if(!initialize(ini, fin)){
		std::cout << "Invalid initial value was given." << std::endl;
		return NodeList();
	}

	auto start = std::chrono::system_clock::now();
	int iteration = 0;

	while(iteration < max_iterations || goal_candidates.empty())
	{
		++iteration;

		static int i = 0;	++i;
		if(i>500000)	exit(5963);

		// Random sampling and format
		Node Rand = generate_newnode();
		RRTNode opponent_node;
		int opponent_index;

		if(rrtstar_verbose() && i % 10 == 0){
			std::cout << "start conf: " << s_tree.size() <<
			    	   "  goal conf: " << g_tree.size() << std::endl;
		}
		if(rrtstar_verbose()) std::cout << i << ": ";

		if(s_tree.size() < g_tree.size()){
			Node newnode = s_tree.format(Rand);
			if(rrtstar_verbose()) std::cout << newnode.node << std::endl;
			opponent_index = g_tree.get_nearest_index(newnode);
			opponent_node = g_tree.get_RRTNode(opponent_index);

			if(!sconf_update())	continue;
			RRTNode sconf_newnode = s_tree.back_RRTNode();

			// RRTStar*: Choose Parent for s_tree
			int q_new_index = s_tree.get_now_index();
			double r = std::min(gamma * std::pow(std::log(s_tree.size()) / s_tree.size(), 1.0 / d), eta);
			std::vector<int> q_near_s = get_neighbors_weighted_s(q_new_index, r);
			if (!choose_parent_s(q_new_index, q_near_s)) {
				s_tree.pop_back();
				continue;
			}

			// RRTStar*: Rewire for s_tree
			rewire_s(q_new_index, q_near_s);

			sconf_newnode = s_tree.back_RRTNode();
			GoalJudge sgj = sconf_goaljudge(goal, sconf_newnode, opponent_node);
			if(sgj != GoalJudge::NotGoal){
				if(sgj == GoalJudge::Connect){
					add_goal_candidate(sgj, s_tree.get_now_index(), opponent_index);
				}
				else{
					add_goal_candidate(sgj, s_tree.get_now_index(), -1);
				}
			}

			while(1){
				g_tree.add(opponent_index, newnode);
				if(!gconf_update())	break;

				// RRTStar*: Choose Parent for g_tree
				int q_new_index_g = g_tree.get_now_index();
				double r_g = std::min(gamma * std::pow(std::log(g_tree.size()) / g_tree.size(), 1.0 / d), eta);
				std::vector<int> q_near_g = get_neighbors_weighted_g(q_new_index_g, r_g);
				if (!choose_parent_g(q_new_index_g, q_near_g)) {
					g_tree.pop_back();
					break;
				}

				// RRTStar*: Rewire for g_tree
				rewire_g(q_new_index_g, q_near_g);

				assert(s_tree.back_RRTNode().get_cfree_obj().size() == 1);
				GoalJudge ggj = gconf_goaljudge(
						g_tree.back_RRTNode().get_cfree_obj(),
						s_tree.back_RRTNode(), g_tree.back_RRTNode());

				if(ggj != GoalJudge::NotGoal){
					if(ggj == GoalJudge::Connect){
						add_goal_candidate(ggj, s_tree.get_now_index(), g_tree.get_now_index());
					}
					else{
						add_goal_candidate(ggj, -1, g_tree.get_now_index());
					}
				}

				if(extend_limit(newnode, g_tree.back_RRTNode().node)){
					break;
				}

				opponent_index = g_tree.get_now_index();
				opponent_node  = g_tree.back_RRTNode();
			}
		}

		else{
			Node newnode = g_tree.format(Rand);
			if(rrtstar_verbose()) std::cout << newnode.node << std::endl;
			opponent_index = s_tree.get_nearest_index(newnode);
			opponent_node  = s_tree.get_RRTNode(opponent_index);

			if(!gconf_update())	continue;
			RRTNode gconf_newnode = g_tree.back_RRTNode();

			// RRTStar*: Choose Parent for g_tree
			int q_new_index_g = g_tree.get_now_index();
			double r_g = std::min(gamma * std::pow(std::log(g_tree.size()) / g_tree.size(), 1.0 / d), eta);
			std::vector<int> q_near_g = get_neighbors_weighted_g(q_new_index_g, r_g);
			if (!choose_parent_g(q_new_index_g, q_near_g)) {
				g_tree.pop_back();
				continue;
			}

			// RRTStar*: Rewire for g_tree
			rewire_g(q_new_index_g, q_near_g);

			gconf_newnode = g_tree.back_RRTNode();
			GoalJudge ggj = gconf_goaljudge(
					g_tree.back_RRTNode().get_cfree_obj(),
					opponent_node, gconf_newnode);

			if(ggj != GoalJudge::NotGoal){
				if(ggj == GoalJudge::Connect){
					add_goal_candidate(ggj, opponent_index, g_tree.get_now_index());
				}
				else{
					add_goal_candidate(ggj, -1, g_tree.get_now_index());
				}
			}

			while(1){
				s_tree.add(opponent_index, newnode);
				if(!sconf_update())	break;

				// RRTStar*: Choose Parent for s_tree
				int q_new_index = s_tree.get_now_index();
				double r = std::min(gamma * std::pow(std::log(s_tree.size()) / s_tree.size(), 1.0 / d), eta);
				std::vector<int> q_near_s = get_neighbors_weighted_s(q_new_index, r);
				if (!choose_parent_s(q_new_index, q_near_s)) {
					s_tree.pop_back();
					break;
				}

				// RRTStar*: Rewire for s_tree
				rewire_s(q_new_index, q_near_s);

				assert(opponent_node.get_cfree_obj().size() == 1);
				GoalJudge sgj = sconf_goaljudge(goal, s_tree.back_RRTNode(), g_tree.back_RRTNode());
				if(sgj != GoalJudge::NotGoal){
					if(sgj == GoalJudge::Connect){
						add_goal_candidate(sgj, s_tree.get_now_index(), g_tree.get_now_index());
					}
					else{
						add_goal_candidate(sgj, s_tree.get_now_index(), -1);
					}
				}

				if(extend_limit(newnode, s_tree.back_RRTNode().node)){
					break;
				}

				opponent_index = s_tree.get_now_index();
				opponent_node  = s_tree.back_RRTNode();
			}
		}
	}

	auto end = std::chrono::system_clock::now();
	auto dur = end - start;
	auto sec = std::chrono::duration_cast<std::chrono::seconds>(dur).count();
	std::cout << "Elapsed time [s] :";	std::cout << sec << std::endl;

	int selected_candidate = select_min_goal_candidate_index();
	g_path_cost_report = make_goal_cost_report(selected_candidate);
	if(selected_candidate >= 0){
		NodeList selected_path = goal_candidate_path(goal_candidates[selected_candidate]);
		double selected_cost = goal_candidate_cost(goal_candidates[selected_candidate].flag,
		                                           goal_candidates[selected_candidate].s_index,
		                                           goal_candidates[selected_candidate].g_index);
		append_selected_path_log("RRTStarConnect", selected_path.size(), selected_cost);
		return selected_path;
	}

	return NodeList();
}




bool RRTStar::choose_parent(int q_new_index, const std::vector<int>& q_near)
{
	RRTNode& q_new = tree.get_RRTNode(q_new_index);
	const Node child_node = q_new.node;
	const int current_parent = tree.get_parent_index(q_new_index);
	if(current_parent < 0) return false;

	const RRTNode& current_parent_node = tree.get_RRTNode(current_parent);
	const double current_cost = current_parent_node.cost + weighted_distance(current_parent_node.node, child_node);

	std::vector<std::pair<double, int>> candidates;
	for(int parent_index : q_near){
		if(parent_index == q_new_index) continue;
		RRTNode& parent = tree.get_RRTNode(parent_index);
		double tentative_cost = parent.cost + weighted_distance(parent.node, child_node);
		if(tentative_cost < current_cost){
			candidates.emplace_back(tentative_cost, parent_index);
		}
	}
	std::sort(candidates.begin(), candidates.end());

	int best_parent = -1;
	RRTNode best_node;
	for(auto& cand : candidates){
		int parent_index = cand.second;
		RRTNode validnode;
		if(validate_edge(parent_index, child_node, validnode, -1)){
			best_parent = parent_index;
			best_node = validnode;
			break;
		}
	}

	if(best_parent == -1){
		q_new.cost = current_cost;
		q_new.is_valid = true;
		if(rrtstar_verbose()){
			std::cout << "[choose_parent] 親未更新: コスト関数 = " << weighted_distance(current_parent_node.node, child_node)
				  << ", 累積コスト = " << q_new.cost << std::endl;
		}
		return true;
	}

	tree.set_parent_index(q_new_index, best_parent);
	tree.get_RRTNode(q_new_index) = best_node;
	if(rrtstar_verbose()){
		std::cout << "[choose_parent] 親更新: コスト関数 = " << weighted_distance(tree.get_RRTNode(best_parent).node, child_node)
			  << ", 累積コスト = " << best_node.cost << std::endl;
	}
	return true;
}

bool RRTStar::validate_edge(int parent_index, const Node& child_node, RRTNode& out_node, int child_index)
{
	RRTNode& parent = tree.get_RRTNode(parent_index);

    if(!robot_update(child_node)){
        if(rrtstar_verbose()) std::cout << "衝突判定fail (RRT*)" << std::endl;
		return false;
	}

    std::vector<PointCloud> cfree = strategy->extract(parent.pc(), child_node);
    if(cfree.size() != 1){
	    if(rrtstar_verbose()) std::cout << "分裂判定fail (RRT*)" << std::endl;
        return false;
	}

    // if(cluster_number_judge(cfree[0], parent.pc(), 90)){
    //     std::cout << "重なり率判定fail (RRT*)" << std::endl;
    //     return false;
    // }

    if(rrtstar_verbose()) std::cout << "制約pass (RRT*)" << std::endl;

    RRTNode validnode(child_node, cfree[0]);
	double edge_cost = weighted_distance(parent.node, child_node);
	validnode.cost = parent.cost + edge_cost;
	validnode.is_valid = true;
	if(rrtstar_verbose()){
		std::cout << "[validate_edge] ノード採用: コスト関数 = " << edge_cost
			  << ", 累積コスト = " << validnode.cost << std::endl;
	}
	// リワイヤされたノードのクラスタ情報をキャッシュに保存（child_indexが有効な場合）
	if(child_index >= 0){
		RRTNode& child_rrt = tree.get_RRTNode(child_index);
		validnode.cluster_cache = {child_rrt.pc()};
	}
	out_node = validnode;
    return true;
}

std::vector<PointCloud> RRTStar::extract_cfree(int parent_index, const Node& child_node)
{
    return strategy->extract(tree.get_RRTNode(parent_index).pc(), child_node);
}

bool RRTStar::rewire(int q_new_index, const std::vector<int>& q_near)
{
	bool rewired = false;
	int rewired_count = 0;
	RRTNode& q_new = tree.get_RRTNode(q_new_index);

	for(int near_index : q_near){
		if(near_index == q_new_index) continue;

		RRTNode& near_node = tree.get_RRTNode(near_index);
		double edge_cost = weighted_distance(q_new.node, near_node.node);
		double new_cost = q_new.cost + edge_cost;
		if(new_cost >= near_node.cost) continue;

		RRTNode validnode;
		double old_cost = near_node.cost;
		if(validate_edge(q_new_index, near_node.node, validnode, near_index)){
			tree.set_parent_index(near_index, q_new_index);
			validnode.cost = new_cost;
			tree.get_RRTNode(near_index) = validnode;
			if(rrtstar_verbose()){
				std::cout << "[rewire] ノード" << near_index << "をリワイヤ: コスト関数 = " << edge_cost
					  << ", 新しい累積コスト = " << new_cost << " (前: " << old_cost << ")" << std::endl;
			}

			update_subtree_after_rewire(near_index);
			rewired = true;
			rewired_count++;
		}
	}

	if(rewired_count > 0){
		if(rrtstar_verbose()) std::cout << "rewired(" << rewired_count << "個）成功" << std::endl;
	}
	return rewired;
}

void RRTStar::update_subtree_after_rewire(int index)
{
	std::vector<int> queue = get_children_indices(index);
	std::vector<bool> visited(tree.size(), false);
	if(0 <= index && index < tree.size()) visited[index] = true;

	for(int head = 0; head < (int)queue.size(); ++head){
		int child_index = queue[head];
		if(child_index < 0 || child_index >= tree.size()) continue;
		if(visited[child_index]) continue;
		visited[child_index] = true;

		int parent_index = tree.get_parent_index(child_index);
		if(parent_index < 0 || parent_index >= tree.size()){
			repair_failed_branch(child_index);
			continue;
		}

		RRTNode& parent = tree.get_RRTNode(parent_index);
		RRTNode& child = tree.get_RRTNode(child_index);

		// 親の位置で再度クラスタを抽出
		std::vector<PointCloud> parent_cluster_reextracted = strategy->extract(parent.pc(), child.node);

		// 抽出結果が親のキャッシュされたクラスタと同じかチェック
		bool cluster_same = false;
		if(parent.cluster_cache.size() == 1 && parent_cluster_reextracted.size() == 1){
			cluster_same = same_pointcloud(parent.cluster_cache[0], parent_cluster_reextracted[0]);
		}

		if(cluster_same){
			// クラスタが同じ -> 親以降の全子孫ノードがpassしたと見なす
			// 単にコストとparent_indexを更新
			child.cost = parent.cost + weighted_distance(parent.node, child.node);
			if(rrtstar_verbose()) std::cout << "クラスタ情報が同じため、子孫ノード再チェックをスキップ (RRT*)" << std::endl;
		}
		else{
			// クラスタが異なる -> 制約チェックを再度実行
			RRTNode validated_node;
			if(validate_edge(parent_index, child.node, validated_node, child_index)){
				tree.set_parent_index(child_index, parent_index);
				tree.get_RRTNode(child_index) = validated_node;
			}
			else{
				repair_failed_branch(child_index);
				continue;
			}
		}

		std::vector<int> children = get_children_indices(child_index);
		for(int next_child : children){
			if(0 <= next_child && next_child < tree.size() && !visited[next_child]){
				queue.push_back(next_child);
			}
		}
	}
}

std::vector<int> RRTStar::get_children_indices(int parent)
{
	return tree.get_children_indices(parent);
}

void RRTStar::repair_failed_branch(int root_index)
{
	if(root_index < 0 || root_index >= tree.size()) return;

	const int sample_count = 10;
	Node failed_node = tree.get_RRTNode(root_index).node;
	std::vector<int> nearby = get_neighbors_weighted(root_index, eta);
	std::vector<int> sampled_parents;

	while(!nearby.empty() && (int)sampled_parents.size() < sample_count){
		int pick = std::rand() % nearby.size();
		int parent_index = nearby[pick];
		nearby.erase(nearby.begin() + pick);

		if(parent_index == root_index) continue;
		if(!tree.get_RRTNode(parent_index).is_valid) continue;
		sampled_parents.push_back(parent_index);
	}

	std::vector<std::pair<double, int>> candidates;
	for(int parent_index : sampled_parents){
		RRTNode& parent = tree.get_RRTNode(parent_index);
		double candidate_cost = parent.cost + weighted_distance(parent.node, failed_node);
		candidates.emplace_back(candidate_cost, parent_index);
	}
	std::sort(candidates.begin(), candidates.end());

	for(const auto& candidate : candidates){
		int parent_index = candidate.second;
		RRTNode validated_node;
		if(validate_edge(parent_index, failed_node, validated_node, -1)){
			tree.set_parent_index(root_index, parent_index);
			validated_node.cost = candidate.first;
			tree.get_RRTNode(root_index) = validated_node;
			update_subtree_after_rewire(root_index);
			return;
		}
	}

	std::vector<int> queue;
	std::vector<bool> visited(tree.size(), false);
	queue.push_back(root_index);

	for(int head = 0; head < (int)queue.size(); ++head){
		int index = queue[head];
		if(index < 0 || index >= tree.size()) continue;
		if(visited[index]) continue;
		visited[index] = true;

		tree.invalidate(index);

		std::vector<int> children = get_children_indices(index);
		for(int child_index : children){
			if(0 <= child_index && child_index < tree.size() && !visited[child_index]){
				queue.push_back(child_index);
			}
		}
	}
}

// ===============================================================================
// RevRRTStar* methods implementation
// ===============================================================================

bool RevRRTStar::choose_parent(int q_new_index, const std::vector<int>& q_near)
{
	RRTNode& q_new = tree.get_RRTNode(q_new_index);
	const Node child_node = q_new.node;
	const int current_parent = tree.get_parent_index(q_new_index);
	if(current_parent < 0) return false;

	const RRTNode& current_parent_node = tree.get_RRTNode(current_parent);
	const double current_cost = current_parent_node.cost + weighted_distance(current_parent_node.node, child_node);

	std::vector<std::pair<double, int>> candidates;
	for(int parent_index : q_near){
		if(parent_index == q_new_index) continue;
		RRTNode& parent = tree.get_RRTNode(parent_index);
		double tentative_cost = parent.cost + weighted_distance(parent.node, child_node);
		if(tentative_cost < current_cost){
			candidates.emplace_back(tentative_cost, parent_index);
		}
	}
	std::sort(candidates.begin(), candidates.end());

	int best_parent = -1;
	RRTNode best_node;
	for(auto& cand : candidates){
		int parent_index = cand.second;
		RRTNode validnode;
		if(validate_edge(parent_index, child_node, validnode, -1)){
			best_parent = parent_index;
			best_node = validnode;
			break;
		}
	}

	if(best_parent == -1){
		q_new.cost = current_cost;
		q_new.is_valid = true;
		if(rrtstar_verbose()){
			std::cout << "[choose_parent] 親未更新: コスト関数 = " << weighted_distance(current_parent_node.node, child_node)
				  << ", 累積コスト = " << q_new.cost << std::endl;
		}
		return true;
	}

	tree.set_parent_index(q_new_index, best_parent);
	tree.get_RRTNode(q_new_index) = best_node;
	if(rrtstar_verbose()){
		std::cout << "[choose_parent] 親更新: コスト関数 = " << weighted_distance(tree.get_RRTNode(best_parent).node, child_node)
			  << ", 累積コスト = " << best_node.cost << std::endl;
	}
	return true;
}

bool RevRRTStar::validate_edge(int parent_index, const Node& child_node, RRTNode& out_node, int child_index)
{
	RRTNode& parent = tree.get_RRTNode(parent_index);
	Controller* controller = Controller::get_instance();

    if(!robot_update(child_node)){
        if(rrtstar_verbose()) std::cout << "衝突判定fail (RevRRT*)" << std::endl;
		return false;
	}

	    // For Reverse version, handle multiple clusters from parent
	std::vector<PointCloud> prev_cfree_obj = parent.get_cfree_obj();
	std::vector<PointCloud> cfree_obj;
	std::vector<PointCloud> del_list;

	for(const auto& eo: prev_cfree_obj){
		std::vector<PointCloud> cfree_obj_tmp = strategy->extract(eo, child_node);

		for(const auto& e : cfree_obj_tmp){
			bool flag = false;
			for(int i=0; i<(int)cfree_obj.size(); ++i){
				if(cfree_obj[i].overlap(e)){
					flag = true;
					break;
				}
			}
			if(flag)	continue;
			cfree_obj.push_back(e);
		}
	}

	Node parent_node = parent.node;
	controller->robot_update(parent_node);
	for(auto it = cfree_obj.begin(); it != cfree_obj.end(); ){
		std::vector<PointCloud> prev_real_cfree = strategy->extract(*it, parent_node);
		if(prev_real_cfree.size() != 1){
			del_list.push_back(*it);
			it = cfree_obj.erase(it);
		}
		else{
			if(continuous_check(prev_real_cfree[0], *it)){
				++it;
			}
			else{
				del_list.push_back(*it);
				it = cfree_obj.erase(it);
			}
		}
	}
	controller->robot_update(child_node);

	if((int)cfree_obj.size() == 0){
	    if(rrtstar_verbose()) std::cout << "分裂判定fail (RevRRT*)" << std::endl;
        return false;
	}

    if(rrtstar_verbose()) std::cout << "制約pass (RevRRT*)" << std::endl;

    RRTNode validnode(child_node, cfree_obj, del_list);
	double edge_cost = weighted_distance(parent.node, child_node);
	validnode.cost = parent.cost + edge_cost;
	validnode.is_valid = true;
	if(rrtstar_verbose()){
		std::cout << "[validate_edge] ノード採用: コスト関数 = " << edge_cost
			  << ", 累積コスト = " << validnode.cost << std::endl;
	}
	// キャッシュに保存（child_indexが有効な場合）
	if(child_index >= 0){
		RRTNode& child_rrt = tree.get_RRTNode(child_index);
		validnode.cluster_cache = child_rrt.get_cfree_obj();
	}
	out_node = validnode;
    return true;
}

std::vector<PointCloud> RevRRTStar::extract_cfree(int parent_index, const Node& child_node)
{
	RRTNode& parent = tree.get_RRTNode(parent_index);
	std::vector<PointCloud> prev_cfree_obj = parent.get_cfree_obj();
	std::vector<PointCloud> result;

	for(const auto& eo: prev_cfree_obj){
		std::vector<PointCloud> tmp = strategy->extract(eo, child_node);
		for(const auto& e : tmp){
			result.push_back(e);
		}
	}
	return result;
}

bool RevRRTStar::rewire(int q_new_index, const std::vector<int>& q_near)
{
	bool rewired = false;
	int rewired_count = 0;
	RRTNode& q_new = tree.get_RRTNode(q_new_index);

	for(int near_index : q_near){
		if(near_index == q_new_index) continue;

		RRTNode& near_node = tree.get_RRTNode(near_index);
			double tentative_cost = q_new.cost + weighted_distance(q_new.node, near_node.node);

		if(tentative_cost < near_node.cost){
			RRTNode validated_node;
			if(validate_edge(q_new_index, near_node.node, validated_node, near_index)){
				tree.set_parent_index(near_index, q_new_index);
				tree.get_RRTNode(near_index) = validated_node;
				if(rrtstar_verbose()){
					std::cout << "[rewire] リワイア実行: インデックス = " << near_index
						  << ", 新コスト = " << validated_node.cost << std::endl;
				}
				update_subtree_after_rewire(near_index);
				rewired = true;
				rewired_count++;
			}
		}
	}

	if(rewired_count > 0){
		if(rrtstar_verbose()) std::cout << "[rewire] リワイア完了: " << rewired_count << "個のノードを更新" << std::endl;
	}
	return rewired;
}

void RevRRTStar::update_subtree_after_rewire(int index)
{
	std::vector<int> queue = get_children_indices(index);
	std::vector<bool> visited(tree.size(), false);
	if(0 <= index && index < tree.size()) visited[index] = true;

	for(int head = 0; head < (int)queue.size(); ++head){
		int child_index = queue[head];
		if(child_index < 0 || child_index >= tree.size()) continue;
		if(visited[child_index]) continue;
		visited[child_index] = true;

		int parent_index = tree.get_parent_index(child_index);
		if(parent_index < 0 || parent_index >= tree.size()){
			repair_failed_branch(child_index);
			continue;
		}

		const Node child_node = tree.get_RRTNode(child_index).node;
		RRTNode validated_node;
		if(!validate_edge(parent_index, child_node, validated_node, child_index)){
			repair_failed_branch(child_index);
			continue;
		}

		tree.get_RRTNode(child_index) = validated_node;
		if(rrtstar_verbose()){
			std::cout << "[update_subtree] 経路状態とコストを再計算: インデックス = "
				  << child_index << ", 新コスト = " << validated_node.cost << std::endl;
		}

		std::vector<int> children = get_children_indices(child_index);
		for(int next_child : children){
			if(0 <= next_child && next_child < tree.size() && !visited[next_child]){
				queue.push_back(next_child);
			}
		}
	}
}

std::vector<int> RevRRTStar::get_children_indices(int parent)
{
	return tree.get_children_indices(parent);
}

void RevRRTStar::repair_failed_branch(int root_index)
{
	if(root_index < 0 || root_index >= tree.size()) return;

	const int sample_count = 10;
	Node failed_node = tree.get_RRTNode(root_index).node;
	std::vector<int> nearby = get_neighbors_weighted(root_index, eta);
	std::vector<int> sampled_parents;

	while(!nearby.empty() && (int)sampled_parents.size() < sample_count){
		int pick = std::rand() % nearby.size();
		int parent_index = nearby[pick];
		nearby.erase(nearby.begin() + pick);

		if(parent_index == root_index) continue;
		if(!tree.get_RRTNode(parent_index).is_valid) continue;
		sampled_parents.push_back(parent_index);
	}

	std::vector<std::pair<double, int>> candidates;
	for(int parent_index : sampled_parents){
		RRTNode& parent = tree.get_RRTNode(parent_index);
		double candidate_cost = parent.cost + weighted_distance(parent.node, failed_node);
		candidates.emplace_back(candidate_cost, parent_index);
	}
	std::sort(candidates.begin(), candidates.end());

	for(const auto& candidate : candidates){
		int parent_index = candidate.second;
		RRTNode validated_node;
		if(validate_edge(parent_index, failed_node, validated_node, -1)){
			tree.set_parent_index(root_index, parent_index);
			validated_node.cost = candidate.first;
			tree.get_RRTNode(root_index) = validated_node;
			update_subtree_after_rewire(root_index);
			return;
		}
	}

	std::vector<int> queue;
	std::vector<bool> visited(tree.size(), false);
	queue.push_back(root_index);

	for(int head = 0; head < (int)queue.size(); ++head){
		int index = queue[head];
		if(index < 0 || index >= tree.size()) continue;
		if(visited[index]) continue;
		visited[index] = true;

		tree.invalidate(index);

		std::vector<int> children = get_children_indices(index);
		for(int child_index : children){
			if(0 <= child_index && child_index < tree.size() && !visited[child_index]){
				queue.push_back(child_index);
			}
		}
	}
}

void RevRRTStar::add_garound()
{
	garound.push_back(tree.size() - 1);
	if(rrtstar_verbose()) std::cout << "around goal: " << garound.size() << std::endl;
}

Node RevRRTStar::format_around(Node rand)
{
	double dist = DBL_MAX;
	int index = -1;
	for (int i = 0; i < (int)garound.size(); ++i) {
		double tmp = tree.get_RRTNode(garound[i]).distance(rand);
		if (dist > tmp) {
			index = garound[i];
			dist = tmp;
		}
	}

	Node fmt = rand.normalize(tree.get_RRTNode(index).node);
	tree.push_back(fmt, index);
	return fmt;
}
// ===============================================================================
// RRTStarConnect* methods implementation
// ===============================================================================

std::vector<int> RRTStarConnect::get_neighbors_weighted_s(int index, double radius)
{
	std::vector<int> neighbors = s_tree.get_neighbors(index, radius);
	if(0 <= index && index < s_tree.size() && s_tree.get_RRTNode(index).is_valid && radius >= 0.0){
		neighbors.push_back(index);
	}

	return neighbors;
}

std::vector<int> RRTStarConnect::get_neighbors_weighted_g(int index, double radius)
{
	std::vector<int> neighbors = g_tree.get_neighbors(index, radius);
	if(0 <= index && index < g_tree.size() && g_tree.get_RRTNode(index).is_valid && radius >= 0.0){
		neighbors.push_back(index);
	}

	return neighbors;
}

bool RRTStarConnect::choose_parent_s(int q_new_index, const std::vector<int>& q_near)
{
	RRTNode& q_new = s_tree.get_RRTNode(q_new_index);
	const Node child_node = q_new.node;
	const int current_parent = s_tree.get_parent_index(q_new_index);
	if(current_parent < 0) return false;

	const RRTNode& current_parent_node = s_tree.get_RRTNode(current_parent);
	const double current_cost = current_parent_node.cost + weighted_distance(current_parent_node.node, child_node);

	std::vector<std::pair<double, int>> candidates;
	for(int parent_index : q_near){
		if(parent_index == q_new_index) continue;
		RRTNode& parent = s_tree.get_RRTNode(parent_index);
		double tentative_cost = parent.cost + weighted_distance(parent.node, child_node);
		if(tentative_cost < current_cost){
			candidates.emplace_back(tentative_cost, parent_index);
		}
	}
	std::sort(candidates.begin(), candidates.end());

	int best_parent = -1;
	RRTNode best_node;
	for(auto& cand : candidates){
		int parent_index = cand.second;
		RRTNode validnode;
		if(validate_edge_s(parent_index, child_node, validnode, -1)){
			best_parent = parent_index;
			best_node = validnode;
			break;
		}
	}

	if(best_parent == -1){
		q_new.cost = current_cost;
		q_new.is_valid = true;
		if(rrtstar_verbose()){
			std::cout << "[choose_parent_s] 親未更新: コスト関数 = " << weighted_distance(current_parent_node.node, child_node)
				  << ", 累積コスト = " << q_new.cost << std::endl;
		}
		return true;
	}

	s_tree.set_parent_index(q_new_index, best_parent);
	s_tree.get_RRTNode(q_new_index) = best_node;
	if(rrtstar_verbose()){
		std::cout << "[choose_parent_s] 親更新: コスト関数 = " << weighted_distance(s_tree.get_RRTNode(best_parent).node, child_node)
			  << ", 累積コスト = " << best_node.cost << std::endl;
	}
	return true;
}

bool RRTStarConnect::choose_parent_g(int q_new_index, const std::vector<int>& q_near)
{
	RRTNode& q_new = g_tree.get_RRTNode(q_new_index);
	const Node child_node = q_new.node;
	const int current_parent = g_tree.get_parent_index(q_new_index);
	if(current_parent < 0) return false;

	const RRTNode& current_parent_node = g_tree.get_RRTNode(current_parent);
	const double current_cost = current_parent_node.cost + weighted_distance(current_parent_node.node, child_node);

	std::vector<std::pair<double, int>> candidates;
	for(int parent_index : q_near){
		if(parent_index == q_new_index) continue;
		RRTNode& parent = g_tree.get_RRTNode(parent_index);
		double tentative_cost = parent.cost + weighted_distance(parent.node, child_node);
		if(tentative_cost < current_cost){
			candidates.emplace_back(tentative_cost, parent_index);
		}
	}
	std::sort(candidates.begin(), candidates.end());

	int best_parent = -1;
	RRTNode best_node;
	for(auto& cand : candidates){
		int parent_index = cand.second;
		RRTNode validnode;
		if(validate_edge_g(parent_index, child_node, validnode, -1)){
			best_parent = parent_index;
			best_node = validnode;
			break;
		}
	}

	if(best_parent == -1){
		q_new.cost = current_cost;
		q_new.is_valid = true;
		if(rrtstar_verbose()){
			std::cout << "[choose_parent_g] 親未更新: コスト関数 = " << weighted_distance(current_parent_node.node, child_node)
				  << ", 累積コスト = " << q_new.cost << std::endl;
		}
		return true;
	}

	g_tree.set_parent_index(q_new_index, best_parent);
	g_tree.get_RRTNode(q_new_index) = best_node;
	if(rrtstar_verbose()){
		std::cout << "[choose_parent_g] 親更新: コスト関数 = " << weighted_distance(g_tree.get_RRTNode(best_parent).node, child_node)
			  << ", 累積コスト = " << best_node.cost << std::endl;
	}
	return true;
}

bool RRTStarConnect::rewire_s(int q_new_index, const std::vector<int>& q_near)
{
	bool rewired = false;
	RRTNode& q_new = s_tree.get_RRTNode(q_new_index);

	for(int near_index : q_near){
		if(near_index == q_new_index) continue;

		RRTNode& near_node = s_tree.get_RRTNode(near_index);
		double edge_cost = weighted_distance(q_new.node, near_node.node);
		double new_cost = q_new.cost + edge_cost;
		if(new_cost >= near_node.cost) continue;

		RRTNode validnode;
		double old_cost = near_node.cost;
		if(validate_edge_s(q_new_index, near_node.node, validnode, near_index)){
			s_tree.set_parent_index(near_index, q_new_index);
			validnode.cost = new_cost;
			s_tree.get_RRTNode(near_index) = validnode;
			if(rrtstar_verbose()){
				std::cout << "[rewire_s] ノード" << near_index << "をリワイヤ: コスト関数 = " << edge_cost
					  << ", 新しい累積コスト = " << new_cost << " (前: " << old_cost << ")" << std::endl;
			}
			update_subtree_after_rewire_s(near_index);
			rewired = true;
		}
	}

	return rewired;
}

bool RRTStarConnect::rewire_g(int q_new_index, const std::vector<int>& q_near)
{
	bool rewired = false;
	RRTNode& q_new = g_tree.get_RRTNode(q_new_index);

	for(int near_index : q_near){
		if(near_index == q_new_index) continue;

		RRTNode& near_node = g_tree.get_RRTNode(near_index);
		double edge_cost = weighted_distance(q_new.node, near_node.node);
		double new_cost = q_new.cost + edge_cost;
		if(new_cost >= near_node.cost) continue;

		RRTNode validnode;
		double old_cost = near_node.cost;
		if(validate_edge_g(q_new_index, near_node.node, validnode, near_index)){
			g_tree.set_parent_index(near_index, q_new_index);
			validnode.cost = new_cost;
			g_tree.get_RRTNode(near_index) = validnode;
			if(rrtstar_verbose()){
				std::cout << "[rewire_g] ノード" << near_index << "をリワイヤ: コスト関数 = " << edge_cost
					  << ", 新しい累積コスト = " << new_cost << " (前: " << old_cost << ")" << std::endl;
			}
			update_subtree_after_rewire_g(near_index);
			rewired = true;
		}
	}

	return rewired;
}

void RRTStarConnect::update_subtree_after_rewire_s(int index)
{
	std::vector<int> queue = get_children_indices_s(index);
	std::vector<bool> visited(s_tree.size(), false);
	if(0 <= index && index < s_tree.size()) visited[index] = true;

	for(int head = 0; head < (int)queue.size(); ++head){
		int child_index = queue[head];
		if(child_index < 0 || child_index >= s_tree.size()) continue;
		if(visited[child_index]) continue;
		visited[child_index] = true;

		int parent_index = s_tree.get_parent_index(child_index);
		if(parent_index < 0 || parent_index >= s_tree.size()){
			repair_failed_branch_s(child_index);
			continue;
		}

		const Node child_node = s_tree.get_RRTNode(child_index).node;
		RRTNode validated_node;
		if(!validate_edge_s(parent_index, child_node, validated_node, child_index)){
			repair_failed_branch_s(child_index);
			continue;
		}

		s_tree.get_RRTNode(child_index) = validated_node;

		std::vector<int> children = get_children_indices_s(child_index);
		for(int next_child : children){
			if(0 <= next_child && next_child < s_tree.size() && !visited[next_child]){
				queue.push_back(next_child);
			}
		}
	}
}

void RRTStarConnect::update_subtree_after_rewire_g(int index)
{
	std::vector<int> queue = get_children_indices_g(index);
	std::vector<bool> visited(g_tree.size(), false);
	if(0 <= index && index < g_tree.size()) visited[index] = true;

	for(int head = 0; head < (int)queue.size(); ++head){
		int child_index = queue[head];
		if(child_index < 0 || child_index >= g_tree.size()) continue;
		if(visited[child_index]) continue;
		visited[child_index] = true;

		int parent_index = g_tree.get_parent_index(child_index);
		if(parent_index < 0 || parent_index >= g_tree.size()){
			repair_failed_branch_g(child_index);
			continue;
		}

		const Node child_node = g_tree.get_RRTNode(child_index).node;
		RRTNode validated_node;
		if(!validate_edge_g(parent_index, child_node, validated_node, child_index)){
			repair_failed_branch_g(child_index);
			continue;
		}

		g_tree.get_RRTNode(child_index) = validated_node;

		std::vector<int> children = get_children_indices_g(child_index);
		for(int next_child : children){
			if(0 <= next_child && next_child < g_tree.size() && !visited[next_child]){
				queue.push_back(next_child);
			}
		}
	}
}

std::vector<int> RRTStarConnect::get_children_indices_s(int parent)
{
	return s_tree.get_children_indices(parent);
}

std::vector<int> RRTStarConnect::get_children_indices_g(int parent)
{
	return g_tree.get_children_indices(parent);
}

void RRTStarConnect::repair_failed_branch_s(int root_index)
{
	if(root_index < 0 || root_index >= s_tree.size()) return;

	const int sample_count = 10;
	Node failed_node = s_tree.get_RRTNode(root_index).node;
	std::vector<int> nearby = get_neighbors_weighted_s(root_index, eta);
	std::vector<int> sampled_parents;

	while(!nearby.empty() && (int)sampled_parents.size() < sample_count){
		int pick = std::rand() % nearby.size();
		int parent_index = nearby[pick];
		nearby.erase(nearby.begin() + pick);

		if(parent_index == root_index) continue;
		if(!s_tree.get_RRTNode(parent_index).is_valid) continue;
		sampled_parents.push_back(parent_index);
	}

	std::vector<std::pair<double, int>> candidates;
	for(int parent_index : sampled_parents){
		RRTNode& parent = s_tree.get_RRTNode(parent_index);
		double candidate_cost = parent.cost + weighted_distance(parent.node, failed_node);
		candidates.emplace_back(candidate_cost, parent_index);
	}
	std::sort(candidates.begin(), candidates.end());

	for(const auto& candidate : candidates){
		int parent_index = candidate.second;
		RRTNode validated_node;
		if(validate_edge_s(parent_index, failed_node, validated_node, -1)){
			s_tree.set_parent_index(root_index, parent_index);
			validated_node.cost = candidate.first;
			s_tree.get_RRTNode(root_index) = validated_node;
			update_subtree_after_rewire_s(root_index);
			return;
		}
	}

	std::vector<int> queue;
	std::vector<bool> visited(s_tree.size(), false);
	queue.push_back(root_index);

	for(int head = 0; head < (int)queue.size(); ++head){
		int index = queue[head];
		if(index < 0 || index >= s_tree.size()) continue;
		if(visited[index]) continue;
		visited[index] = true;

		s_tree.invalidate(index);

		std::vector<int> children = get_children_indices_s(index);
		for(int child_index : children){
			if(0 <= child_index && child_index < s_tree.size() && !visited[child_index]){
				queue.push_back(child_index);
			}
		}
	}
}

void RRTStarConnect::repair_failed_branch_g(int root_index)
{
	if(root_index < 0 || root_index >= g_tree.size()) return;

	const int sample_count = 10;
	Node failed_node = g_tree.get_RRTNode(root_index).node;
	std::vector<int> nearby = get_neighbors_weighted_g(root_index, eta);
	std::vector<int> sampled_parents;

	while(!nearby.empty() && (int)sampled_parents.size() < sample_count){
		int pick = std::rand() % nearby.size();
		int parent_index = nearby[pick];
		nearby.erase(nearby.begin() + pick);

		if(parent_index == root_index) continue;
		if(!g_tree.get_RRTNode(parent_index).is_valid) continue;
		sampled_parents.push_back(parent_index);
	}

	std::vector<std::pair<double, int>> candidates;
	for(int parent_index : sampled_parents){
		RRTNode& parent = g_tree.get_RRTNode(parent_index);
		double candidate_cost = parent.cost + weighted_distance(parent.node, failed_node);
		candidates.emplace_back(candidate_cost, parent_index);
	}
	std::sort(candidates.begin(), candidates.end());

	for(const auto& candidate : candidates){
		int parent_index = candidate.second;
		RRTNode validated_node;
		if(validate_edge_g(parent_index, failed_node, validated_node, -1)){
			g_tree.set_parent_index(root_index, parent_index);
			validated_node.cost = candidate.first;
			g_tree.get_RRTNode(root_index) = validated_node;
			update_subtree_after_rewire_g(root_index);
			return;
		}
	}

	std::vector<int> queue;
	std::vector<bool> visited(g_tree.size(), false);
	queue.push_back(root_index);

	for(int head = 0; head < (int)queue.size(); ++head){
		int index = queue[head];
		if(index < 0 || index >= g_tree.size()) continue;
		if(visited[index]) continue;
		visited[index] = true;

		g_tree.invalidate(index);

		std::vector<int> children = get_children_indices_g(index);
		for(int child_index : children){
			if(0 <= child_index && child_index < g_tree.size() && !visited[child_index]){
				queue.push_back(child_index);
			}
		}
	}
}

bool RRTStarConnect::validate_edge_s(int parent_index, const Node& child_node, RRTNode& out_node, int child_index)
{
	RRTNode& parent = s_tree.get_RRTNode(parent_index);

    if(!robot_update(child_node)){
		return false;
	}

    std::vector<PointCloud> cfree = strategy->extract(parent.pc(), child_node);
    if(cfree.size() != 1){
        return false;
	}

    if(rrtstar_verbose()) std::cout << "制約pass (RRTStarConnect-s)" << std::endl;

    RRTNode validnode(child_node, cfree[0]);
	double edge_cost = weighted_distance(parent.node, child_node);
	validnode.cost = parent.cost + edge_cost;
	validnode.is_valid = true;
	if(rrtstar_verbose()){
		std::cout << "[validate_edge_s] ノード採用: コスト関数 = " << edge_cost
			  << ", 累積コスト = " << validnode.cost << std::endl;
	}
	out_node = validnode;
    return true;
}

bool RRTStarConnect::validate_edge_g(int parent_index, const Node& child_node, RRTNode& out_node, int child_index)
{
	RRTNode& parent = g_tree.get_RRTNode(parent_index);
	Controller* controller = Controller::get_instance();

    if(!robot_update(child_node)){
		return false;
	}

	    // For Reverse version (goal tree), handle multiple clusters from parent
	std::vector<PointCloud> prev_cfree_obj = parent.get_cfree_obj();
	std::vector<PointCloud> cfree_obj;
	std::vector<PointCloud> del_list;

	for(const auto& eo: prev_cfree_obj){
		std::vector<PointCloud> cfree_obj_tmp = strategy->extract(eo, child_node);

		for(const auto& e : cfree_obj_tmp){
			bool flag = false;
			for(int i=0; i<(int)cfree_obj.size(); ++i){
				if(cfree_obj[i].overlap(e)){
					flag = true;
					break;
				}
			}
			if(flag)	continue;
			cfree_obj.push_back(e);
		}
	}

	Node parent_node = parent.node;
	controller->robot_update(parent_node);
	for(auto it = cfree_obj.begin(); it != cfree_obj.end(); ){
		std::vector<PointCloud> prev_real_cfree = strategy->extract(*it, parent_node);
		if(prev_real_cfree.size() != 1){
			del_list.push_back(*it);
			it = cfree_obj.erase(it);
		}
		else{
			++it;
		}
	}
	controller->robot_update(child_node);

	if((int)cfree_obj.size() == 0)	return false;
	else{
		if(rrtstar_verbose()) std::cout << "制約pass (RRTStarConnect-g)" << std::endl;

		RRTNode validnode(child_node, cfree_obj, del_list);
		double edge_cost = weighted_distance(parent.node, child_node);
		validnode.cost = parent.cost + edge_cost;
		validnode.is_valid = true;
		if(rrtstar_verbose()){
			std::cout << "[validate_edge_g] ノード採用: コスト関数 = " << edge_cost
				  << ", 累積コスト = " << validnode.cost << std::endl;
		}
		out_node = validnode;
		return true;
	}
}

std::vector<PointCloud> RRTStarConnect::extract_cfree_s(int parent_index, const Node& child_node)
{
    return strategy->extract(s_tree.get_RRTNode(parent_index).pc(), child_node);
}

std::vector<PointCloud> RRTStarConnect::extract_cfree_g(int parent_index, const Node& child_node)
{
	RRTNode& parent = g_tree.get_RRTNode(parent_index);
	std::vector<PointCloud> result;

	for(const auto& cfree: parent.get_cfree_obj()){
		std::vector<PointCloud> tmp = strategy->extract(cfree, child_node);
		for(const auto& e: tmp){
			if(duplicate_check(e, result)) continue;
			result.push_back(e);
		}
	}

	return result;
}

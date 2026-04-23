
#include<iostream>
#include<chrono>
#include<climits>

#include "RRT.h"
#include "CFreeICS.h"
#include "Robot.h"
#include "CFree.h"
#include "pathsmooth.h"


int origin = -1;
using namespace std::chrono;

void RRT::set_strategy(CFO* cfo)
{
	strategy = cfo;
}


bool RRT::initialize(Node ini)
{
	tree.push_back(ini, origin);
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
	
	tree.replace(RRTNode(ini, init_CFree[index]));
	return true;
}


bool RRT::dfsconfig_valid(Node newnode)
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


Node RRT::sampling(Node Rand)
{
	static int loop = 0;
	loop++;
	Node newnode;

	if (garound.size() < 10)	newnode = tree.format(Rand);
	else {
		if (loop % 10 == 3 || loop % 10 == 7)	newnode = tree.format(Rand);
		else                                    newnode = format_around(Rand);
	}
	std::cout << loop << ": ";
	std::cout << newnode << std::endl;

	return newnode;
}


void RRT::add_garound()
{
	garound.push_back(tree.size() - 1);
	std::cout << "around goal: " << garound.size() << std::endl;
}


Node RRT::format_around(Node rand)
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


GoalJudge RRT::goal_judge(State3D goal)
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

    std::cout << "max distance:" << max_dist << std::endl;

//    if (!contain_yth(pc, goal)) {
//        std::cout << "epsilon is satisfied but doesn't contain goal state." << std::endl;
//        return GoalJudge::NotGoal;
//    }
    if (max_dist > threshold)    return GoalJudge::MiddleGoal;
    else                         return GoalJudge::Goal;

}


bool RRT::config_valid(Node newnode)
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


RRT::RRT()
	:tree(), garound(), strategy(new DfsCFO()), 
  	threshold(read_threshold())
{
	std::ofstream log("../ICM_Log/icm.log", std::ios::app);
	log << "Forward epsilon : " << threshold << std::endl;
}


NodeList RRT::plan(Node ini, Node fin, State3D goal)
{
	rand_init();

	if (!initialize(ini)) {
		std::cout << "Invalid initial value was given." << std::endl;
		return NodeList();
	}
	auto start = std::chrono::system_clock::now();

	while (1)
	{
		static int i = 0;
		++i;
		if (i > 300000)	exit(5963);
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

		// Post process
		GoalJudge flag = goal_judge(goal);
		if (flag == GoalJudge::MiddleGoal)	add_garound();
		if (flag == GoalJudge::Goal)	break;
	}

	auto end = std::chrono::system_clock::now();
	auto dur = end - start;
	auto sec = std::chrono::duration_cast<std::chrono::seconds>(dur).count();
	std::cout << "Elapsed time [s] :";	std::cout << sec << std::endl;

	return tree.generate_path();
}


// ===============================================================================


void RevRRT::set_strategy(CFO* cfo)
{
	strategy = cfo;
}

bool RevRRT::initialize(Node fin)
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
	tree.replace(RRTNode(fin, {selected}, fin_CFree));

	return true;
}

bool RevRRT::dfsconfig_valid(Node newnode)
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


bool RevRRT::continuous_check(PointCloud prev, PointCloud curr)
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

Node RevRRT::sampling(Node Rand)
{
	static int loop = 0;
	loop++;
	Node newnode = tree.format(Rand);
	std::cout << loop << ": " << newnode << std::endl;

	return newnode;
}


GoalJudge RevRRT::goal_judge(std::vector<PointCloud> pcs)
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
			std::ofstream log("../ICM_Log/icm.log", std::ios::app);
			log << "Reverse nu : " << nu << std::endl;
			std::cout << "num: " << pcs[i].size() << std::endl;

			return GoalJudge::Goal;
		}
	}
	std::cout << "Maxi: " << maxi << std::endl;
	return GoalJudge::NotGoal;
}


RevRRT::RevRRT()
	:tree(), strategy(new DfsCFO())
{}


NodeList RevRRT::plan(Node ini, Node fin, State3D goal)
{
	rand_init();

	if(!initialize(fin)){
		std::cout << "Invalid initial value was given." << std::endl;
		return NodeList();
	}
	
	while(1)
	{
		static int i = 0;	++i;
		//if(i>200000)	exit(5963);
		
		// Random sampling and format
		Node Rand = generate_newnode();
		Node newnode = sampling(Rand);

		// Validation
		if(!robot_update(newnode)){
			tree.pop_back();
			continue;
		}
		if(!dfsconfig_valid(newnode)){
			tree.pop_back();
			continue;
		}

		GoalJudge flag = goal_judge(tree.back_RRTNode().get_cfree_obj());

		if(flag == GoalJudge::Goal)	break;
	}
	
	NodeList path = tree.generate_path();
	path.reverse();

	std::cout << "Debug time!\n";
	PointCloud pre_cfo = tree.back_RRTNode().get_cfree_obj()[0];
	for(int i=1; i<path.size(); ++i){
		Node check = path.get(i);
		std::cout << check << std::endl;
		if(!robot_update(check))	assert(false);
		
		DfsCFO dfs;
		std::vector<PointCloud> cfo_now = dfs.extract(pre_cfo, check);
		if((int)cfo_now.size() != 1)	assert(false);
		pre_cfo = cfo_now[0];
	}

	return path;
}


//////////////////////////////////////////////////////////////////////////

bool RRTConnect::initialize(Node ini, Node fin)
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
	s_tree.replace(RRTNode(ini, ini_CFree[index1]));

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
	g_tree.replace(RRTNode(fin, {selected}, fin_CFree));

	return true;
}


bool RRTConnect::sconf_update()
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


bool RRTConnect::gconf_update()
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


GoalJudge RRTConnect::sconf_goaljudge(State3D goal, RRTNode bef, RRTNode aft)
{
	if(goal_sconf(goal) == GoalJudge::SGoal){
		std::ofstream log("../ICM_Log/icm.log", std::ios::app);
		log << "Forward Goal\n";
		std::cout << "Start conf reached to the goal" << std::endl;
		return GoalJudge::SGoal;
	}
	if(goal_connect(bef, aft) == GoalJudge::Connect){
		std::ofstream log("../ICM_Log/icm.log", std::ios::app);
		log << "Connect Goal\n";
		std::cout << "Connect goal!" << std::endl;
		return GoalJudge::Connect;
	}

	return GoalJudge::NotGoal;
}


GoalJudge RRTConnect::gconf_goaljudge(std::vector<PointCloud> cfo, RRTNode bef, RRTNode aft)
{
	if(goal_gconf(cfo) == GoalJudge::GGoal){
		std::ofstream log("../ICM_Log/icm.log", std::ios::app);
		log << "Reverse Goal\n";
		std::cout << "Goal conf reached to desire start condition." << std::endl;
		return GoalJudge::GGoal;
	}
	if(goal_connect(bef, aft) == GoalJudge::Connect){
		std::ofstream log("../ICM_Log/icm.log", std::ios::app);
		log << "Connect Goal\n";
		std::cout << "Connect goal." << std::endl;
		return GoalJudge::Connect;
	}
	
	return GoalJudge::NotGoal;
}


bool RRTConnect::caging_validation_sconf(Node node)
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

bool RRTConnect::caging_validation_gconf(Node newnode)
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


GoalJudge RRTConnect::goal_sconf(State3D goal)
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

    std::cout << "max distance:" << max_dist << std::endl;

    if (!contain_yth(pc, goal)) {
        std::cout << "epsilon is satisfied but doesn't contain goal state." << std::endl;
        return GoalJudge::NotGoal;
    }

	return GoalJudge::SGoal;
}


GoalJudge RRTConnect::goal_connect(RRTNode bef, RRTNode aft)
{
	double dist = bef.distance(aft);
	if(dist > 1.0)	return GoalJudge::NotGoal;

	std::vector<PointCloud> bef_cfree = bef.get_cfree_obj();
	std::vector<PointCloud> aft_cfree = aft.get_cfree_obj();
	std::vector<PointCloud> overlap_cfree;

	for(const auto& bfree: bef_cfree){
		for(const auto& afree: aft_cfree){
			bool tof = bfree.overlap(afree);
			if(!tof){
				return GoalJudge::NotGoal;
			}
		}
	}

	assert(bef_cfree.size() == 1);
	overlap_cfree = strategy->extract(bef_cfree[0], aft.getNode());
	
	if(overlap_cfree.size() == 1){
		return GoalJudge::Connect;
	}
	else return GoalJudge::NotGoal;
}


GoalJudge RRTConnect::goal_gconf(std::vector<PointCloud> cfo)
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
			std::ofstream log("../ICM_Log/icm.log", std::ios::app);
			log << "Reverse nu : " << nu << std::endl;
			return GoalJudge::GGoal;
		}
	}
//	std::cout << "Maxi: " << maxi << std::endl;
	return GoalJudge::NotGoal;

}


NodeList RRTConnect:: make_path(GoalJudge flag)
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


NodeList RRTConnect::make_path(GoalJudge flag, int sindex, int gindex)
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


bool RRTConnect::extend_limit(Node n1, Node n2)
{
	double threshold = n1.distance(n2);
	if(threshold < 1.0)	return true;
	return false;
}


NodeList RRTConnect::path_concat()
{
	NodeList spath, gpath;
	spath = s_tree.generate_path();
	gpath = g_tree.generate_path();
	gpath.reverse();
	spath.concat(gpath);
	return spath;
}

NodeList RRTConnect::path_concat(int sindex, int gindex)
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


RRTConnect::RRTConnect()
	:s_tree(), g_tree(),
	 s_threshold(read_threshold()),
	 strategy(new DfsCFO())
{
	std::ofstream log("../ICM_Log/icm.log", std::ios::app);
	log << "Forward epsilon : " << s_threshold << std::endl;
}


NodeList RRTConnect::plan(Node ini, Node fin, State3D goal)
{
	rand_init();

	if(!initialize(ini, fin)){
		std::cout << "Invalid initial value was given." << std::endl;
		return NodeList();
	}

	while(1)
	{
		static int i = 0;	++i;
		if(i>500000)	exit(5963);
		
		// Random sampling and format
		Node Rand = generate_newnode();
		RRTNode opponent_node;
		int opponent_index;
	
		if(i % 10 == 0){
			std::cout << "start conf: " << s_tree.size() << 
			    	   "  goal conf: " << g_tree.size() << std::endl;
		}std::cout << i << ": ";
		
		if(s_tree.size() < g_tree.size()){
			Node newnode = s_tree.format(Rand);
			std::cout << newnode.node << std::endl;
			opponent_node = g_tree.get_nearest_node(newnode);
			opponent_index = g_tree.get_nearest_index(newnode);

			if(!sconf_update())	continue;
			RRTNode sconf_newnode = s_tree.back_RRTNode();

			GoalJudge sgj = sconf_goaljudge(goal, sconf_newnode, opponent_node);	
			if(sgj != GoalJudge::NotGoal)	
				return make_path(sgj, s_tree.get_now_index(), opponent_index);

			while(1){
				g_tree.add(opponent_index, newnode); 
				if(!gconf_update())	break;

				assert(s_tree.back_RRTNode().get_cfree_obj().size() == 1);
				GoalJudge ggj = gconf_goaljudge(
						g_tree.back_RRTNode().get_cfree_obj(), 
						s_tree.back_RRTNode(), g_tree.back_RRTNode());
				
				if(ggj != GoalJudge::NotGoal)	return make_path(ggj);

				if(extend_limit(newnode, g_tree.back_RRTNode().node)){
					break;
				}
				
				opponent_index = g_tree.get_now_index();
				opponent_node  = g_tree.back_RRTNode();
			}
		}

		else{
			Node newnode = g_tree.format(Rand);
			std::cout << newnode.node << std::endl;
			opponent_index = s_tree.get_nearest_index(newnode);
			opponent_node  = s_tree.get_nearest_node(newnode);

			if(!gconf_update())	continue;
			RRTNode gconf_newnode = g_tree.back_RRTNode();

			GoalJudge ggj = gconf_goaljudge(
					g_tree.back_RRTNode().get_cfree_obj(), 
					opponent_node, gconf_newnode);
			
			if(ggj != GoalJudge::NotGoal)	
				return make_path(ggj, opponent_index, g_tree.get_now_index());
			
			while(1){
				s_tree.add(opponent_index, newnode);
				if(!sconf_update())	break;
				assert(opponent_node.get_cfree_obj().size() == 1);
				GoalJudge sgj = sconf_goaljudge(goal, s_tree.back_RRTNode(), g_tree.back_RRTNode());
				if(sgj != GoalJudge::NotGoal)	return make_path(sgj);
			
				if(extend_limit(newnode, s_tree.back_RRTNode().node)){
					break;
				}
	
				opponent_index = s_tree.get_now_index();
				opponent_node  = s_tree.back_RRTNode();
			}
		}
	}
	
	return NodeList();
}



void rand_init()
{
	std::ofstream log("../ICM_Log/icm.log", std::ios::app);
	auto seed = duration_cast<nanoseconds>(system_clock::now().time_since_epoch()).count() % 100000;
	log << "Seed : " << seed << std::endl;

	std::srand((unsigned int)seed);
	std::cout << "Seed value is " << seed << std::endl;
}
//固定seed版
// void rand_init()
// {
//     std::ofstream log("../ICM_Log/icm.log", std::ios::app);
//     unsigned int seed = 45790;  // ★ 固定値にする（例: 0や42など）

//     log << "Seed : " << seed << " (fixed)" << std::endl;
//     std::srand(seed);
//     std::cout << "Seed value is " << seed << " (fixed)" << std::endl;
// }

Node generate_newnode()
{
	std::vector<double> vecrand(6);
	for (int i = 0; i < 6; ++i) {
		vecrand[i] = std::rand() % 181 - 90;
	}
	return Node(vecrand);
}


double calc_dth2(double th, double goalth)
{
    // Return the value nearer to the goal state
	CSpaceConfig* space = CSpaceConfig::get_instance();
	int sym = space->getsymangle();
    double dif = std::abs(goalth - th);
    return (dif < sym/2) ? dif : sym - dif;
}


double calc_dist(State3D st, State3D goal)
{
    double dy2 = (st.y - goal.y) * (st.y - goal.y);
    double dth = calc_dth2(st.th, goal.th);
	double dth2 = dth * dth;

    double dist = std::sqrt(dy2 + dth2);
    return dist;
}


bool contain_yth(PointCloud pc, State3D goal)
{
    for (int i = 0; i < pc.size(); ++i) 
    if (pc.get(i).y == goal.y && pc.get(i).th == goal.th)    return true;
   
    return false;
}


bool contain_xyth(PointCloud pc, State3D goal)
{
    for (int i = 0; i < pc.size(); ++i) 
    if (pc.get(i).x == goal.x && 
		pc.get(i).y == goal.y && 
		pc.get(i).th == goal.th)    return true;
   
    return false;
}


void print_ICSs(std::vector<PointCloud> pcs)
{
	int num = 0;
	for (const auto& cls : pcs) {
		std::cout << num;	std::cout << ":" << cls.size() << std::endl;
		for (int i = 0; i < (int)cls.size(); ++i) {
			std::cout << "[";	std::cout << cls.get(i).x;	std::cout << ",";
			std::cout << cls.get(i).y;	std::cout << ",";	std::cout << cls.get(i).th;	std::cout << "],";
		//	if (i > 4)	break;
		}
		std::cout << std::endl << std::endl;
		++num;
	}
}

bool duplicate_check(PointCloud subject, std::vector<PointCloud> db)
{
	for(const auto& e : db){
		if(e.exist(subject.get(0)))	return true;
		else continue;
	}
	return false;
}

int read_threshold()
{
    bp::ptree pt;
    read_ini("config/ProblemDefine.ini", pt);
    boost::optional<double> carrier = pt.get_optional<double>("goal.epsilon");
    return carrier.get();
}

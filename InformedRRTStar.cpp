#include "InformedRRTStar.h"

#include <algorithm>
#include <chrono>
#include <cfloat>
#include <climits>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <limits>

#include "CFreeICS.h"
#include "Controller.h"
#include "pathsmooth.h"
#include <boost/property_tree/ini_parser.hpp>
#include <boost/property_tree/ptree.hpp>

// static bool cluster_size_ratio_ok(const PointCloud& a, const PointCloud& b)
// {
//     if(a.size() == 0 || b.size() == 0) return false;

//     double ratio;
//     if(a.size() < b.size()){
//         ratio = (double)b.size() / a.size();
//     } else {
//         ratio = (double)a.size() / b.size();
//     }

//     const double TH = 3.0; // ← RRTと同じ
//     return ratio < TH;
// }

static bool cluster_size_ratio_ok_vec(const std::vector<PointCloud>& A,
                                      const std::vector<PointCloud>& B)
{
    if(A.empty() || B.empty()) return false;

    // 最大クラスタサイズを比較
    size_t maxA = 0, maxB = 0;

    for(const auto& c : A) maxA = std::max(maxA, static_cast<size_t>(c.size()));
    for(const auto& c : B) maxB = std::max(maxB, static_cast<size_t>(c.size()));

    if(maxA == 0 || maxB == 0) return false;

    double ratio;
    if(maxA < maxB){
        ratio = (double)maxB / maxA;
    } else {
        ratio = (double)maxA / maxB;
    }

    const double TH = 3.0;
    return ratio < TH;
}

namespace {
    int origin = -1;
    constexpr double kEps = 1e-9;

    template <class T>
    int nearest_index_impl(const std::vector<T>& tree, Node q) {
        assert(!tree.empty());
        double best = std::numeric_limits<double>::infinity();
        int best_idx = 0;
        for (int i = 0; i < (int)tree.size(); ++i) {
            Node a = tree[i].node;
            double d = a.distance(q);
            if (d < best) {
                best = d;
                best_idx = i;
            }
        }
        return best_idx;
    }

    template <class T>
    std::vector<int> near_indices_impl(const std::vector<T>& tree, Node q, double radius) {
        std::vector<int> ids;
        for (int i = 0; i < (int)tree.size(); ++i) {
            Node a = tree[i].node;
            if (a.distance(q) <= radius) ids.push_back(i);
        }
        return ids;
    }

    template <class T>
    NodeList build_path_impl(const std::vector<T>& tree, int idx) {
        NodeList path;
        if (tree.empty() || idx < 0 || idx >= (int)tree.size()) return path;
        std::vector<Node> rev;
        while (idx != -1) {
            rev.push_back(tree[idx].node);
            idx = tree[idx].parent;
        }
        for (auto it = rev.rbegin(); it != rev.rend(); ++it) path.push_back(*it);
        return path;
    }

    double adaptive_radius(int n, double base_radius) {
        if (n <= 1) return base_radius;
        double scale = std::sqrt(std::log((double)n + 1.0) / (double)n);
        return std::max(1.0, base_radius * scale * 8.0);
    }

    double node_distance(Node a, Node b) {
        return a.distance(b);
    }

    Node sample_toward(Node from, Node to, double t = 0.5) {
        return from.interpolate(to, t);
    }

    double pointcloud_centroid_distance(const PointCloud& a, const PointCloud& b) {
        if (a.size() == 0 || b.size() == 0) return DBL_MAX;
        double ax = 0, ay = 0, at = 0;
        for (int i = 0; i < a.size(); ++i) { ax += a.get(i).x; ay += a.get(i).y; at += a.get(i).th; }
        double bx = 0, by = 0, bt = 0;
        for (int i = 0; i < b.size(); ++i) { bx += b.get(i).x; by += b.get(i).y; bt += b.get(i).th; }
        ax /= a.size(); ay /= a.size(); at /= a.size();
        bx /= b.size(); by /= b.size(); bt /= b.size();
        return std::sqrt((ax - bx) * (ax - bx) + (ay - by) * (ay - by) + (at - bt) * (at - bt));
    }
}


// ---------------- InformedRRTStar ----------------

void InformedRRTStar::set_strategy(CFO* cfo) { strategy = cfo; }

InformedRRTStar::InformedRRTStar()
    : tree(),
      garound(),
      strategy(new DfsCFO()),
      threshold(read_threshold()),
      cluster_distance_threshold(15.0),
      allowed_drop(0),
      ini_node(),
      fin_node(),
      have_solution(false),
      best_cost(DBL_MAX)
{
    std::ofstream log("../ICM_Log/icm.log", std::ios::app);
    log << "Forward epsilon : " << threshold << std::endl;
}

bool InformedRRTStar::initialize(Node ini)
{
    tree.clear();
    garound.clear();

    tree.push_back(StarNode(ini, origin, 0.0, PointCloud()));

    CFreeICS ics(ini);
    std::cout << ini << std::endl;
    if (!robot_update(ini)) return false;

    std::vector<PointCloud> init_CFree = ics.extract();
    if (init_CFree.empty()) return false;
    print_ICSs(init_CFree);

    std::cout << "Select the cluster:";
    int index = 0;
    std::cin >> index;
    assert(0 <= index && index < (int)init_CFree.size());

    tree[0].pc = init_CFree[index];
    return true;
}

Node InformedRRTStar::informed_sampling()
{
    Node mid = ini_node.interpolate(fin_node, 0.5);
    Node sample = generate_newnode();
    for (int i = 0; i < Node::dof; ++i) sample[i] = (sample[i] + mid[i]) * 0.5;
    return sample;
}

double InformedRRTStar::heuristic_cost(Node a, Node b) const { return node_distance(a, b); }

void InformedRRTStar::update_solution_cost()
{
    if (!tree.empty()) {
        best_cost = tree.back().cost;
        have_solution = true;
    }
}

Node InformedRRTStar::sampling(Node Rand)
{
    static int loop = 0;
    ++loop;

    if (have_solution) {
        Node biased = informed_sampling();
        std::cout << loop << ": " << biased << std::endl;
        return biased;
    }

    Node newnode;
    if (garound.size() < 10) {
        int idx = nearest_index(Rand);
        newnode = sample_toward(tree[idx].node, Rand, 0.5);
    } else {
        if (loop % 10 == 3 || loop % 10 == 7) {
            int idx = nearest_index(Rand);
            newnode = sample_toward(tree[idx].node, Rand, 0.5);
        } else {
            newnode = format_around(Rand);
        }
    }

    std::cout << loop << ": " << newnode << std::endl;
    return newnode;
}

Node InformedRRTStar::format_around(Node rand)
{
    double dist = DBL_MAX;
    int index = -1;
    for (int i = 0; i < (int)garound.size(); ++i) {
        Node a = tree[garound[i]].node;
        double tmp = a.distance(rand);
        if (dist > tmp) {
            index = garound[i];
            dist = tmp;
        }
    }

    if (index < 0) index = nearest_index(rand);
    return sample_toward(tree[index].node, rand, 0.5);
}

void InformedRRTStar::add_garound()
{
    garound.push_back((int)tree.size() - 1);
    std::cout << "around goal: " << garound.size() << std::endl;
}

bool InformedRRTStar::dfsconfig_valid(Node newnode, int parent_index, PointCloud& out_pc)
{
    std::vector<PointCloud> cfo = strategy->extract(tree[parent_index].pc, newnode);
    if ((int)cfo.size() == 1) {
        out_pc = cfo[0];
        return true;
    }
    return false;
}

int InformedRRTStar::nearest_index(Node query) const { return nearest_index_impl(tree, query); }

std::vector<int> InformedRRTStar::near_indices(Node query) const
{
    double radius = adaptive_radius((int)tree.size(), cluster_distance_threshold);
    return near_indices_impl(tree, query, radius);
}

double InformedRRTStar::edge_cost(Node a, Node b) const { return node_distance(a, b); }

double InformedRRTStar::path_cost(int idx) const
{
    if (idx < 0 || idx >= (int)tree.size()) return std::numeric_limits<double>::infinity();
    return tree[idx].cost;
}

int InformedRRTStar::choose_parent_index(Node newnode, const std::vector<int>& near_ids, PointCloud& out_pc)
{
    int nearest = nearest_index(newnode);
    if (near_ids.empty()) {
        if (!dfsconfig_valid(newnode, nearest, out_pc)) return -1;
        return nearest;
    }

    int best_parent = -1;
    double best_cost = std::numeric_limits<double>::infinity();
    PointCloud best_pc;

    for (int idx : near_ids) {
        PointCloud tmp_pc;
        if (!dfsconfig_valid(newnode, idx, tmp_pc)) continue;
        double cand_cost = tree[idx].cost + edge_cost(tree[idx].node, newnode);
        if (cand_cost < best_cost) {
            best_cost = cand_cost;
            best_parent = idx;
            best_pc = tmp_pc;
        }
    }

    if (best_parent != -1) {
        out_pc = best_pc;
        return best_parent;
    }

    if (!dfsconfig_valid(newnode, nearest, out_pc)) return -1;
    return nearest;
}

void InformedRRTStar::rewire_neighbors(int new_index, const std::vector<int>& near_ids)
{
    Node newnode = tree[new_index].node;
    std::vector<PointCloud> tmp_obj = tree[new_index].cfree_obj;
    for (int idx : near_ids) {
        if (idx == 0 || idx == new_index) continue;
        if (tree[idx].parent == new_index) continue;

        double through_new = tree[new_index].cost + edge_cost(newnode, tree[idx].node);
        if (through_new + kEps >= tree[idx].cost) continue;

        PointCloud tmp_pc;
        if (!dfsconfig_valid(tree[idx].node, new_index, tmp_pc)) continue;
        if (!cluster_size_ratio_ok_vec(tree[idx].cfree_obj, tmp_obj)) continue;

        tree[idx].parent = new_index;
        tree[idx].cost = through_new;
        tree[idx].pc = tmp_pc;
    }
}

NodeList InformedRRTStar::build_path() const { return build_path_impl(tree, (int)tree.size() - 1); }

double InformedRRTStar::calculate_cluster_distance(const PointCloud& old_pc, const PointCloud& new_pc)
{
    return pointcloud_centroid_distance(old_pc, new_pc);
}

InformedRRTStarGoalJudge InformedRRTStar::goal_judge(State3D goal)
{
    PointCloud pc = tree.back().pc;
    const double around_rate = 1.5;
    double max_dist = DBL_MIN;
    int xmin = INT_MAX, xmax = INT_MIN;

    for (int i = 0; i < pc.size(); ++i) {
        double dist = calc_dist(pc.get(i), goal);
        if (dist > threshold * around_rate) return InformedRRTStarGoalJudge::NotGoal;

        int xtmp = pc.get(i).x;
        if (xtmp < xmin) xmin = xtmp;
        if (xtmp > xmax) xmax = xtmp;
        if (max_dist < dist) max_dist = dist;
    }

    int delta_x = (xmax - xmin) / 2;
    max_dist = std::sqrt(max_dist * max_dist + delta_x * delta_x);
    if (max_dist > threshold * around_rate) return InformedRRTStarGoalJudge::NotGoal;

    std::cout << "max distance:" << max_dist << std::endl;
    if (max_dist > threshold) return InformedRRTStarGoalJudge::MiddleGoal;
    return InformedRRTStarGoalJudge::Goal;
}

NodeList InformedRRTStar::plan(Node ini, Node fin, State3D goal)
{
    ini_node = ini;
    fin_node = fin;
    have_solution = false;
    best_cost = DBL_MAX;

    rand_init();

    if (!initialize(ini)) {
        std::cout << "Invalid initial value was given." << std::endl;
        return NodeList();
    }

    auto start = std::chrono::system_clock::now();

    while (1) {
        static int i = 0;
        ++i;
        if (i >= 5000) exit(5963);

        Node Rand = generate_newnode();
        Node newnode = sampling(Rand);

        if (!robot_update(newnode)) continue;

        std::vector<int> near_ids = near_indices(newnode);
        PointCloud chosen_pc;
        int parent_index = choose_parent_index(newnode, near_ids, chosen_pc);
        if (parent_index < 0) continue;

        double new_cost = tree[parent_index].cost + edge_cost(tree[parent_index].node, newnode);
        tree.push_back(StarNode(newnode, parent_index, new_cost, chosen_pc));
        rewire_neighbors((int)tree.size() - 1, near_ids);

        InformedRRTStarGoalJudge flag = goal_judge(goal);
        if (flag == InformedRRTStarGoalJudge::MiddleGoal) add_garound();
        if (flag == InformedRRTStarGoalJudge::Goal) {
            update_solution_cost();
            break;
        }
    }

    auto end = std::chrono::system_clock::now();
    auto sec = std::chrono::duration_cast<std::chrono::seconds>(end - start).count();
    std::cout << "Elapsed time [s] :" << sec << std::endl;

    return build_path();
}

bool InformedRRTStar::debug() { return true; }

// ---------------- RevInformedRRTStar ----------------

void RevInformedRRTStar::set_strategy(CFO* cfo) { strategy = cfo; }

RevInformedRRTStar::RevInformedRRTStar()
    : tree(),
      strategy(new DfsCFO()),
      cluster_distance_threshold(15.0),
      allowed_drop(0),
      ini_node(),
      fin_node(),
      have_solution(false),
      best_cost(DBL_MAX)
{}

bool RevInformedRRTStar::initialize(Node fin)
{
    tree.clear();

    tree.push_back(StarNode(fin, origin, 0.0, std::vector<PointCloud>(), std::vector<PointCloud>()));

    CFreeICS ics(fin);
    for (int i = 0; i < 6; ++i) std::cout << fin.get_element(i) << ", ";
    std::cout << std::endl;

    if (!robot_update(fin)) return false;

    std::vector<PointCloud> fin_CFree = ics.extract();
    if (fin_CFree.empty()) return false;
    print_ICSs(fin_CFree);

    std::cout << "Select the cluster:";
    int index = -1;
    std::cin >> index;
    assert(0 <= index && index < (int)fin_CFree.size());
    PointCloud selected = fin_CFree[index];
    fin_CFree.erase(fin_CFree.begin() + index);

    tree[0].cfree_obj.push_back(selected);
    tree[0].cfree_del = fin_CFree;
    return true;
}

Node RevInformedRRTStar::sampling(Node Rand)
{
    static int loop = 0;
    ++loop;
    Node newnode = sample_toward(tree[nearest_index(Rand)].node, Rand, 0.5);
    std::cout << loop << ": " << newnode << std::endl;
    return newnode;
}

bool RevInformedRRTStar::continuous_check(PointCloud prev, PointCloud curr)
{
    double rate = 0.0;
    const double continue_rate = 3.0;
    if (prev.size() < curr.size()) rate = (double)curr.size() / prev.size();
    else rate = (double)prev.size() / curr.size();
    return rate < continue_rate;
}

bool RevInformedRRTStar::dfsconfig_valid(Node newnode,
                                         const std::vector<PointCloud>& parent_obj,
                                         std::vector<PointCloud>& out_obj,
                                         std::vector<PointCloud>& out_del,
                                         int parent_index)
{
    Controller* controller = Controller::get_instance();
    std::vector<PointCloud> cfree_obj;

    for (const auto& eo : parent_obj) {
        std::vector<PointCloud> cfree_obj_tmp = strategy->extract(eo, newnode);
        for (const auto& e : cfree_obj_tmp) {
            bool dup = false;
            for (const auto& existing : cfree_obj) {
                if (existing.overlap(e)) { dup = true; break; }
            }
            if (!dup) cfree_obj.push_back(e);
        }
    }

    Node parent = tree[parent_index].node;
    controller->robot_update(parent);
    for (auto it = cfree_obj.begin(); it != cfree_obj.end(); ) {
        std::vector<PointCloud> prev_real_cfree = strategy->extract(*it, parent);
        if (prev_real_cfree.size() != 1) {
            out_del.push_back(*it);
            it = cfree_obj.erase(it);
        } else {
            if (continuous_check(prev_real_cfree[0], *it)) ++it;
            else {
                out_del.push_back(*it);
                it = cfree_obj.erase(it);
            }
        }
    }
    controller->robot_update(newnode);

    if (cfree_obj.empty()) return false;
    out_obj = cfree_obj;
    return true;
}

int RevInformedRRTStar::nearest_index(Node query) const { return nearest_index_impl(tree, query); }

std::vector<int> RevInformedRRTStar::near_indices(Node query) const
{
    double radius = adaptive_radius((int)tree.size(), cluster_distance_threshold);
    return near_indices_impl(tree, query, radius);
}

double RevInformedRRTStar::edge_cost(Node a, Node b) const { return node_distance(a, b); }

double RevInformedRRTStar::path_cost(int idx) const
{
    if (idx < 0 || idx >= (int)tree.size()) return std::numeric_limits<double>::infinity();
    return tree[idx].cost;
}

int RevInformedRRTStar::choose_parent_index(Node newnode, const std::vector<int>& near_ids,
                                            const std::vector<PointCloud>& parent_obj,
                                            std::vector<PointCloud>& out_obj,
                                            std::vector<PointCloud>& out_del)
{
    int nearest = nearest_index(newnode);
    if (near_ids.empty()) {
        if (!dfsconfig_valid(newnode, parent_obj, out_obj, out_del, nearest)) return -1;
        return nearest;
    }

    int best_parent = -1;
    double best_cost = std::numeric_limits<double>::infinity();
    std::vector<PointCloud> best_obj, best_del;

    for (int idx : near_ids) {
        std::vector<PointCloud> tmp_obj, tmp_del;
        if (!dfsconfig_valid(newnode, tree[idx].cfree_obj, tmp_obj, tmp_del, idx)) continue;
        double cand_cost = tree[idx].cost + edge_cost(tree[idx].node, newnode);
        if (cand_cost < best_cost) {
            best_cost = cand_cost;
            best_parent = idx;
            best_obj = tmp_obj;
            best_del = tmp_del;
        }
    }

    if (best_parent != -1) {
        out_obj = best_obj;
        out_del = best_del;
        return best_parent;
    }

    if (!dfsconfig_valid(newnode, parent_obj, out_obj, out_del, nearest)) return -1;
    return nearest;
}

void RevInformedRRTStar::rewire_neighbors(int new_index, const std::vector<int>& near_ids)
{
    Node newnode = tree[new_index].node;
    for (int idx : near_ids) {
        if (idx == 0 || idx == new_index) continue;
        if (tree[idx].parent == new_index) continue;

        double through_new = tree[new_index].cost + edge_cost(newnode, tree[idx].node);
        if (through_new + kEps >= tree[idx].cost) continue;

        std::vector<PointCloud> tmp_obj, tmp_del;
        if (!dfsconfig_valid(tree[idx].node, tree[new_index].cfree_obj, tmp_obj, tmp_del, new_index)) continue;
        if (!cluster_size_ratio_ok_vec(tree[idx].cfree_obj, tmp_obj)) continue;

        tree[idx].parent = new_index;
        tree[idx].cost = through_new;
        tree[idx].cfree_obj = tmp_obj;
        tree[idx].cfree_del = tmp_del;
    }
}

NodeList RevInformedRRTStar::build_path() const
{
    NodeList path = build_path_impl(tree, (int)tree.size() - 1);
    path.reverse();
    return path;
}

double RevInformedRRTStar::calculate_cluster_distance(const PointCloud& old_pc, const PointCloud& new_pc)
{
    return pointcloud_centroid_distance(old_pc, new_pc);
}

InformedRRTStarGoalJudge RevInformedRRTStar::goal_judge(std::vector<PointCloud> pcs)
{
    static int maxi = 0;
    int nu = 1000;
    for (int i = 0; i < (int)pcs.size(); ++i) {
        if (maxi < pcs[i].size()) maxi = pcs[i].size();
        if (pcs[i].size() > nu) {
            std::ofstream log("../ICM_Log/icm.log", std::ios::app);
            log << "Reverse nu : " << nu << std::endl;
            return InformedRRTStarGoalJudge::Goal;
        }
    }
    return InformedRRTStarGoalJudge::NotGoal;
}

NodeList RevInformedRRTStar::plan(Node ini, Node fin, State3D goal)
{
    (void)ini;
    (void)goal;
    rand_init();

    if (!initialize(fin)) {
        std::cout << "Invalid initial value was given." << std::endl;
        return NodeList();
    }

    while (1) {
        static int i = 0;
        ++i;
        if (i >= 5000) exit(5963);

        Node Rand = generate_newnode();
        Node newnode = sampling(Rand);

        if (!robot_update(newnode)) continue;

        std::vector<int> near_ids = near_indices(newnode);
        std::vector<PointCloud> out_obj, out_del;
        int parent_index = choose_parent_index(newnode, near_ids, tree[nearest_index(newnode)].cfree_obj, out_obj, out_del);
        if (parent_index < 0) continue;

        double new_cost = tree[parent_index].cost + edge_cost(tree[parent_index].node, newnode);
        tree.push_back(StarNode(newnode, parent_index, new_cost, out_obj, out_del));
        rewire_neighbors((int)tree.size() - 1, near_ids);

        if (goal_judge(tree.back().cfree_obj) == InformedRRTStarGoalJudge::Goal) break;
    }

    NodeList path = build_path();
    if (path.size() > 0) {
        PointCloud pre_cfo = tree.back().cfree_obj[0];
        for (int i = 1; i < path.size(); ++i) {
            Node check = path.get(i);
            if (!robot_update(check)) assert(false);
            DfsCFO dfs;
            std::vector<PointCloud> cfo_now = dfs.extract(pre_cfo, check);
            if ((int)cfo_now.size() != 1) assert(false);
            pre_cfo = cfo_now[0];
        }
    }
    return path;
}

// ---------------- InformedRRTStarConnect ----------------

InformedRRTStarConnect::InformedRRTStarConnect()
    : s_tree(),
      g_tree(),
      s_threshold(read_threshold()),
      cluster_distance_threshold(15.0),
      strategy(new DfsCFO()),
      allowed_drop(0),
      ini_node(),
      fin_node(),
      have_solution(false),
      best_cost(DBL_MAX)
{
    std::ofstream log("../ICM_Log/icm.log", std::ios::app);
    log << "Forward epsilon : " << s_threshold << std::endl;
}

Node InformedRRTStarConnect::informed_sampling(Node a, Node b) const
{
    if (have_solution) {
        Node mid = a.interpolate(b, 0.5);
        Node sample = generate_newnode();
        for (int i = 0; i < Node::dof; ++i) sample[i] = (sample[i] + mid[i]) * 0.5;
        return sample;
    }
    return generate_newnode();
}

Node InformedRRTStarConnect::informed_sampling_near(Node anchor) const
{
    Node sample = generate_newnode();
    return anchor.interpolate(sample, 0.5);
}

double InformedRRTStarConnect::heuristic_cost(Node a, Node b) const { return node_distance(a, b); }

void InformedRRTStarConnect::update_solution_cost()
{
    if (!s_tree.empty() && !g_tree.empty()) {
        best_cost = s_tree.back().cost + g_tree.back().cost;
        have_solution = true;
    }
}

bool InformedRRTStarConnect::initialize(Node ini, Node fin)
{
    s_tree.clear();
    g_tree.clear();

    CFreeICS ini_ics(ini), fin_ics(fin);
    s_tree.push_back(FNode(ini, origin, 0.0, PointCloud()));
    g_tree.push_back(RNode(fin, origin, 0.0, std::vector<PointCloud>(), std::vector<PointCloud>()));

    int index1 = -1, index2 = -1;

    if (!robot_update(ini)) return false;
    std::vector<PointCloud> ini_CFree = ini_ics.extract();
    if (ini_CFree.empty()) return false;
    print_ICSs(ini_CFree);
    std::cout << "Select a cluster (start configuration):";
    std::cin >> index1;
    assert(0 <= index1 && index1 < (int)ini_CFree.size());
    s_tree[0].pc = ini_CFree[index1];

    if (!robot_update(fin)) return false;
    std::vector<PointCloud> fin_CFree = fin_ics.extract();
    if (fin_CFree.empty()) return false;
    print_ICSs(fin_CFree);
    std::cout << "Select a cluster (goal configuration):";
    std::cin >> index2;
    assert(0 <= index2 && index2 < (int)fin_CFree.size());
    PointCloud selected = fin_CFree[index2];
    fin_CFree.erase(fin_CFree.begin() + index2);
    g_tree[0].cfree_obj.push_back(selected);
    g_tree[0].cfree_del = fin_CFree;

    return true;
}

bool InformedRRTStarConnect::caging_validation_sconf(Node node, PointCloud& out_pc, int parent_index)
{
    std::vector<PointCloud> cfo = strategy->extract(s_tree[parent_index].pc, node);
    if ((int)cfo.size() == 1) {
        out_pc = cfo[0];
        return true;
    }
    return false;
}

bool InformedRRTStarConnect::caging_validation_gconf(Node node,
                                                     const std::vector<PointCloud>& prev_cfree_obj,
                                                     std::vector<PointCloud>& out_obj,
                                                     std::vector<PointCloud>& out_del,
                                                     int parent_index)
{
    Controller* controller = Controller::get_instance();
    std::vector<PointCloud> cfree_obj;

    for (const auto& eo : prev_cfree_obj) {
        std::vector<PointCloud> cfree_obj_tmp = strategy->extract(eo, node);
        for (const auto& e : cfree_obj_tmp) {
            bool flag = false;
            for (const auto& existing : cfree_obj) {
                if (existing.overlap(e)) { flag = true; break; }
            }
            if (!flag) cfree_obj.push_back(e);
        }
    }

    Node parent = g_tree[parent_index].node;
    controller->robot_update(parent);
    for (auto it = cfree_obj.begin(); it != cfree_obj.end(); ) {
        std::vector<PointCloud> prev_real_cfree = strategy->extract(*it, parent);
        if (prev_real_cfree.size() != 1) {
            out_del.push_back(*it);
            it = cfree_obj.erase(it);
        } else {
            ++it;
        }
    }
    controller->robot_update(node);

    if (cfree_obj.empty()) return false;
    out_obj = cfree_obj;
    return true;
}

int InformedRRTStarConnect::nearest_s_index(Node query) const { return nearest_index_impl(s_tree, query); }
int InformedRRTStarConnect::nearest_g_index(Node query) const { return nearest_index_impl(g_tree, query); }
std::vector<int> InformedRRTStarConnect::near_s_indices(Node query) const { return near_indices_impl(s_tree, query, adaptive_radius((int)s_tree.size(), cluster_distance_threshold)); }
std::vector<int> InformedRRTStarConnect::near_g_indices(Node query) const { return near_indices_impl(g_tree, query, adaptive_radius((int)g_tree.size(), cluster_distance_threshold)); }
double InformedRRTStarConnect::edge_cost(Node a, Node b) const { return node_distance(a, b); }

double InformedRRTStarConnect::path_cost_s(int idx) const { return (idx >= 0 && idx < (int)s_tree.size()) ? s_tree[idx].cost : std::numeric_limits<double>::infinity(); }
double InformedRRTStarConnect::path_cost_g(int idx) const { return (idx >= 0 && idx < (int)g_tree.size()) ? g_tree[idx].cost : std::numeric_limits<double>::infinity(); }

int InformedRRTStarConnect::choose_s_parent(Node newnode, const std::vector<int>& near_ids, PointCloud& out_pc)
{
    int nearest = nearest_s_index(newnode);
    if (near_ids.empty()) {
        if (!caging_validation_sconf(newnode, out_pc, nearest)) return -1;
        return nearest;
    }

    int best_parent = -1;
    double best_cost = std::numeric_limits<double>::infinity();
    PointCloud best_pc;
    for (int idx : near_ids) {
        PointCloud tmp_pc;
        if (!caging_validation_sconf(newnode, tmp_pc, idx)) continue;
        double cand_cost = s_tree[idx].cost + edge_cost(s_tree[idx].node, newnode);
        if (cand_cost < best_cost) {
            best_cost = cand_cost;
            best_parent = idx;
            best_pc = tmp_pc;
        }
    }
    if (best_parent != -1) { out_pc = best_pc; return best_parent; }
    if (!caging_validation_sconf(newnode, out_pc, nearest)) return -1;
    return nearest;
}

int InformedRRTStarConnect::choose_g_parent(Node newnode, const std::vector<int>& near_ids,
                                            const std::vector<PointCloud>& parent_obj,
                                            std::vector<PointCloud>& out_obj,
                                            std::vector<PointCloud>& out_del)
{
    int nearest = nearest_g_index(newnode);
    if (near_ids.empty()) {
        if (!caging_validation_gconf(newnode, parent_obj, out_obj, out_del, nearest)) return -1;
        return nearest;
    }

    int best_parent = -1;
    double best_cost = std::numeric_limits<double>::infinity();
    std::vector<PointCloud> best_obj, best_del;
    for (int idx : near_ids) {
        std::vector<PointCloud> tmp_obj, tmp_del;
        if (!caging_validation_gconf(newnode, g_tree[idx].cfree_obj, tmp_obj, tmp_del, idx)) continue;
        double cand_cost = g_tree[idx].cost + edge_cost(g_tree[idx].node, newnode);
        if (cand_cost < best_cost) {
            best_cost = cand_cost;
            best_parent = idx;
            best_obj = tmp_obj;
            best_del = tmp_del;
        }
    }

    if (best_parent != -1) { out_obj = best_obj; out_del = best_del; return best_parent; }
    if (!caging_validation_gconf(newnode, parent_obj, out_obj, out_del, nearest)) return -1;
    return nearest;
}

void InformedRRTStarConnect::rewire_s_neighbors(int new_index, const std::vector<int>& near_ids)
{
    Node newnode = s_tree[new_index].node;
    std::vector<PointCloud> tmp_obj = s_tree[new_index].cfree_obj;
    for (int idx : near_ids) {
        if (idx == 0 || idx == new_index) continue;
        if (s_tree[idx].parent == new_index) continue;
        double through_new = s_tree[new_index].cost + edge_cost(newnode, s_tree[idx].node);
        if (through_new + kEps >= s_tree[idx].cost) continue;
        PointCloud tmp_pc;
        if (!caging_validation_sconf(s_tree[idx].node, tmp_pc, new_index)) continue;
                if (!cluster_size_ratio_ok_vec(s_tree[idx].cfree_obj, tmp_obj)) continue;

        s_tree[idx].parent = new_index;
        s_tree[idx].cost = through_new;
        s_tree[idx].pc = tmp_pc;
    }
}

void InformedRRTStarConnect::rewire_g_neighbors(int new_index, const std::vector<int>& near_ids)
{
    Node newnode = g_tree[new_index].node;
    for (int idx : near_ids) {
        if (idx == 0 || idx == new_index) continue;
        if (g_tree[idx].parent == new_index) continue;
        double through_new = g_tree[new_index].cost + edge_cost(newnode, g_tree[idx].node);
        if (through_new + kEps >= g_tree[idx].cost) continue;
        std::vector<PointCloud> tmp_obj, tmp_del;
        if (!caging_validation_gconf(g_tree[idx].node, g_tree[new_index].cfree_obj, tmp_obj, tmp_del, new_index)) continue;
                if (!cluster_size_ratio_ok_vec(g_tree[idx].cfree_obj, tmp_obj)) continue;

        g_tree[idx].parent = new_index;
        g_tree[idx].cost = through_new;
        g_tree[idx].cfree_obj = tmp_obj;
        g_tree[idx].cfree_del = tmp_del;
    }
}

InformedRRTStarGoalJudge InformedRRTStarConnect::goal_sconf(State3D goal)
{
    PointCloud pc = s_tree.back().pc;
    double max_dist = DBL_MIN;
    int xmin = INT_MAX, xmax = INT_MIN;

    for (int i = 0; i < pc.size(); ++i) {
        double dist = calc_dist(pc.get(i), goal);
        if (dist > s_threshold) return InformedRRTStarGoalJudge::NotGoal;
        int xtmp = pc.get(i).x;
        if (xtmp < xmin) xmin = xtmp;
        if (xtmp > xmax) xmax = xtmp;
        if (max_dist < dist) max_dist = dist;
    }

    int delta_x = (xmax - xmin) / 2;
    max_dist = std::sqrt(max_dist * max_dist + delta_x * delta_x);
    if (max_dist > s_threshold) return InformedRRTStarGoalJudge::NotGoal;
    if (!contain_yth(pc, goal)) return InformedRRTStarGoalJudge::NotGoal;
    return InformedRRTStarGoalJudge::SGoal;
}

InformedRRTStarGoalJudge InformedRRTStarConnect::goal_connect(int bef_index, int aft_index)
{
    Node bef_node = s_tree[bef_index].node;
    Node aft_node = g_tree[aft_index].node;
    double dist = bef_node.distance(aft_node);
    if (dist > 1.0) return InformedRRTStarGoalJudge::NotGoal;

    for (const auto& afree : g_tree[aft_index].cfree_obj) {
        if (!s_tree[bef_index].pc.overlap(afree)) return InformedRRTStarGoalJudge::NotGoal;
    }

    std::vector<PointCloud> overlap_cfree = strategy->extract(s_tree[bef_index].pc, g_tree[aft_index].node);
    if ((int)overlap_cfree.size() == 1) return InformedRRTStarGoalJudge::Connect;
    return InformedRRTStarGoalJudge::NotGoal;
}

InformedRRTStarGoalJudge InformedRRTStarConnect::goal_gconf(std::vector<PointCloud> cfo)
{
    static int maxi = 0;
    int nu = 1000;
    for (int i = 0; i < (int)cfo.size(); ++i) {
        if (maxi < cfo[i].size()) maxi = cfo[i].size();
        if (cfo[i].size() > nu) return InformedRRTStarGoalJudge::GGoal;
    }
    return InformedRRTStarGoalJudge::NotGoal;
}

InformedRRTStarGoalJudge InformedRRTStarConnect::sconf_goaljudge(State3D goal, int bef, int aft)
{
    if (goal_connect(bef, aft) == InformedRRTStarGoalJudge::Connect) return InformedRRTStarGoalJudge::Connect;
    return InformedRRTStarGoalJudge::NotGoal;
}

InformedRRTStarGoalJudge InformedRRTStarConnect::gconf_goaljudge(std::vector<PointCloud> cfo, int bef, int aft)
{
    if (goal_gconf(cfo) == InformedRRTStarGoalJudge::GGoal) return InformedRRTStarGoalJudge::GGoal;
    if (goal_connect(bef, aft) == InformedRRTStarGoalJudge::Connect) return InformedRRTStarGoalJudge::Connect;
    return InformedRRTStarGoalJudge::NotGoal;
}

bool InformedRRTStarConnect::extend_limit(Node n1, Node n2)
{
    return n1.distance(n2) < 1.0;
}

NodeList InformedRRTStarConnect::path_concat()
{
    return path_concat((int)s_tree.size() - 1, (int)g_tree.size() - 1);
}

NodeList InformedRRTStarConnect::path_concat(int sindex, int gindex)
{
    NodeList spath = build_path_impl(s_tree, sindex);
    NodeList gpath = build_path_impl(g_tree, gindex);
    gpath.reverse();
    spath.concat(gpath);
    return spath;
}

NodeList InformedRRTStarConnect::make_path(InformedRRTStarGoalJudge flag)
{
    if (flag == InformedRRTStarGoalJudge::SGoal) return build_path_impl(s_tree, (int)s_tree.size() - 1);
    if (flag == InformedRRTStarGoalJudge::GGoal) {
        NodeList path = build_path_impl(g_tree, (int)g_tree.size() - 1);
        path.reverse();
        return path;
    }
    if (flag == InformedRRTStarGoalJudge::Connect) return path_concat();
    return NodeList();
}

NodeList InformedRRTStarConnect::make_path(InformedRRTStarGoalJudge flag, int sindex, int gindex)
{
    if (flag == InformedRRTStarGoalJudge::SGoal) return build_path_impl(s_tree, sindex);
    if (flag == InformedRRTStarGoalJudge::GGoal) {
        NodeList path = build_path_impl(g_tree, gindex);
        path.reverse();
        return path;
    }
    if (flag == InformedRRTStarGoalJudge::Connect) return path_concat(sindex, gindex);
    return NodeList();
}

double InformedRRTStarConnect::calculate_cluster_distance(const PointCloud& old_pc, const PointCloud& new_pc)
{
    return pointcloud_centroid_distance(old_pc, new_pc);
}

bool InformedRRTStarConnect::sconf_update() { return true; }
bool InformedRRTStarConnect::gconf_update() { return true; }

NodeList InformedRRTStarConnect::plan(Node ini, Node fin, State3D goal)
{
    ini_node = ini;
    fin_node = fin;
    have_solution = false;
    best_cost = DBL_MAX;

    rand_init();
    if (!initialize(ini, fin)) {
        std::cout << "Invalid initial value was given." << std::endl;
        return NodeList();
    }

    while (1) {
        static int i = 0;
        ++i;
        if (i >= 5000) exit(5963);

        Node Rand = have_solution ? informed_sampling(ini_node, fin_node) : generate_newnode();

        if (s_tree.size() <= g_tree.size()) {
            Node newnode = sample_toward(s_tree[nearest_s_index(Rand)].node, Rand, 0.5);
            if (!robot_update(newnode)) continue;

            std::vector<int> near_ids = near_s_indices(newnode);
            PointCloud chosen_pc;
            int parent_index = choose_s_parent(newnode, near_ids, chosen_pc);
            if (parent_index < 0) continue;

            double new_cost = s_tree[parent_index].cost + edge_cost(s_tree[parent_index].node, newnode);
            s_tree.push_back(FNode(newnode, parent_index, new_cost, chosen_pc));
            rewire_s_neighbors((int)s_tree.size() - 1, near_ids);

            InformedRRTStarGoalJudge flag = sconf_goaljudge(goal, (int)s_tree.size() - 1, nearest_g_index(newnode));
            if (flag != InformedRRTStarGoalJudge::NotGoal) return make_path(flag, (int)s_tree.size() - 1, nearest_g_index(newnode));
        } else {
            Node newnode = sample_toward(g_tree[nearest_g_index(Rand)].node, Rand, 0.5);
            if (!robot_update(newnode)) continue;

            std::vector<int> near_ids = near_g_indices(newnode);
            std::vector<PointCloud> out_obj, out_del;
            int parent_index = choose_g_parent(newnode, near_ids, g_tree[nearest_g_index(newnode)].cfree_obj, out_obj, out_del);
            if (parent_index < 0) continue;

            double new_cost = g_tree[parent_index].cost + edge_cost(g_tree[parent_index].node, newnode);
            g_tree.push_back(RNode(newnode, parent_index, new_cost, out_obj, out_del));
            rewire_g_neighbors((int)g_tree.size() - 1, near_ids);

            InformedRRTStarGoalJudge flag = gconf_goaljudge(g_tree.back().cfree_obj, nearest_s_index(newnode), (int)g_tree.size() - 1);
            if (flag != InformedRRTStarGoalJudge::NotGoal) return make_path(flag, nearest_s_index(newnode), (int)g_tree.size() - 1);
        }
    }
}


#pragma once

#include <vector>
#include <cassert>

#include "RRT.h"

enum class InformedRRTStarGoalJudge {
    NotGoal,
    MiddleGoal,
    Goal,
    Connect,
    SGoal,
    GGoal
};

class InformedRRTStar : public Planner
{
private:
    struct StarNode {
        Node node;
        int parent;
        double cost;
        PointCloud pc;
        std::vector<PointCloud> cfree_obj;

        StarNode(const Node& n, int p, double c, const PointCloud& cloud)
            : node(n), parent(p), cost(c), pc(cloud) {}
    };

    std::vector<StarNode> tree;
    std::vector<int> garound;
    CFO* strategy;
    int threshold;
    double cluster_distance_threshold;
    int allowed_drop;

    Node ini_node;
    Node fin_node;
    bool have_solution;
    double best_cost;

    void set_strategy(CFO* cfo);
    bool initialize(Node ini);

    Node sampling(Node Rand);
    Node format_around(Node rand);
    void add_garound();
    Node informed_sampling();

    bool dfsconfig_valid(Node newnode, int parent_index, PointCloud& out_pc);

    InformedRRTStarGoalJudge goal_judge(State3D goal);

    int nearest_index(Node query) const;
    std::vector<int> near_indices(Node query) const;
    double edge_cost(Node a, Node b) const;
    double path_cost(int idx) const;

    int choose_parent_index(Node newnode, const std::vector<int>& near_ids, PointCloud& out_pc);
    void rewire_neighbors(int new_index, const std::vector<int>& near_ids);

    NodeList build_path() const;
    double calculate_cluster_distance(const PointCloud& old_pc, const PointCloud& new_pc);
    double heuristic_cost(Node a, Node b) const;
    void update_solution_cost();

public:
    InformedRRTStar();
    NodeList plan(Node ini, Node fin, State3D goal);
    bool debug();
};

class RevInformedRRTStar : public Planner
{
private:
    struct StarNode {
        Node node;
        int parent;
        double cost;
        std::vector<PointCloud> cfree_obj;
        std::vector<PointCloud> cfree_del;

        StarNode(const Node& n, int p, double c,
                 const std::vector<PointCloud>& obj,
                 const std::vector<PointCloud>& del)
            : node(n), parent(p), cost(c), cfree_obj(obj), cfree_del(del) {}
    };

    std::vector<StarNode> tree;
    CFO* strategy;
    double cluster_distance_threshold;
    int allowed_drop;

    Node ini_node;
    Node fin_node;
    bool have_solution;
    double best_cost;

    void set_strategy(CFO* cfo);
    bool initialize(Node fin);

    Node sampling(Node Rand);
    bool dfsconfig_valid(Node newnode,
                         const std::vector<PointCloud>& parent_obj,
                         std::vector<PointCloud>& out_obj,
                         std::vector<PointCloud>& out_del,
                         int parent_index);

    bool continuous_check(PointCloud prev, PointCloud curr);

    InformedRRTStarGoalJudge goal_judge(std::vector<PointCloud> pcs);

    int nearest_index(Node query) const;
    std::vector<int> near_indices(Node query) const;
    double edge_cost(Node a, Node b) const;
    double path_cost(int idx) const;

    int choose_parent_index(Node newnode, const std::vector<int>& near_ids,
                            const std::vector<PointCloud>& parent_obj,
                            std::vector<PointCloud>& out_obj,
                            std::vector<PointCloud>& out_del);

    void rewire_neighbors(int new_index, const std::vector<int>& near_ids);

    NodeList build_path() const;
    double calculate_cluster_distance(const PointCloud& old_pc, const PointCloud& new_pc);

public:
    RevInformedRRTStar();
    NodeList plan(Node ini, Node fin, State3D goal);
};

class InformedRRTStarConnect : public Planner
{
private:
    struct FNode {
        Node node;
        int parent;
        double cost;
        PointCloud pc;
        std::vector<PointCloud> cfree_obj;

        FNode(const Node& n, int p, double c, const PointCloud& cloud)
            : node(n), parent(p), cost(c), pc(cloud) {}
    };

    struct RNode {
        Node node;
        int parent;
        double cost;
        std::vector<PointCloud> cfree_obj;
        std::vector<PointCloud> cfree_del;

        RNode(const Node& n, int p, double c,
              const std::vector<PointCloud>& obj,
              const std::vector<PointCloud>& del)
            : node(n), parent(p), cost(c), cfree_obj(obj), cfree_del(del) {}
    };

    std::vector<FNode> s_tree;
    std::vector<RNode> g_tree;
    int s_threshold;
    double cluster_distance_threshold;
    CFO* strategy;
    int allowed_drop;

    Node ini_node;
    Node fin_node;
    bool have_solution;
    double best_cost;

    bool initialize(Node ini, Node fin);
    bool sconf_update();
    bool gconf_update();

    InformedRRTStarGoalJudge sconf_goaljudge(State3D goal, int bef, int aft);
    InformedRRTStarGoalJudge gconf_goaljudge(std::vector<PointCloud> cfo, int bef, int aft);

    bool caging_validation_sconf(Node node, PointCloud& out_pc, int parent_index);
    bool caging_validation_gconf(Node node,
                                 const std::vector<PointCloud>& prev_cfree_obj,
                                 std::vector<PointCloud>& out_obj,
                                 std::vector<PointCloud>& out_del,
                                 int parent_index);

    InformedRRTStarGoalJudge goal_sconf(State3D goal);
    InformedRRTStarGoalJudge goal_connect(int bef_index, int aft_index);
    InformedRRTStarGoalJudge goal_gconf(std::vector<PointCloud> cfo);

    NodeList make_path(InformedRRTStarGoalJudge flag);
    NodeList make_path(InformedRRTStarGoalJudge flag, int sindex, int gindex);

    bool extend_limit(Node n1, Node n2);

    NodeList path_concat();
    NodeList path_concat(int sindex, int gindex);

    int nearest_s_index(Node query) const;
    int nearest_g_index(Node query) const;
    std::vector<int> near_s_indices(Node query) const;
    std::vector<int> near_g_indices(Node query) const;

    double edge_cost(Node a, Node b) const;
    double path_cost_s(int idx) const;
    double path_cost_g(int idx) const;

    int choose_s_parent(Node newnode, const std::vector<int>& near_ids, PointCloud& out_pc);
    int choose_g_parent(Node newnode, const std::vector<int>& near_ids,
                        const std::vector<PointCloud>& parent_obj,
                        std::vector<PointCloud>& out_obj,
                        std::vector<PointCloud>& out_del);

    void rewire_s_neighbors(int new_index, const std::vector<int>& near_ids);
    void rewire_g_neighbors(int new_index, const std::vector<int>& near_ids);

    double calculate_cluster_distance(const PointCloud& old_pc, const PointCloud& new_pc);
    Node informed_sampling(Node a, Node b) const;
    Node informed_sampling_near(Node anchor) const;
    void update_solution_cost();
    double heuristic_cost(Node a, Node b) const;

public:
    InformedRRTStarConnect();
    NodeList plan(Node ini, Node fin, State3D goal);
};


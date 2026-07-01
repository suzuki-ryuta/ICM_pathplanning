// pathshortcut.h
#pragma once

#include <vector>
#include <string>

#include "Node.h"
#include "PointCloud.h"
#include "SpectralUtil.h"

class PathShortcut {
private:
    NodeList path;
    int trials;

    bool robot_update(const Node& nd);
    bool caging_valid(PointCloud& prev, const Node& now, const Node& next);
    void validate_and_fix(NodeList& path);


public:
    PathShortcut(const NodeList& input_path, int num_trials = 100);
    NodeList shortcut();

    void shrink_phase(NodeList& result,
                      bool      check_reset);  // ← true: クラスタ再抽出
    // void set_trials(int t) { trials = t; }
};
double overlap_ratio(const PointCloud& a, const PointCloud& b);
double angle_diff(const Node& n1, const Node& n2);

// pathsmooth.h の最後に追記
NodeList csv_to_nodelist(std::string fn);

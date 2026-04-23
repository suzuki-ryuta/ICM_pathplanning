// pathshortcut.cpp
#include "pathshortcut.h"
#include "Controller.h"
#include "CFreeICS.h"
#include "CFree.h"
#include <random>
#include <iostream>
#include <cassert>

#include <algorithm>
#include <cmath>
#include <chrono>      // 計算時間タイムアウト用
#include <iterator>  // ★ これを追加！
#include "GraphUtils.h"  // Laplacian 関連の計算モジュール
#include <unordered_map>
#include <fstream>   // ★追加

// ★コンストラクタの実装（忘れずに）--------------------------------
PathShortcut::PathShortcut(const NodeList& input_path, int num_trials)
    : path(input_path), trials(num_trials) {}
//以下，ロボット衝突チェック------------------------------------------
bool PathShortcut::robot_update(const Node& newnode) {
    Controller* controller = Controller::get_instance();
    controller->robot_update(newnode);
    return !(controller->RintersectR(newnode) || controller->RintersectW(newnode));
}
//---------固定------------------------------------------------------

/* -------------- caging_valid は未使用だが残しておく -------------- */
bool PathShortcut::caging_valid(PointCloud& prev, const Node& now, const Node&)
{
    auto cls = CFreeICS(now).extract();
    if (cls.size() != 1) return false;
    prev = cls[0];
    return true;
}
//====方法1用=========================================================
Vector2D<double> compute_centroid(const PointCloud& pc) {
    Vector2D<double> sum(0.0, 0.0);
    for (int i = 0; i < pc.size(); ++i) {
        State3D s = pc.get(i);
        sum.x += s.x;
        sum.y += s.y;
    }
    if (pc.size() > 0) {
        sum.x /= pc.size();
        sum.y /= pc.size();
    }
    return sum;
}
// --- クラスタ識別キャッシュ用構造 ---
std::string hash_cluster(const PointCloud& pc) {
    std::string key;
    for (const auto& pt : pc) {
        key += std::to_string(pt.x) + "_" + std::to_string(pt.y) + "_" + std::to_string(pt.th) + "|";
    }
    return std::to_string(std::hash<std::string>{}(key));
}
// --- 固有値距離計算のためのキャッシュ ---
constexpr int MAX_TRIALS = 600;

struct BoundingBox {
    double x_min, x_max, y_min, y_max;
};

// --- ★クラスタのバウンディングボックスを計算する関数 ---
BoundingBox compute_bounding_box(const PointCloud& pc) {
    BoundingBox box;
    if (pc.size() == 0) {
        box.x_min = box.y_min = 0;
        box.x_max = box.y_max = 0;
        return box;
    }
    box.x_min = box.x_max = pc.get(0).x;
    box.y_min = box.y_max = pc.get(0).y;

    for (int i = 1; i < pc.size(); ++i) {
        State3D s = pc.get(i);
        if (s.x < box.x_min) box.x_min = s.x;
        if (s.x > box.x_max) box.x_max = s.x;
        if (s.y < box.y_min) box.y_min = s.y;
        if (s.y > box.y_max) box.y_max = s.y;
    }
    return box;
}

// --- ★バウンディングボックス同士が max_move_dist 以内にあるかを判定 ---
bool overlap_with_margin(const BoundingBox& a, const BoundingBox& b, double margin) {
    return !(a.x_max + margin < b.x_min - margin ||
             a.x_min - margin > b.x_max + margin ||
             a.y_max + margin < b.y_min - margin ||
             a.y_min - margin > b.y_max + margin);
}

NodeList PathShortcut::shortcut()
{
    std::mt19937 rng(0);  // 固定シードで再現性あり
    NodeList result = path;

    if (result.size() < 3) {
        std::cout << "[INFO] path too short to shortcut.\n";
        return result;
    }

    const double ANGLE_THRESHOLD = 5.0;  // 5度j候補用
    // const int steps = 20;
    const double STEP_MAX_ANGLE = 1.0;   // 1 step あたり最大角度変化 (度). 必要に応じて変更
    const int MIN_STEPS = 2;             // 最低分割数

    int successful = 0;

    // ★ログファイル
    std::ofstream log_file("timelog.txt");

    for (int trial = 0; trial < trials; ++trial) {
        if (result.size() < 3) break;

        int i = rng() % (result.size() - 2);  // i = 0～size-3
        Node n1 = result[i];

        // --- j候補の抽出 ---
        std::vector<int> j_candidates;
        for (int j = i + 2; j < result.size(); ++j) {
            Node n2 = result[j];
            bool within_limit = true;
            for (int d = 0; d < Node::dof; ++d) {
                if (std::abs(n1[d] - n2[d]) > ANGLE_THRESHOLD) {
                    within_limit = false;
                    break;
                }
            }
            if (within_limit) j_candidates.push_back(j);
        }
        if (j_candidates.empty()) continue;

        std::uniform_int_distribution<> dist(0, j_candidates.size() - 1);
        int j = j_candidates[dist(rng)];
        Node n2 = result[j];

        bool valid = true;
        PointCloud used_cluster;

        // --- (1) 最大変化点から探索距離を決定 -----------------
        double max_move_dist = 0.0;  // ← 新規追加

        // n1 と n2 の各クラスタ点のうち、最大の 2D 変化距離を求める
        {
            auto cls_i = CFreeICS(n1).extract();
            auto cls_j = CFreeICS(n2).extract();
            if (!cls_i.empty() && !cls_j.empty()) {
                const PointCloud& pc_i = cls_i[0]; // 代表クラスタ（あるいは used_cluster）
                const PointCloud& pc_j = cls_j[0];

                // 点数が異なる場合に備え、最小サイズに合わせて比較
                int N = std::min(pc_i.size(), pc_j.size());
                for (int idx = 0; idx < N; ++idx) {
                    State3D s_i = pc_i.get(idx);
                    State3D s_j = pc_j.get(idx);
                    double dx = s_i.x - s_j.x;
                    double dy = s_i.y - s_j.y;
                    double dist = std::sqrt(dx * dx + dy * dy);
                    if (dist > max_move_dist) max_move_dist = dist;
                }
            }
        }
        // --- ★ 最小値を設定 ---
        if (max_move_dist < 10.0) max_move_dist = 10.0;
        std::cout << "[INFO] max_move_dist = " << max_move_dist << " mm\n";

        // --- n1 のクラスタ抽出 ---
        auto t0 = std::chrono::steady_clock::now();
        CFreeICS ics(n1);
        auto cls = ics.extract();
        auto t1 = std::chrono::steady_clock::now();
        log_file << "[TIME] extract(n1): "
                 << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count()
                 << " ms\n";

        if (cls.empty()) continue;

        used_cluster = *std::max_element(
            cls.begin(), cls.end(),
            [](const PointCloud& a, const PointCloud& b) {
                return a.size() < b.size();
            });

        // n1 の固有値計算
        t0 = std::chrono::steady_clock::now();
        std::vector<double> eig_used = compute_laplacian_eigenvalues(used_cluster, 5);
        t1 = std::chrono::steady_clock::now();
        log_file << "[TIME] eig(n1): "
                 << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count()
                 << " ms\n";
        
        // compute max delta angle across DOF
        double max_dtheta = 0.0;
        for (int d = 0; d < Node::dof; ++d) {
            double diff = std::abs(n1[d] - n2[d]);
            if (diff > max_dtheta) max_dtheta = diff;
        }

        // 単位に注意：もしノードがラジアンなら STEP_MAX_ANGLE をラジアンにするか変換する
        int steps = std::max(MIN_STEPS, static_cast<int>(std::ceil(max_dtheta / STEP_MAX_ANGLE)));
    
    

        // --- 補間ループ ---
        t0 = std::chrono::steady_clock::now();
        for (int k = 1; k < steps; ++k) {
            double t = static_cast<double>(k) / steps;
            Node mid = n1.interpolate(n2, t);

            auto tA = std::chrono::steady_clock::now();
            if (!robot_update(mid)) {
                valid = false;
                break;
            }
            auto tB = std::chrono::steady_clock::now();
            log_file << "[TIME] robot_update: "
                     << std::chrono::duration_cast<std::chrono::microseconds>(tB - tA).count()
                     << " µs\n";

            // --- mid のクラスタ抽出 ---
            tA = std::chrono::steady_clock::now();
            auto mid_cls = CFreeICS(mid).extract();
            // --- ここで距離制限を適用 ---
            // --- ★ mid_cls のうち、バウンディングボックスが used_cluster の範囲 + max_move_dist に入るもののみ残す ---
            auto used_box = compute_bounding_box(used_cluster);

            mid_cls.erase(
                std::remove_if(mid_cls.begin(), mid_cls.end(),
                            [&](const PointCloud& cand) {
                                auto cand_box = compute_bounding_box(cand);
                                return !overlap_with_margin(used_box, cand_box, max_move_dist);
                            }),
                mid_cls.end());
            // auto centroid_used_n1 = compute_centroid(used_cluster);

            // // mid_cls のうち、重心が max_move_dist 以内のものだけを残す
            // mid_cls.erase(
            //     std::remove_if(mid_cls.begin(), mid_cls.end(),
            //                 [&](const PointCloud& cand) {
            //                     auto cc = compute_centroid(cand);
            //                     double d = (cc - centroid_used_n1).norm();
            //                     return d > max_move_dist; // 外側を除外
            //                 }),
            //     mid_cls.end());
            tB = std::chrono::steady_clock::now();
            log_file << "[TIME] extract(mid): "
                     << std::chrono::duration_cast<std::chrono::milliseconds>(tB - tA).count()
                     << " ms\n";

            if (mid_cls.empty()) {
                valid = false;
                break;
            }

            // --- クラスタを重心距離でソート ---
            auto centroid_used = compute_centroid(used_cluster);
            std::sort(mid_cls.begin(), mid_cls.end(),
                      [&](const PointCloud& a, const PointCloud& b) {
                          auto ca = compute_centroid(a);
                          auto cb = compute_centroid(b);
                          double da = (ca - centroid_used).norm();
                          double db = (cb - centroid_used).norm();
                          return da < db;
                      });

            bool found_same = false;

            // --- 近いクラスタから順にチェック ---
            for (auto& cand : mid_cls) {
                tA = std::chrono::steady_clock::now();
                std::vector<double> eig_cand = compute_laplacian_eigenvalues(cand, 5);
                tB = std::chrono::steady_clock::now();
                log_file << "[TIME] eig(mid): "
                         << std::chrono::duration_cast<std::chrono::milliseconds>(tB - tA).count()
                         << " ms\n";

                // 固有値距離計算
                double dist = 0.0;
                size_t loop_len = std::min(eig_used.size(), eig_cand.size());
                for (size_t ei = 0; ei < loop_len; ++ei) {
                    double d = eig_used[ei] - eig_cand[ei];
                    dist += d * d;
                }
                dist = std::sqrt(dist);

                if (dist <= 0.4) {
                    // 同一クラスタ判定 → 更新して次の補間点へ
                    used_cluster = cand;
                    eig_used = eig_cand;
                    found_same = true;
                    break;
                }
            }

            if (!found_same) {
                log_file << "[FAIL] no matching cluster at t=" << t << "\n";
                valid = false;
                break;
            }
        }
        auto t2 = std::chrono::steady_clock::now();
        log_file << "[TIME] interpolation loop: "
                 << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t0).count()
                 << " ms\n";

        if (valid) {
            result.elm.erase(result.elm.begin() + i + 1, result.elm.begin() + j);
            std::cout << "[SUCCESS] shortcut success: node " << i << " to " << j << '\n';
            ++successful;
        } else {
            std::cout << "[FAIL] shortcut failed: node " << i << " to " << j << '\n';
        }
    }

    log_file << "[INFO] shortcut result size = " << result.size()
             << ", successful trials = " << successful << " / " << trials << '\n';
    log_file.close();

    return result;
}
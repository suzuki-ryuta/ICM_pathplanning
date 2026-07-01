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



// NodeList PathShortcut::shortcut()// 高速化 (角度ジャンプ制限付き)事前にペア作成
// {
//     std::mt19937 rng(std::random_device{}());
//     NodeList result = path;

//     if (result.size() < 3) {
//         std::cout << "[INFO] path too short to shortcut.\n";
//         return result;
//     }

//     const double ANGLE_THRESHOLD = 5.0 ;  // 5度
//     const int steps = 20;

//     // --- 有効な(i,j)ペアを列挙 ---
//     std::vector<std::pair<int, int>> valid_pairs;
//     for (int i = 0; i < result.size(); ++i) {
//         for (int j = i + 2; j < result.size(); ++j) {
//             Node n1 = result[i];
//             Node n2 = result[j];

//             bool within_limit = true;
//             for (int d = 0; d < Node::dof; ++d) {
//                 if (std::abs(n1[d] - n2[d]) > ANGLE_THRESHOLD) {
//                     within_limit = false;
//                     break;
//                 }
//             }

//             if (within_limit) valid_pairs.emplace_back(i, j);
//         }
//     }

//     if (valid_pairs.empty()) {
//         std::cout << "[WARN] no valid (i,j) pairs within angle threshold.\n";
//         return result;
//     }

//     std::shuffle(valid_pairs.begin(), valid_pairs.end(), rng);
//     int attempt_limit = std::min(trials, static_cast<int>(valid_pairs.size()));

//     for (int attempt = 0; attempt < attempt_limit; ++attempt) {
//         auto [i, j] = valid_pairs[attempt];
//         Node n1 = result[i];
//         Node n2 = result[j];

//         bool valid = true;
//         PointCloud used_cluster;

//         {
//             CFreeICS ics(n1);
//             auto cls = ics.extract();
//             if (cls.empty()) {
//                 std::cout << "[FAIL] no clusters at node " << i << '\n';
//                 continue;
//             }

//             used_cluster = *std::max_element(
//                 cls.begin(), cls.end(),
//                 [](const PointCloud& a, const PointCloud& b) {
//                     return a.size() < b.size();
//                 });

//             std::cout << "[INFO] using cluster of size " << used_cluster.size()
//                       << " from node " << i << " to " << j << '\n';
//         }

//         std::vector<double> eig_used = compute_laplacian_eigenvalues(used_cluster, 5);

//         for (int k = 1; k < steps; ++k) {
//             double t = static_cast<double>(k) / steps;
//             Node mid = n1.interpolate(n2, t);

//             if (!robot_update(mid)) {
//                 valid = false;
//                 break;
//             }

//             auto mid_cls = CFreeICS(mid).extract();
//             if (mid_cls.empty()) {
//                 valid = false;
//                 break;
//             }

//             auto max_cluster = *std::max_element(
//                 mid_cls.begin(), mid_cls.end(),
//                 [](const PointCloud& a, const PointCloud& b) {
//                     return a.size() < b.size();
//                 });

//             std::vector<double> eig_mid = compute_laplacian_eigenvalues(max_cluster, 5);

//             double dist = 0.0;
//             size_t loop_len = std::min(eig_used.size(), eig_mid.size());
//             for (size_t ei = 0; ei < loop_len; ++ei) {
//                 double d = eig_used[ei] - eig_mid[ei];
//                 dist += d * d;
//             }
//             dist = std::sqrt(dist);

//             if (dist > 0.4) {
//                 std::cout << "[FAIL] spectral divergence at t=" << t << ", dist=" << dist << '\n';
//                 valid = false;
//                 break;
//             }
//         }

//         if (valid) {
//             result.elm.erase(result.elm.begin() + i + 1, result.elm.begin() + j);
//             std::cout << "[SUCCESS] shortcut success: node " << i << " to " << j << '\n';
//         } else {
//             std::cout << "[FAIL] shortcut failed: node " << i << " to " << j << '\n';
//         }
//     }

//     std::cout << "[INFO] shortcut result size = " << result.size() << '\n';
//     return result;
// }

// NodeList PathShortcut::shortcut() // 高速化 (角度ジャンプ制限付き)
// {
//     std::mt19937 rng(std::random_device{}());
//     NodeList result = path;

//     if (result.size() < 3) {
//         std::cout << "[INFO] path too short to shortcut.\n";
//         return result;
//     }

//     const double ANGLE_THRESHOLD = 5.0 ;  // 5度
//     const int steps = 20;

//     for (int trial = 0; trial < trials; ++trial) {
//         std::uniform_int_distribution<> dist(0, result.size() - 1);
//         int i = dist(rng), j = dist(rng);
//         if (i == j) continue;
//         if (i > j) std::swap(i, j);
//         if ((j - i) < 2|| (j - i) > 10) continue;

//         Node n1 = result[i];
//         Node n2 = result[j];

//         // 🆕 各関節の角度差が5度以内かチェック
//         bool within_angle_limit = true;
//         for (int d = 0; d < Node::dof; ++d) {
//             double diff = std::abs(n1[d] - n2[d]);
//             if (diff > ANGLE_THRESHOLD) {
//                 within_angle_limit = false;
//                 break;
//             }
//         }
//         if (!within_angle_limit) {
//             // std::cout << "[SKIP] angle jump too large from node " << i << " to " << j << '\n';
//             continue;
//         }

//         bool valid = true;
//         PointCloud used_cluster;

//         {
//             CFreeICS ics(n1);
//             auto cls = ics.extract();
//             if (cls.empty()) {
//                 std::cout << "[FAIL] no clusters at node " << i << '\n';
//                 continue;
//             }

//             used_cluster = *std::max_element(
//                 cls.begin(), cls.end(),
//                 [](const PointCloud& a, const PointCloud& b) {
//                     return a.size() < b.size();
//                 });

//             std::cout << "[INFO] using cluster of size " << used_cluster.size()
//                       << " from node " << i << " to " << j << '\n';
//         }

//         std::vector<double> eig_used = compute_laplacian_eigenvalues(used_cluster, 5);

//         // クラスタ一貫性チェック（補間ステップ）
//         for (int k = 1; k < steps; ++k) {
//             double t = static_cast<double>(k) / steps;
//             Node mid = n1.interpolate(n2, t);

//             if (!robot_update(mid)) {
//                 valid = false;
//                 break;
//             }

//             auto mid_cls = CFreeICS(mid).extract();
//             if (mid_cls.empty()) {
//                 valid = false;
//                 break;
//             }

//             auto max_cluster = *std::max_element(
//                 mid_cls.begin(), mid_cls.end(),
//                 [](const PointCloud& a, const PointCloud& b) {
//                     return a.size() < b.size();
//                 });

//             std::vector<double> eig_mid = compute_laplacian_eigenvalues(max_cluster, 5);

//             double dist = 0;
//             size_t loop_len = std::min(eig_used.size(), eig_mid.size());
//             for (size_t ei = 0; ei < loop_len; ++ei) {
//                 double d = eig_used[ei] - eig_mid[ei];
//                 dist += d * d;
//             }
//             dist = std::sqrt(dist);

//             if (dist > 0.4) {
//                 std::cout << "[FAIL] spectral divergence at t=" << t << ", dist=" << dist << '\n';
//                 valid = false;
//                 break;
//             }
//         }

//         if (valid) {
//             result.elm.erase(result.elm.begin() + i + 1, result.elm.begin() + j);
//             std::cout << "[SUCCESS] shortcut success: node " << i << " to " << j << '\n';
//         } else {
//             std::cout << "[FAIL] shortcut failed: node " << i << " to " << j << '\n';
//         }
//     }

//     std::cout << "[INFO] shortcut result size = " << result.size() << '\n';
//     return result;
// }

// NodeList PathShortcut::shortcut() //中間失敗したらstepを減らしてく（FAIL）
// {
//     std::mt19937 rng(std::random_device{}());
//     NodeList result = path;

//     if (result.size() < 3) {
//         std::cout << "[INFO] path too short to shortcut.\n";
//         return result;
//     }

//     const double EIG_THRESHOLD = 0.5;
//     const double MAX_ANGLE_STEP = 5.0;

//     for (int trial = 0; trial < trials; ++trial) {
//         std::uniform_int_distribution<> dist(0, result.size() - 1);
//         int i = dist(rng), j = dist(rng);
//         if (i == j) continue;
//         if (i > j) std::swap(i, j);
//         if ((j - i) < 2) continue;

//         Node n1 = result[i];
//         Node n2 = result[j];

//         CFreeICS ics(n1);
//         auto cls = ics.extract();
//         if (cls.empty()) {
//             std::cout << "[FAIL] no clusters at node " << i << '\n';
//             continue;
//         }

//         PointCloud used_cluster = *std::max_element(
//             cls.begin(), cls.end(),
//             [](const PointCloud& a, const PointCloud& b) {
//                 return a.size() < b.size();
//             });

//         std::cout << "[INFO] using cluster of size " << used_cluster.size()
//                   << " from node " << i << " to " << j << '\n';

//         std::vector<double> eig_used = compute_laplacian_eigenvalues(used_cluster, 5);

//         int steps = 1;
//         for (int d = 0; d < Node::dof; ++d) {
//             double angle_diff = std::abs(n2[d] - n1[d]);
//             int step_d = static_cast<int>(std::ceil(angle_diff / MAX_ANGLE_STEP));
//             steps = std::max(steps, step_d);
//         }

//         bool valid = false;

//         // 🆕 [A案]: ステップ数を段階的に下げて検証
//         int attempt_steps = steps;
//         while (attempt_steps >= 2 && !valid) {
//             valid = true;

//             for (int k = 1; k < attempt_steps; ++k) {
//                 double t = static_cast<double>(k) / attempt_steps;
//                 Node mid = n1.interpolate(n2, t);

//                 if (!robot_update(mid)) {
//                     valid = false;
//                     break;
//                 }

//                 auto mid_cls = CFreeICS(mid).extract();
//                 if (mid_cls.empty()) {
//                     valid = false;
//                     break;
//                 }

//                 int max_common = 0;
//                 for (auto& c : mid_cls) {
//                     int common = 0;
//                     for (int i = 0; i < c.size(); ++i) {
//                         if (used_cluster.exist(c.get(i))) ++common;
//                     }
//                     if (common > max_common) max_common = common;
//                 }

//                 if (max_common < static_cast<int>(used_cluster.size() * 0.8)) {
//                     valid = false;
//                     break;
//                 }

//                 int match_count = 0;
//                 for (auto& c : mid_cls) {
//                     int common = 0;
//                     for (auto& pt : c) {
//                         if (std::find(used_cluster.begin(), used_cluster.end(), pt) != used_cluster.end()) {
//                             ++common;
//                         }
//                     }
//                     if (common >= static_cast<int>(used_cluster.size() * 0.9)) {
//                         ++match_count;
//                     }
//                 }

//                 if (match_count != 1) {
//                     std::cout << "[FAIL] cluster split detected at t=" << t
//                               << " (matches=" << match_count << ") with steps=" << attempt_steps << '\n';
//                     valid = false;
//                     break;
//                 }
//             }

//             if (!valid) {
//                 attempt_steps /= 2;  // 👈 ここでステップ数を半分に減らして再挑戦
//             }
//         }

//         if (valid) {
//             result.elm.erase(result.elm.begin() + i + 1, result.elm.begin() + j);
//             std::cout << "[PASS] shortcut success: node " << i << " to " << j 
//                       << " with steps=" << attempt_steps << '\n';

//             int inserted = 0;
//             for (int k = 1; k < attempt_steps; ++k) {
//                 double t = static_cast<double>(k) / attempt_steps;
//                 Node mid = n1.interpolate(n2, t);

//                 if (!robot_update(mid)) continue;

//                 auto mid_cls = CFreeICS(mid).extract();
//                 if (mid_cls.empty()) continue;

//                 PointCloud mid_cluster = *std::max_element(
//                     mid_cls.begin(), mid_cls.end(),
//                     [](const PointCloud& a, const PointCloud& b) {
//                         return a.size() < b.size();
//                     });

//                 std::vector<double> eig_c = compute_laplacian_eigenvalues(mid_cluster, 5);

//                 double dist = 0.0;
//                 size_t min_size = std::min(eig_used.size(), eig_c.size());
//                 for (size_t ei = 0; ei < min_size; ++ei)
//                     dist += std::pow(eig_used[ei] - eig_c[ei], 2);
//                 dist = std::sqrt(dist);

//                 if (dist < EIG_THRESHOLD) {
//                     result.elm.insert(result.elm.begin() + i + 1 + inserted, mid);
//                     ++inserted;
//                 }
//             }
//         } else {
//             std::cout << "[FAIL] shortcut failed: node " << i << " to " << j << '\n';
//         }
//     }

//     std::cout << "[INFO] shortcut result size = " << result.size() << '\n';
//     return result;
// }

// NodeList PathShortcut::shortcut() //中間ノードもチェック（うまく削れない）
// {
//     std::mt19937 rng(std::random_device{}());
//     NodeList result = path;

//     if (result.size() < 3) {
//         std::cout << "[INFO] path too short to shortcut.\n";
//         return result;
//     }

//     const double EIG_THRESHOLD = 0.5;
//     const double MAX_ANGLE_STEP = 5.0;  // deg

//     for (int trial = 0; trial < trials; ++trial) {
//         std::uniform_int_distribution<> dist(0, result.size() - 1);
//         int i = dist(rng), j = dist(rng);
//         if (i == j) continue;
//         if (i > j) std::swap(i, j);
//         if ((j - i) < 2) continue;

//         Node n1 = result[i];
//         Node n2 = result[j];

//         bool valid = true;
//         PointCloud used_cluster;

//         {
//             CFreeICS ics(n1);
//             auto cls = ics.extract();
//             if (cls.empty()) {
//                 std::cout << "[FAIL] no clusters at node " << i << '\n';
//                 continue;
//             }

//             used_cluster = *std::max_element(
//                 cls.begin(), cls.end(),
//                 [](const PointCloud& a, const PointCloud& b) {
//                     return a.size() < b.size();
//                 });

//             std::cout << "[INFO] using cluster of size " << used_cluster.size()
//                       << " from node " << i << " to " << j << '\n';
//         }

//         std::vector<double> eig_used = compute_laplacian_eigenvalues(used_cluster, 5);

//         // ステップ数を動的に決定（関節角度差に基づく）
//         int steps = 1;
//         for (int d = 0; d < Node::dof; ++d) {
//             double angle_diff = std::abs(n2[d] - n1[d]);
//             int step_d = static_cast<int>(std::ceil(angle_diff / MAX_ANGLE_STEP));
//             steps = std::max(steps, step_d);
//         }

//         // 補間チェック
//         for (int k = 1; k < steps; ++k) {
//             double t = static_cast<double>(k) / steps;
//             Node mid = n1.interpolate(n2, t);

//             if (!robot_update(mid)) {
//                 valid = false;
//                 break;
//             }

//             auto mid_cls = CFreeICS(mid).extract();
//             if (mid_cls.empty()) {
//                 valid = false;
//                 break;
//             }

//             int max_common = 0;
//             for (auto& c : mid_cls) {
//                 int common = 0;
//                 for (int i = 0; i < c.size(); ++i) {
//                     if (used_cluster.exist(c.get(i))) ++common;
//                 }
//                 if (common > max_common) max_common = common;
//             }

//             if (max_common < static_cast<int>(used_cluster.size() * 0.8)) {
//                 valid = false;
//                 break;
//             }

//             int match_count = 0;
//             for (auto& c : mid_cls) {
//                 int common = 0;
//                 for (auto& pt : c) {
//                     if (std::find(used_cluster.begin(), used_cluster.end(), pt) != used_cluster.end()) {
//                         ++common;
//                     }
//                 }
//                 if (common >= static_cast<int>(used_cluster.size() * 0.9)) {
//                     ++match_count;
//                 }
//             }

//             if (match_count != 1) {
//                 std::cout << "[FAIL] cluster split detected at t=" << t
//                           << " (matches=" << match_count << ")\n";
//                 valid = false;
//                 break;
//             }
//         }

//         if (valid) {
//             result.elm.erase(result.elm.begin() + i + 1, result.elm.begin() + j);
//             std::cout << "[PASS] shortcut success: node " << i << " to " << j << '\n';

//             // 中間ノードを挿入（クラスタの固有値チェックを通過するものだけ）
//             int inserted = 0;
//             for (int k = 1; k < steps; ++k) {
//                 double t = static_cast<double>(k) / steps;
//                 Node mid = n1.interpolate(n2, t);

//                 if (!robot_update(mid)) continue;

//                 auto mid_cls = CFreeICS(mid).extract();
//                 if (mid_cls.empty()) continue;

//                 PointCloud mid_cluster = *std::max_element(
//                     mid_cls.begin(), mid_cls.end(),
//                     [](const PointCloud& a, const PointCloud& b) {
//                         return a.size() < b.size();
//                     });

//                 std::vector<double> eig_c = compute_laplacian_eigenvalues(mid_cluster, 5);

//                 double dist = 0.0;
//                 size_t min_size = std::min(eig_used.size(), eig_c.size());
//                 for (size_t ei = 0; ei < min_size; ++ei)
//                     dist += std::pow(eig_used[ei] - eig_c[ei], 2);
//                 dist = std::sqrt(dist);

//                 if (dist < EIG_THRESHOLD) {
//                     result.elm.insert(result.elm.begin() + i + 1 + inserted, mid);
//                     ++inserted;
//                 }
//             }
//         } else {
//             std::cout << "[FAIL] shortcut failed: node " << i << " to " << j << '\n';
//         }
//     }

//     std::cout << "[INFO] shortcut result size = " << result.size() << '\n';
//     return result;
// }

// NodeList PathShortcut::shortcut() {
//     std::mt19937 rng(std::random_device{}());
//     NodeList result = path;

//     if (result.size() < 3) {
//         std::cout << "[INFO] path too short to shortcut.\n";
//         return result;
//     }

//     for (int trial = 0; trial < trials; ++trial) {
//         std::uniform_int_distribution<> dist(0, result.size() - 1);
//         int i = dist(rng), j = dist(rng);
//         if (i == j || (j - i) < 2) continue;
//         if (i > j) std::swap(i, j);

//         Node n1 = result[i];
//         Node n2 = result[j];

//         // === 補間ステップ数の動的決定 ====================
//         const int dof = n1.node.size();
//         int dynamic_steps = 1;
//         for (int d = 0; d < dof; ++d) {
//             double diff = std::abs(n2.node[d] - n1.node[d]);
//             int steps = static_cast<int>(std::ceil(diff / 5.0));
//             if (steps > dynamic_steps) dynamic_steps = steps;
//         }
//         int steps = std::clamp(dynamic_steps, 20, 100);
//         // ===============================================

//         bool valid = true;
//         PointCloud used_cluster;

//         {
//             CFreeICS ics(n1);
//             auto cls = ics.extract();
//             if (cls.empty()) continue;

//             used_cluster = *std::max_element(cls.begin(), cls.end(),
//                 [](const PointCloud& a, const PointCloud& b) {
//                     return a.size() < b.size();
//                 });
//         }

//         for (int k = 1; k < steps; ++k) {
//             double t = static_cast<double>(k) / steps;
//             Node mid = n1.interpolate(n2, t);

//             if (!robot_update(mid)) {
//                 valid = false;
//                 break;
//             }

//             auto mid_cls = CFreeICS(mid).extract();
//             if (mid_cls.empty()) {
//                 valid = false;
//                 break;
//             }

//             // Laplacian固有値との距離でクラスタ整合性評価
//             std::vector<double> eig_used = compute_laplacian_eigenvalues(used_cluster, 5);
//             double min_diff = 1e9;

//             for (const auto& c : mid_cls) {
//                 std::vector<double> eig_c = compute_laplacian_eigenvalues(c, 5);
//                 size_t min_size = std::min(eig_used.size(), eig_c.size());
//                 double diff = 0.0;

//                 for (size_t ei = 0; ei < min_size; ++ei) {
//                     double delta = eig_used[ei] - eig_c[ei];
//                     diff += delta * delta;
//                 }

//                 if (diff < min_diff) min_diff = diff;
//             }

//             if (min_diff > 5.0) { // ← このしきい値は調整可能
//                 valid = false;
//                 break;
//             }
//         }

//         if (valid) {
//             // 中間ノードを必要に応じて追加
//             std::vector<Node> mids;
//             for (int k = 1; k < steps; ++k) {
//                 double t = static_cast<double>(k) / steps;
//                 mids.push_back(n1.interpolate(n2, t));
//             }

//             result.elm.erase(result.elm.begin() + i + 1, result.elm.begin() + j);
//             result.elm.insert(result.elm.begin() + i + 1, mids.begin(), mids.end());

//             std::cout << "[PASS] shortcut success: node " << i << " to " << j
//                       << " with " << mids.size() << " mid nodes.\n";
//         } else {
//             std::cout << "[FAIL] shortcut failed: node " << i << " to " << j << '\n';
//         }
//     }

//     std::cout << "[INFO] shortcut result size = " << result.size() << '\n';
//     return result;
// }
// NodeList PathShortcut::shortcut() 
// {
//     std::mt19937 rng(std::random_device{}());
//     NodeList result = path;

//     if (result.size() < 3) {
//         std::cout << "[INFO] path too short to shortcut.\n";
//         return result;
//     }

//     for (int trial = 0; trial < trials; ++trial) {
//         std::uniform_int_distribution<> dist(0, result.size() - 1);
//         int i = dist(rng), j = dist(rng);
//         if (i == j) continue;
//         if (i > j) std::swap(i, j);
//         if ((j - i) < 2) continue;

//         Node n1 = result[i];
//         Node n2 = result[j];

//         const int steps = 20;

//         bool valid = true;
//         PointCloud used_cluster;

//         {
//             CFreeICS ics(n1);
//             auto cls = ics.extract();
//             if (cls.empty()) {
//                 std::cout << "[FAIL] no clusters at node " << i << '\n';
//                 continue;
//             }

//             used_cluster = *std::max_element(
//                 cls.begin(), cls.end(),
//                 [](const PointCloud& a, const PointCloud& b) {
//                     return a.size() < b.size();
//                 });

//             std::cout << "[INFO] using cluster of size " << used_cluster.size()
//                       << " from node " << i << " to " << j << '\n';
//         }

//         for (int k = 1; k < steps; ++k) {
//             double t = static_cast<double>(k) / steps;
//             Node mid = n1.interpolate(n2, t);

//             if (!robot_update(mid)) {
//                 valid = false;
//                 break;
//             }

//             auto mid_cls = CFreeICS(mid).extract();
//             if (mid_cls.empty()) {
//                 valid = false;
//                 break;
//             }

//             std::vector<double> eig_used = compute_laplacian_eigenvalues(used_cluster, 5);
//             std::vector<double> eig_mid;

//             PointCloud best_cluster;
//             double min_diff = 1e9;

//             for (auto& c : mid_cls) {
//                 auto eig_c = compute_laplacian_eigenvalues(c, 5);
//                 double diff = 0.0;
//                 size_t min_size = std::min(eig_used.size(), eig_c.size());
//                 for (size_t ei = 0; ei < min_size; ++ei) {
//                     diff += std::abs(eig_used[ei] - eig_c[ei]);
//                 }
//                 if (diff < min_diff) {
//                     min_diff = diff;
//                     best_cluster = c;
//                     eig_mid = eig_c;
//                 }
//             }

//             if (min_diff > 1.0) {
//                 std::cout << "[FAIL] cluster structure mismatch at t=" << t << ", eigen diff = " << min_diff << '\n';
//                 valid = false;
//                 break;
//             }
//         }

//         if (valid) {
//             // ===== 中間ノードの挿入処理 =====
//             auto compute_mid_steps = [](const Node& n1, const Node& n2, int max_insert = 10) -> int {
//                 double angle_diff = 0.0;
//                 for (int i = 0; i < Node::dof; ++i) {
//                     double diff = std::abs(n1.get_element(i) - n2.get_element(i));
//                     angle_diff += diff;
//                 }
//                 int steps = static_cast<int>(angle_diff / 30.0);
//                 steps = std::clamp(steps, 1, max_insert);

//                 // ✅ ボーナス: デバッグログ表示
//                 std::cout << "[MIDGEN] angle_diff = " << angle_diff 
//                           << ", inserted = " << steps << " nodes.\n";

//                 return steps;
//             };

//             int insert_steps = compute_mid_steps(n1, n2);
//             std::vector<Node> mids;
//             for (int m = 1; m <= insert_steps; ++m) {
//                 double t = static_cast<double>(m) / (insert_steps + 1);
//                 mids.push_back(n1.interpolate(n2, t));
//             }

//             result.elm.erase(result.elm.begin() + i + 1, result.elm.begin() + j);
//             result.elm.insert(result.elm.begin() + i + 1, mids.begin(), mids.end());

//             std::cout << "[PASS] shortcut success: node " << i << " to " << j
//                       << " with " << insert_steps << " mid-nodes inserted.\n";
//         } else {
//             std::cout << "[FAIL] shortcut failed: node " << i << " to " << j << '\n';
//         }
//     }

//     std::cout << "[INFO] shortcut result size = " << result.size() << '\n';
//     return result;
// }

// NodeList PathShortcut::shortcut() // 高速化
// {
//     std::mt19937 rng(std::random_device{}());
//     NodeList result = path;

//     if (result.size() < 3) {
//         std::cout << "[INFO] path too short to shortcut.\n";
//         return result;
//     }

//     for (int trial = 0; trial < trials; ++trial) {
//         std::uniform_int_distribution<> dist(0, result.size() - 1);
//         int i = dist(rng), j = dist(rng);
//         if (i == j) continue;
//         if (i > j) std::swap(i, j);
//         if ((j - i) < 2) continue;
//         // if ((j - i) < 2|| (j - i) > 20) continue;
//         // int i = rng() % (result.size() - 2);
//         // int j = i + 2 + rng() % std::min(10, (int)result.size() - i - 2);


//         Node n1 = result[i];
//         Node n2 = result[j];

//         const int steps = 20;
//         bool valid = true;

//         // 距離に応じた補間数（最小10、最大100）
//         // double angle_dist = n1.norm(n2);
//         // int steps = std::clamp(static_cast<int>(std::ceil(angle_dist / 2.0)), 5, 30);

//         PointCloud used_cluster;

//         // ★ n1でクラスタを取得し、その中の最大サイズのクラスタを1つ選んで「使うクラスタ」に設定
//         {
//             CFreeICS ics(n1);
//             auto cls = ics.extract();
//             if (cls.empty()) {
//                 std::cout << "[FAIL] no clusters at node " << i << '\n';
//                 continue;
//             }

//             used_cluster = *std::max_element(
//                 cls.begin(), cls.end(),
//                 [](const PointCloud& a, const PointCloud& b) {
//                     return a.size() < b.size();
//                 });

//             std::cout << "[INFO] using cluster of size " << used_cluster.size()
//                       << " from node " << i << " to " << j << '\n';
//         }
//         // ラプラシアン固有値（上位5個）を使用して一致性を評価
//         std::vector<double> eig_used = compute_laplacian_eigenvalues(used_cluster, 5); // 上位5次元だけ比較

//         // ★ n1→n2 の補間検証
//         for (int k = 1; k < steps; ++k) {
//             double t = static_cast<double>(k) / steps;
//             Node mid = n1.interpolate(n2, t);

//             if (!robot_update(mid)) {
//                 valid = false;
//                 break;
//             }

//             // ★ mid におけるクラスタを抽出し、used_cluster が分裂してないか確認
//             auto mid_cls = CFreeICS(mid).extract();
//             if (mid_cls.empty()) {
//                 valid = false;
//                 break;
//             }

//             auto max_cluster = *std::max_element(
//                 mid_cls.begin(), mid_cls.end(),
//                 [](const PointCloud& a, const PointCloud& b) {
//                     return a.size() < b.size();
//                 });

//             std::vector<double> eig_mid = compute_laplacian_eigenvalues(max_cluster, 5);

//             // L2ノルム距離で判定
//             double dist = 0;
//             size_t loop_len = std::min(eig_used.size(), eig_mid.size());
//             for (size_t ei = 0; ei < loop_len; ++ei) {
//                 double d = eig_used[ei] - eig_mid[ei];
//                 dist += d * d;
//             }
//             dist = std::sqrt(dist);

//             if (dist > 0.4) {  // 閾値：調整可能（0.3〜0.6）
//                 std::cout << "[FAIL] spectral divergence at t=" << t << ", dist=" << dist << '\n';
//                 valid = false;
//                 break;
//             }
//         }

//         if (valid) {
//             result.elm.erase(result.elm.begin() + i + 1, result.elm.begin() + j);
//             std::cout << "[SUCCESS] shortcut success: node " << i << " to " << j << '\n';
//         } else {
//             std::cout << "[FAIL] shortcut failed: node " << i << " to " << j << '\n';
//         }
//     }

//     std::cout << "[INFO] shortcut result size = " << result.size() << '\n';
//     return result;
// }

//             for (auto& c : mid_cls) {
//                 auto spec = compute_laplacian_spectrum(c);
//                 double dist = eigen_distance(base_spec, spec);
//                 if (dist < 1.0) { // 固有値距離閾値（要調整）
//                     matched = true;
//                     break;
//                 }
//             }

//             if (!matched) {
//                 std::cout << "[FAIL] spectrum mismatch at t=" << t << '\n';
//                 valid = false;
//                 break;
//             }
//         }

//         if (valid) {
//             result.elm.erase(result.elm.begin() + i + 1, result.elm.begin() + j);
//             std::cout << "[PASS] shortcut success: node " << i << " to " << j << '\n';
//         } else {
//             std::cout << "[FAIL] shortcut failed: node " << i << " to " << j << '\n';
//         }
//     }

//     std::cout << "[INFO] shortcut result size = " << result.size() << '\n';
//     return result;
// }


// NodeList PathShortcut::shortcut() 
// {
//     std::mt19937 rng(std::random_device{}());
//     NodeList result = path;

//     if (result.size() < 3) {
//         std::cout << "[INFO] path too short to shortcut.\n";
//         return result;
//     }

//     for (int trial = 0; trial < trials; ++trial) {
//         std::uniform_int_distribution<> dist(0, result.size() - 1);
//         int i = dist(rng), j = dist(rng);
//         if (i == j) continue;
//         if (i > j) std::swap(i, j);
//         if ((j - i) < 2) continue;
//         // if ((j - i) < 2|| (j - i) > 20) continue;

//         Node n1 = result[i];
//         Node n2 = result[j];

//         const int steps = 20;

//         // 距離に応じた補間数（最小10、最大100）
//         // double angle_dist = n1.norm(n2);
//         // int steps = std::clamp(static_cast<int>(std::ceil(angle_dist / 2.0)), 5, 30);

//         bool valid = true;
//         PointCloud used_cluster;

//         // ★ n1でクラスタを取得し、その中の最大サイズのクラスタを1つ選んで「使うクラスタ」に設定
//         {
//             CFreeICS ics(n1);
//             auto cls = ics.extract();
//             if (cls.empty()) {
//                 std::cout << "[FAIL] no clusters at node " << i << '\n';
//                 continue;
//             }

//             used_cluster = *std::max_element(
//                 cls.begin(), cls.end(),
//                 [](const PointCloud& a, const PointCloud& b) {
//                     return a.size() < b.size();
//                 });

//             std::cout << "[INFO] using cluster of size " << used_cluster.size()
//                       << " from node " << i << " to " << j << '\n';
//         }
//         auto base_spec = compute_laplacian_spectrum(used_cluster);


//         // ★ n1→n2 の補間検証
//         for (int k = 1; k < steps; ++k) {
//             double t = static_cast<double>(k) / steps;
//             Node mid = n1.interpolate(n2, t);

//             if (!robot_update(mid)) {
//                 valid = false;
//                 break;
//             }

//             // ★ mid におけるクラスタを抽出し、used_cluster が分裂してないか確認
//             auto mid_cls = CFreeICS(mid).extract();
//             bool matched = false;

//             for (auto& c : mid_cls) {
//                 auto spec = compute_laplacian_spectrum(c);
//                 double dist = eigen_distance(base_spec, spec);
//                 if (dist < 1.0) { // 固有値距離閾値（要調整）
//                     matched = true;
//                     break;
//                 }
//             }

//             if (!matched) {
//                 std::cout << "[FAIL] spectrum mismatch at t=" << t << '\n';
//                 valid = false;
//                 break;
//             }
//         }

//         if (valid) {
//             result.elm.erase(result.elm.begin() + i + 1, result.elm.begin() + j);
//             std::cout << "[PASS] shortcut success: node " << i << " to " << j << '\n';
//         } else {
//             std::cout << "[FAIL] shortcut failed: node " << i << " to " << j << '\n';
//         }
//     }

//     std::cout << "[INFO] shortcut result size = " << result.size() << '\n';
//     return result;
// }


// NodeList PathShortcut::shortcut() 
// {
//     std::mt19937 rng(std::random_device{}());
//     NodeList result = path;

//     if (result.size() < 3) {
//         std::cout << "[INFO] path too short to shortcut.\n";
//         return result;
//     }

//     for (int trial = 0; trial < trials; ++trial) {
//         std::uniform_int_distribution<> dist(0, result.size() - 1);
//         int i = dist(rng), j = dist(rng);
//         if (i == j) continue;
//         if (i > j) std::swap(i, j);
//         if ((j - i) < 2) continue;
//         // if ((j - i) < 2|| (j - i) > 20) continue;

//         Node n1 = result[i];
//         Node n2 = result[j];

//         const int steps = 20;

//         // 距離に応じた補間数（最小10、最大100）
//         // double angle_dist = n1.norm(n2);
//         // int steps = std::clamp(static_cast<int>(std::ceil(angle_dist / 2.0)), 10, 100);

//         bool valid = true;
//         PointCloud used_cluster;

//         // ★ n1でクラスタを取得し、その中の最大サイズのクラスタを1つ選んで「使うクラスタ」に設定
//         {
//             CFreeICS ics(n1);
//             auto cls = ics.extract();
//             if (cls.empty()) {
//                 std::cout << "[FAIL] no clusters at node " << i << '\n';
//                 continue;
//             }

//             used_cluster = *std::max_element(
//                 cls.begin(), cls.end(),
//                 [](const PointCloud& a, const PointCloud& b) {
//                     return a.size() < b.size();
//                 });

//             std::cout << "[INFO] using cluster of size " << used_cluster.size()
//                       << " from node " << i << " to " << j << '\n';
//         }

//         // ★ n1→n2 の補間検証
//         for (int k = 1; k < steps; ++k) {
//             double t = static_cast<double>(k) / steps;
//             Node mid = n1.interpolate(n2, t);

//             if (!robot_update(mid)) {
//                 valid = false;
//                 break;
//             }

//             // ★ mid におけるクラスタを抽出し、used_cluster が分裂してないか確認
//             auto mid_cls = CFreeICS(mid).extract();

//             // if (match_count != 1) {
//             //     valid = false;
//             //     break;
//             // }
//             // 方法3：最大一致クラスタ===================================================
//             int max_common = 0;
//             for (auto& c : mid_cls) {
//                 int common = 0;
//                 for (int i = 0; i < c.size(); ++i) {
//                     if (used_cluster.exist(c.get(i))) ++common;
//                 }
//                 if (common > max_common) max_common = common;
//             }

//             if (max_common < static_cast<int>(used_cluster.size() * 0.8)) {
//                 valid = false;
//                 break;
//             }
//             //============================================================================


//             if (mid_cls.empty()) {
//                 valid = false;
//                 break;
//             }

//             // used_cluster と**重なっているクラスタを数える**
//             int match_count = 0;
//             for (auto& c : mid_cls) {
//                 int common = 0;
//                 for (int i = 0; i < c.size(); ++i) {
//                     if (used_cluster.exist(c.get(i))) ++common;
//                 }

//                 if (common >= static_cast<int>(used_cluster.size() * 0.8)) { // 80%以上一致でOKとする
//                     ++match_count;
//                 }
//             }

//             if (match_count != 1) {
//                 std::cout << "[FAIL] cluster split detected at t=" << t
//                           << " (matches=" << match_count << ")\n";
//                 valid = false;
//                 break;
//             }
//         }

//         if (valid) {
//             result.elm.erase(result.elm.begin() + i + 1, result.elm.begin() + j);
//             std::cout << "[PASS] shortcut success: node " << i << " to " << j << '\n';
//         } else {
//             std::cout << "[FAIL] shortcut failed: node " << i << " to " << j << '\n';
//         }
//     }

//     std::cout << "[INFO] shortcut result size = " << result.size() << '\n';
//     return result;
// }

// NodeList PathShortcut::shortcut() 
// {
//     std::mt19937 rng(std::random_device{}());
//     NodeList result = path;

//     if (result.size() < 3) {
//         std::cout << "[INFO] path too short to shortcut.\n";
//         return result;
//     }

//     for (int trial = 0; trial < trials; ++trial) {
//         std::uniform_int_distribution<> dist(0, result.size() - 1);
//         int i = dist(rng), j = dist(rng);
//         if (i == j) continue;
//         if (i > j) std::swap(i, j);
//         if ((j - i) < 2) continue;
//         // if ((j - i) < 2|| (j - i) > 20) continue;

//         Node n1 = result[i];
//         Node n2 = result[j];

//         const int steps = 20;

//         // 距離に応じた補間数（最小10、最大100）
//         // double angle_dist = n1.norm(n2);
//         // int steps = std::clamp(static_cast<int>(std::ceil(angle_dist / 2.0)), 10, 100);

//         bool valid = true;
//         PointCloud used_cluster;

//         // ★ n1でクラスタを取得し、その中の最大サイズのクラスタを1つ選んで「使うクラスタ」に設定
//         {
//             CFreeICS ics(n1);
//             auto cls = ics.extract();
//             if (cls.empty()) {
//                 std::cout << "[FAIL] no clusters at node " << i << '\n';
//                 continue;
//             }

//             used_cluster = *std::max_element(
//                 cls.begin(), cls.end(),
//                 [](const PointCloud& a, const PointCloud& b) {
//                     return a.size() < b.size();
//                 });

//             std::cout << "[INFO] using cluster of size " << used_cluster.size()
//                       << " from node " << i << " to " << j << '\n';
//         }

//         // ★ n1→n2 の補間検証
//         for (int k = 1; k < steps; ++k) {
//             double t = static_cast<double>(k) / steps;
//             Node mid = n1.interpolate(n2, t);

//             if (!robot_update(mid)) {
//                 valid = false;
//                 break;
//             }

//             // ★ mid におけるクラスタを抽出し、used_cluster が分裂してないか確認
//             auto mid_cls = CFreeICS(mid).extract();

//             // 方法1：重心距離ベース=============================================
//             // Vector2D<double> center_used = compute_centroid(used_cluster);
//             // int match_count = 0;
//             // for (auto& c : mid_cls) {
//             //     Vector2D<double> center_c = compute_centroid(c);
//             //     double dist = (center_used - center_c).norm();
//             //     if (dist < 5.0) {
//             //         ++match_count;
//             //     }
//             // }
//             //===========================================================================
//             // 方法2：Jaccard係数ベース==================================================
//             // int match_count = 0;
//             // for (auto& c : mid_cls) {
//             //     double jaccard = compute_jaccard(used_cluster, c);
//             //     if (jaccard > 0.8) {
//             //         ++match_count;
//             //     }
//             // }
//             //==========================================================================

//             // if (match_count != 1) {
//             //     valid = false;
//             //     break;
//             // }
//             // 方法3：最大一致クラスタ===================================================
//             int max_common = 0;
//             for (auto& c : mid_cls) {
//                 int common = 0;
//                 for (int i = 0; i < c.size(); ++i) {
//                     if (used_cluster.exist(c.get(i))) ++common;
//                 }
//                 if (common > max_common) max_common = common;
//             }

//             if (max_common < static_cast<int>(used_cluster.size() * 0.8)) {
//                 valid = false;
//                 break;
//             }
//             //============================================================================


//             if (mid_cls.empty()) {
//                 valid = false;
//                 break;
//             }

//             // used_cluster と**重なっているクラスタを数える**
//             int match_count = 0;
//             for (auto& c : mid_cls) {
//                 int common = 0;
//                 for (auto& pt : c) {
//                     if (std::find(used_cluster.begin(), used_cluster.end(), pt) != used_cluster.end()) {
//                         ++common;
//                     }
//                 }
//                 if (common >= static_cast<int>(used_cluster.size() * 0.9)) { // 80%以上一致でOKとする
//                     ++match_count;
//                 }
//             }

//             if (match_count != 1) {
//                 std::cout << "[FAIL] cluster split detected at t=" << t
//                           << " (matches=" << match_count << ")\n";
//                 valid = false;
//                 break;
//             }
//         }

//         if (valid) {
//             result.elm.erase(result.elm.begin() + i + 1, result.elm.begin() + j);
//             std::cout << "[PASS] shortcut success: node " << i << " to " << j << '\n';
//         } else {
//             std::cout << "[FAIL] shortcut failed: node " << i << " to " << j << '\n';
//         }
//     }

//     std::cout << "[INFO] shortcut result size = " << result.size() << '\n';
//     return result;
// }


// NodeList PathShortcut::shortcut() 
// {
//     std::mt19937 rng(std::random_device{}());
//     NodeList result = path;

//     if (result.size() < 3) {
//         std::cout << "[INFO] path too short to shortcut.\n";
//         return result;
//     }

//     // ★ 初期クラスタを1回だけ取得（path[0] を代表とする）
//     PointCloud prev_cloud;
//     {
//         CFreeICS ics(result[0]);
//         auto init_cfree = ics.extract();
//         if (init_cfree.size() != 1) {
//             std::cout << "[SKIP] initial cluster split (size = " << init_cfree.size() << ")\n";
//             return result;
//         }

//         prev_cloud = *std::max_element(
//             init_cfree.begin(), init_cfree.end(),
//             [](const PointCloud& a, const PointCloud& b){
//                 return a.size() < b.size();
//             });

//         std::cout << "[INFO] selected initial cluster, size = " << prev_cloud.size() << '\n';
//     }

//     for (int trial = 0; trial < trials; ++trial) {
//         // if (result.size() < 3) break;
//         std::uniform_int_distribution<> dist(0, result.size() - 1);
//         int i = dist(rng), j = dist(rng);
//         if (i == j) continue;
//         if (i > j) std::swap(i, j);
//         if (j - i < 2) continue;

//         Node n1 = result[i];
//         Node n2 = result[j];

//         // 線形補間
//         const int steps = 20;
//         /* ★ここを追加：角度差に応じて steps を決定 */
//         // double angle_dist = n1.norm(n2);            // ユークリッド距離[deg]
//         // int steps = static_cast<int>(std::ceil(angle_dist / 2.0)); // 2°ごと
//         // steps = std::clamp(steps, 10, 100);
//         //10から100の間に制限　追加
//         // std::vector<Node> interpolated;
//         bool valid = true;
//         // PointCloud prev_cloud;

//         // 初期クラスタを取得
//         // ──────── ここから置き換え ────────
//         // 初期クラスタを取得（最大サイズを自動選択）
//         // {
//         //     CFreeICS ics(n1);
//         //     auto init_cfree = ics.extract();
//         //     if (init_cfree.size() != 1) {
//         //         std::cout << "[INFO] Skipping shortcut from node " << i << " to " << j 
//         //                   << " due to cluster splitting (size = " << init_cfree.size() << ")\n";
//         //         continue;
//         //     }    

//         //     // サイズが最大のクラスタを prev_cloud に採用
//         //     prev_cloud = *std::max_element(
//         //         init_cfree.begin(), init_cfree.end(),
//         //         [](const PointCloud& a, const PointCloud& b){
//         //             return a.size() < b.size();
//         //         });

//         //     // デバッグ表示（任意）
//         //     std::cout << "[INFO] auto-selected cluster size = "
//         //               << prev_cloud.size() << '\n';
//         // }
// // ──────── ここまで置き換え ────────

// //         {
// //             CFreeICS ics(n1);
// //             auto init_cfree = ics.extract();
// //             if (init_cfree.empty()) continue;

// //             // std::cout << "Select cluster index [0 - " << init_cfree.size() - 1 << "]: ";
// //             // int idx;
// //             // std::cin >> idx;
// //             // assert(0 <= idx && idx < (int)init_cfree.size());
// //             // prev_cloud = init_cfree[idx];

// //             prev_cloud = *std::max_element(
// //                 init_cfree.begin(), init_cfree.end(),
// //                 [](const PointCloud& a, const PointCloud& b){
// //                     return a.size() < b.size();
// //                 });

// // // ログ（任意）
// //             std::cout << "[INFO] auto-selected cluster size = "
// //                       << prev_cloud.size() << '\n';
// //         }

//         for (int k = 1; k < steps; ++k) {
//             double t = static_cast<double>(k) / steps;
//             Node mid = n1.interpolate(n2, t);

//             if (!robot_update(mid)) {
//                 valid = false;
//                 break;
//             }
//             if (!caging_valid(prev_cloud, mid, n2)) {
//                 valid = false;
//                 break;
//             }
//             // interpolated.push_back(mid);
//         }

//         if (valid) {
//             result.elm.erase(result.elm.begin() + i + 1, result.elm.begin() + j);
//             std::cout << "[PASS] shortcut success: node " << i << " to " << j << '\n';
//         } else {
//             std::cout << "[FAIL] shortcut failed: node " << i << " to " << j << '\n';
//         }
//     }
//     std::cout << "[INFO] shortcut result size = "
//               << result.size() << '\n';         // ← 追加⑤
//     return result;
// }




//失敗したら諦めて別の2点を取る====================================================================
// NodeList PathShortcut::shortcut() {
//     std::mt19937 rng(std::random_device{}());
//     NodeList result = path;
//     int MAX_TRIALS = 300;  // 必要に応じて調整

//     for (int trial = 0; trial < MAX_TRIALS; ++trial) {
//         if (result.size() < 3) break;

//         std::vector<Node> interpolated;
//         std::uniform_int_distribution<> dist(0, result.size() - 1);
//         int i = dist(rng), j = dist(rng);
//         if (i == j) continue;
//         if (i > j) std::swap(i, j);
//         if ((j - i) < 2|| (j - i) > 50) continue;

//         Node n1 = result[i];
//         Node n2 = result[j];
        
//         // constexpr double ANGLE_THRESH = 5.0;  // 単位：deg
//         // if (angle_diff(n1, n2) < ANGLE_THRESH) continue;  // 変化が小さいなら無視
        


//         // 最初のクラスタを一度だけ取得（固定）
//         static bool initialized = false;
//         static PointCloud base_cloud;
//         if (!initialized) {
//             CFreeICS ics(n1);
//             auto clusters = ics.extract();
//             if (clusters.empty()) continue;
//             base_cloud = *std::max_element(clusters.begin(), clusters.end(),
//                                            [](const PointCloud& a, const PointCloud& b) {
//                                                return a.size() < b.size();
//                                            });
//             initialized = true;
//         }

//         std::vector<Node> inserted;
//         const int steps = 20;
//         bool valid = true;
//         PointCloud current = base_cloud;

//         for (int k = 1; k < steps; ++k) {
//             double t = static_cast<double>(k) / steps;
//             Node mid = n1.interpolate(n2, t);

//             if (!robot_update(mid)) {
//                 valid = false;
//                 break;
//             }

//             CFreeICS ics(mid);
//             auto cls = ics.extract();

//             // 分裂検出（案C）
//             int match_count = 0;
//             for (const auto& c : cls) {
//                 if (overlap_ratio(current, c) > 0.5) {
//                     current = c;
//                     match_count++;
//                 }
//             }
//             if (match_count != 1) {
//                 valid = false;  // 分裂しているかクラスタを見失った
//                 break;
//             }

//             if (!caging_valid(current, mid, n2)) {
//                 valid = false;
//                 break;
//             }

//             // inserted.push_back(mid);
//             interpolated.push_back(mid);
//         }
//         if (valid) {
//             std::cout << "[DEBUG] shortcut succeeded: " << (j - i - 1) << " nodes removed\n";
//             result.elm.erase(result.elm.begin() + i + 1, result.elm.begin() + j);
//             result.elm.insert(result.elm.begin() + i + 1, interpolated.begin(), interpolated.end());
//         } else {
//             std::cout << "[DEBUG] shortcut failed: i=" << i << ", j=" << j << "\n";
//         }


//         // if (valid) {
//         //     result.elm.erase(result.elm.begin() + i + 1, result.elm.begin() + j);
//         //     result.elm.insert(result.elm.begin() + i + 1, inserted.begin(), inserted.end());
//         // }
//         // 無効だったらスキップして次の trial へ（案B）
//     }

//     std::cout << "[INFO] shortcut result size = " << result.size() << std::endl;
//     return result;
// }
// double overlap_ratio(const PointCloud& a, const PointCloud& b) {
//     int overlap_count = 0;  // ← ★ これを追加！

//     for (const auto& p1 : a.elm) {
//         for (const auto& p2 : b.elm) {
//             if (p1 == p2) ++overlap_count;
//         }
//     }

//     int min_size = std::min(a.size(), b.size());
//     if (min_size == 0) return 0.0;

//     return static_cast<double>(overlap_count) / min_size;
// }
//========ここまでがショートカットの実装===========================================

// double angle_diff(const Node& a, const Node& b) {
//     double sum = 0.0;
//     for (size_t i = 0; i < a.q.size(); ++i) {
//         double diff = b.q[i] - a.q[i];
//         sum += diff * diff;
//     }
//     return std::sqrt(sum);
// }
double overlap_ratio(const PointCloud& a, const PointCloud& b) {
    int overlap_count = 0;  // ← ★ これを追加！
    if (a.empty() || b.empty()) return 0.0;

    for (const auto& p1 : a.elm) {
        for (const auto& p2 : b.elm) {
            if (p1 == p2)++overlap_count;
        }
    }
    return static_cast<double>(overlap_count) / std::min(a.size(), b.size());
}


// NodeList PathShortcut::shortcut() {
//     std::mt19937 rng(std::random_device{}());
//     NodeList result = path;

//     const int steps = 20;
//     const int max_trials = 1000;
//     const int target_success = result.size() / 10;//目標成功数（パスの10%）

//     int success_cnt = 0, fail_cnt = 0;

//     for (int trial = 0; trial < max_trials && success_cnt < target_success; ++trial) {
//         if (result.size() < 3) break;

//         std::uniform_int_distribution<> dist(0, result.size() - 1);
//         int i = dist(rng), j = dist(rng);
//         if (i == j) continue;
//         if (i > j) std::swap(i, j);
//         if (j - i < 2) continue;

//         Node n1 = result[i];
//         Node n2 = result[j];

//         // 最初のクラスタを取得
//         CFreeICS ics(n1);
//         auto clusters = ics.extract();
//         if (clusters.empty()) continue;

//         PointCloud prev_cloud = *std::max_element(
//             clusters.begin(), clusters.end(),
//             [](const PointCloud& a, const PointCloud& b) {
//                 return a.size() < b.size();
//             });

//         std::vector<Node> keep_pts;
//         bool valid = true;

//         for (int k = 1; k < steps; ++k) {
//             double t = static_cast<double>(k) / steps;
//             Node mid = n1.interpolate(n2, t);

//             if (!robot_update(mid)) {
//                 valid = false;
//                 break;
//             }

//             if (k % 3 == 0) {
//                 CFreeICS mid_ics(mid);
//                 auto now_clusters = mid_ics.extract();
//                 if (now_clusters.empty()) {
//                     valid = false;
//                     break;
//                 }

//                 bool found_overlap = false;
//                 for (const auto& cls : now_clusters) {
//                     if (cls.overlap(prev_cloud)) {
//                         prev_cloud = cls;
//                         found_overlap = true;
//                         break;
//                     }
//                 }

//                 if (!found_overlap) {
//                     valid = false;
//                     break; // ❗ クラスタが連続しなければ即スキップ
//                 }
//             }

//             keep_pts.push_back(mid);
//         }

//         if (valid) {
//             result.elm.erase(result.elm.begin() + i + 1, result.elm.begin() + j);
//             result.elm.insert(result.elm.begin() + i + 1, keep_pts.begin(), keep_pts.end());
//             ++success_cnt;
//         } else {
//             ++fail_cnt;
//         }
//     }

//     std::cout << "[INFO] success " << success_cnt << " / target " << target_success
//               << "  final size = " << result.size() << '\n';
//     return result;
// }
//=====================================================================================================





// /* =========================================================
//    改良版ショートカット（粗パス＋緩いクラスタリセット）
//    ========================================================= */
// NodeList PathShortcut::shortcut()
// {
//     NodeList result = path;
//     std::mt19937 rng(std::random_device{}());

//     /* ------------------ パラメータ ------------------ */
//     const int   MAX_TRIALS  = result.size() * 60;
//     const int   MAX_LEN     = 30;          // j-i の上限
//     const int   MIN_LEN     = 2;           // 下限
//     const auto  TIME_LIMIT  = std::chrono::minutes(30);
//     auto        t0          = std::chrono::steady_clock::now();

//     /* ----------- 粗パス＋緩いクラスタリセット ----------- */
//     int success_cnt = 0, fail_cnt = 0;
//     for (int trial = 0; trial < MAX_TRIALS; ++trial) {

//         if (std::chrono::steady_clock::now() - t0 > TIME_LIMIT) break;
//         if (result.size() < 3) break;

//         /* 区間選択：長い区間も出るよう指数分布 */
//         std::exponential_distribution<> ex(0.1);          // λ=0.08 → ave≈12
//         int len = std::clamp(int(std::round(ex(rng))) + MIN_LEN,
//                              MIN_LEN, MAX_LEN);
//         std::uniform_int_distribution<> di(0, result.size() - len - 1);
//         int i = di(rng), j = i + len;

//         Node n1 = result[i], n2 = result[j];

//         /* Adaptive steps (角度差×1.2) */
//         // int steps = std::clamp(int(std::ceil(n1.norm(n2) * 1.2)), 8, 100);
//         int steps = std::clamp(std::min(len, int(std::ceil(n1.norm(n2)*1.2))), 8, 100);// ★ len との min
//         /* 初期クラスタ（最大サイズ） */
//         auto init_cls = CFreeICS(n1).extract();
//         if (init_cls.empty()) { ++fail_cnt; continue; }
//         PointCloud prev_cloud = *std::max_element(
//             init_cls.begin(), init_cls.end(),
//             [](auto& a, auto& b){ return a.size() < b.size(); });

//         bool valid = true;
//         std::vector<Node> mids; mids.reserve(steps);
//         std::vector<Node> mids_check;  // ← 判定専用
//         std::vector<Node> check_pts; 
//         check_pts.reserve(steps-1);


//         for (int k = 1; k < steps; ++k)
//         {
//             double t   = static_cast<double>(k) / steps;
//             Node   mid = n1.interpolate(n2, t);

//     /* 衝突チェック */
//             if (!robot_update(mid)) { valid = false; break; }

//     /* 3 点に 1 回だけ厳密クラスタ再抽出 */
//             if (k % 3 == 0)
//             {
//                 auto cls = CFreeICS(mid).extract();
//                 if (cls.empty())              { valid = false; break; }
//                 // mids_check.push_back(mid);  // 判定用に保持
//                 // mids.push_back(mid);         // 実際の経路には残す

//                 /* ★ CFreeICS でクラスタ抽出 */

//         bool has_overlap = false;
//         for (auto& pc : cls)
//             if (prev_cloud.overlap(pc)) { prev_cloud = pc; has_overlap = true; break; }

//         if (!has_overlap)             { valid = false; break; }
//     }
//     else
//     {
//         DfsCFO dfs;
//         auto cnow = dfs.extract(prev_cloud, mid);
//         if (cnow.size() != 1)         { valid = false; break; }
//         prev_cloud = cnow[0];
//     }

//     /* ここでは mid を push しない＝経路には残さない */
//     check_pts.push_back(mid);         // 判定用に保持するだけ
// }


// /* ---------- 成功した区間を実際に置き換える ---------- */
// if (valid)
// {
//     /* 中間ノードをすべて削除 (i+1 ～ j-1) */
//     result.elm.erase(result.elm.begin() + i + 1,
//                      result.elm.begin() + j);
//     /* ★挿入ゼロ★ ── 端点 n1 と n2 を直接つなぐ */
//     ++success_cnt;
// }
// else
// {
//     ++fail_cnt;
// }  
// }
// std::cout << "[INFO] success " << success_cnt
//           << " / trials " << (success_cnt + fail_cnt)
//           << "  final size = " << result.size() << '\n';
// return result;
// }   // ← NodeList PathShortcut::shortcut() の閉じ括弧
// =========================================================



//         for (int k = 1; k < steps; ++k)
//         {
//             double t = double(k) / steps;
//             Node mid = n1.interpolate(n2, t);

//             if (!robot_update(mid)) { valid = false; break; }

//             /* ★ 3 点に 1 回だけ CFreeICS で厳密チェック */
//             if (k % 4 == 0) {
//                 auto cls = CFreeICS(mid).extract();
//                 if (cls.size() == 0) { valid = false; break; }

//                 /* オーバラップしていれば OK */
//                 bool overlap = false;
//                 for (auto& pc : cls)
//                     if (prev_cloud.overlap(pc)) { prev_cloud = pc; overlap = true; break; }
//                 if (!overlap) { valid = false; break; }
//             } else {
//                 /* 旧 dfs 判定 */
//                 DfsCFO dfs;
//                 auto cnow = dfs.extract(prev_cloud, mid);
//                 if (cnow.size() != 1) { valid = false; break; }
//                 prev_cloud = cnow[0];
//             }
//             mids.push_back(mid);
//         } // end for steps

//         if (!valid) { ++fail_cnt; continue; }

//         /* 行数削減が十分か判定を緩める */
//         int removed  = j - i - 1;
//         int inserted = mids.size();
//         // if (inserted <= removed + 4) {          // ★ ここを緩めた
//         double ratio = double(inserted) / removed;   // 例: 0.75 なら 25% 減
//         if (ratio <= 0.8) { 
//             result.elm.erase(result.elm.begin()+i+1, result.elm.begin()+j);
//             result.elm.insert(result.elm.begin()+i+1, mids.begin(), mids.end());
//             ++success_cnt;
//         } else {
//             ++fail_cnt;
//         }
//     } // end trials

//     std::cout << "[INFO] success " << success_cnt
//               << " / trials " << (success_cnt + fail_cnt)
//               << "  final size = " << result.size() << '\n';
//     return result;
// }

//租パス---------------------------------------------------------------
// void PathShortcut::shrink_phase(NodeList& result, bool check_reset)
// {
//     std::mt19937 rng(std::random_device{}());
//     // const int max_trials = result.size() * 50;      // 適当に多め
//     const int    N     = result.size();
//     const int    MAX_TRIALS = N * 50;     // 試行上限
//     const int    MAX_FAIL   = N * 10;     // 失敗連続で打ち切り
//     const auto   TIME_LIMIT = std::chrono::seconds(900); // 15 min
//     auto t_start = std::chrono::steady_clock::now();
//     int  consecutive_fail = 0;

//     for (int trial = 0; trial < MAX_TRIALS; ++trial) {

//         /* ----- タイムアウト ---- */
//         if (std::chrono::steady_clock::now() - t_start > TIME_LIMIT) break;
//         if (consecutive_fail > MAX_FAIL) break;
//         if (result.size() < 3) break;

//         /* ----- 区間選択 (離れ過ぎ抑止) ---- */
//         std::uniform_int_distribution<> di(0, result.size() - 3);
//         int i = di(rng);
//         std::uniform_int_distribution<> dj(i + 2, 
//                           std::min(i + 30, (int)result.size() - 1));
//         int j = dj(rng);

//         if (check_reset && j - i > 10) continue;

//         Node n1 = result[i], n2 = result[j];

//         /* Adaptive steps */
//         double ang = n1.norm(n2);
//         int steps = std::clamp((int)std::ceil(ang / 2.0), 10, 120);

//         PointCloud prev_cloud;
//         auto cls0 = CFreeICS(n1).extract();
//         if (cls0.size() == 0) { ++consecutive_fail; continue; }
//         prev_cloud = *std::max_element(cls0.begin(), cls0.end(),
//                     [](const PointCloud& a,const PointCloud& b){
//                         return a.size() < b.size();});

//         bool valid = true;
//         std::vector<Node> mids;

//         for (int k = 1; k < steps; ++k) {
//             double t = double(k)/steps;
//             Node mid = n1.interpolate(n2, t);

//             if (!robot_update(mid)) { valid=false; break; }

//             if (check_reset) {
//                 auto cls = CFreeICS(mid).extract();
//                 if (cls.size()!=1){ valid=false; break; }
//                 prev_cloud = cls[0];
//             }
//             else {
//                 DfsCFO dfs;
//                 auto cnow = dfs.extract(prev_cloud, mid);
//                 if (cnow.size()!=1){ valid=false; break; }
//                 prev_cloud = cnow[0];
//             }
//             mids.push_back(mid);
//         }
//         if (!valid) { ++consecutive_fail; continue; }
//         consecutive_fail = 0;            // 成功したのでリセット

//         result.elm.erase(result.elm.begin()+i+1, result.elm.begin()+j);
//         result.elm.insert(result.elm.begin()+i+1, mids.begin(), mids.end());
//     }
// }
//--------------------------------------------------------------------
    // for (int trial = 0; trial < max_trials; ++trial) {
    //     if (result.size() < 3) break;

    //     // --- i , j を「遠くを選びやすい」乱数にする ---
    //     std::uniform_int_distribution<> dist_i(0, result.size() - 3);
    //     int i = dist_i(rng);
    //     std::uniform_int_distribution<> dist_j(i + 2, result.size() - 1);
    //     int j = dist_j(rng);

    //     Node n1 = result[i], n2 = result[j];

    //     /* Adaptive steps */
    //     double ang = n1.norm(n2);
    //     int steps = std::clamp( int(std::ceil(ang / 2.0)), 10, 100 );

    //     /* 最大 3×range くらいまでは削除を許す (= 粗パス) */
    //     if (check_reset && j - i > 10) continue;

    //     PointCloud prev_cloud;
    //     /* クラスタ初期化（最大クラスタ） */
    //     {
    //         auto cls = CFreeICS(n1).extract();
    //         if (cls.empty()) continue;
    //         prev_cloud = *std::max_element(
    //             cls.begin(), cls.end(),
    //             [](const PointCloud& a,const PointCloud& b){
    //                 return a.size()<b.size();
    //             });
    //     }

//         bool valid = true;
//         std::vector<Node> mids;
//         for (int k = 1; k < steps; ++k) {
//             double t = double(k)/steps;
//             Node mid = n1.interpolate(n2,t);
//             if (!robot_update(mid)) { valid=false; break; }

//             /* 再抽出判定を on/off */
//             if (check_reset) {
//                 CFreeICS ics(mid);
//                 auto cls = ics.extract();
//                 if (cls.size()!=1) { valid=false; break;}
//                 prev_cloud = cls[0];
//             }
//             else {                 // 旧方式
//                 DfsCFO dfs;
//                 auto cnow = dfs.extract(prev_cloud,mid);
//                 if (cnow.size()!=1){ valid=false; break;}
//                 prev_cloud = cnow[0];
//             }
//             mids.push_back(mid);
//         }
//         if (!valid) continue;

//         /* 削除 / 挿入 */
//         result.elm.erase(result.elm.begin()+i+1, result.elm.begin()+j);
//         result.elm.insert(result.elm.begin()+i+1, mids.begin(), mids.end());
//     }
// }
//-------------------------------------------------------------------------------------

//検証フェーズ---------------------------------------------------------------------------
// void PathShortcut::validate_and_fix(NodeList& path)
// {
//     const double STEP_DEG = 2.0;        // 1° ピッチで検証
//     for (int i = 0; i < path.size()-1; ++i) {
//         Node a = path[i];
//         Node b = path[i+1];

//         int steps = std::max( 8, (int)std::ceil(a.norm(b)/STEP_DEG) );
//         steps = std::min(steps, 40);                 // ★ ④ 上限 40
//         PointCloud dummy;               // 無視

//         bool ok = true;
//         for (int k=1;k<steps;++k){
//             double t = double(k)/steps;
//             Node mid = a.interpolate(b,t);

//             auto cls = CFreeICS(mid).extract();
//             if (cls.size()!=1)
//             { 
//                 ok=false; 
//                 break;
//             }
//         }
//         if (!ok){               // ← 分裂区間を発見
//             /* ① 中割りを戻す (単純): */
//             path.elm.insert(path.elm.begin()+i+1, b); // b をコピーして 2 分割
//             --i;                                      // 現在区間を再チェック

//             /* ② もっと攻めるなら:
//                NodeList seg = {a,b};
//                shrink_phase(seg,true);   // 細粒ショートカット
//                path.elm.erase(...)
//                path.elm.insert(... seg.elm ...)
//             */
//         }
//     }
// }
//-------------------------------------------------------------------------------------

// NodeList PathShortcut::shortcut()
// {
//     NodeList result = path;
//     shrink_phase(result,false);     // 粗パス
//     validate_and_fix(result);       // 後判定で修復
//     std::cout<<"[INFO] final size="<<result.size()<<'\n';
//     return result;
// }

// bool PathShortcut::caging_valid(PointCloud& prev_cloud, const Node& now, const Node& next) 
// {
//     // DfsCFO dfs;
//     // auto cfo_now = dfs.extract(prev, now);
//     // if (cfo_now.size() != 1) return false;

//     // auto cfo_next = dfs.extract(cfo_now[0], next);
//     // return (cfo_next.size() == 1);

//     /* ① 今の姿勢で C_free_obj が 1 つか確認 */
//     CFreeICS ics_now(now);
//     auto cls_now = ics_now.extract();
//     if (cls_now.size() != 1) return false;          // 分裂・空クラスタ → NG

//     prev_cloud = cls_now[0];                        // クラスタをリセット

//     /* ② now → next 移動の可否は従来と同じ */
//     DfsCFO dfs;
//     auto cfo_next = dfs.extract(prev_cloud, next);
//     return (cfo_next.size() == 1);
// }

// NodeList PathShortcut::shortcut()
// {
//     NodeList result = path;
//     auto t_start = std::chrono::steady_clock::now();
//     const auto TIME_LIMIT = std::chrono::minutes(30);

//     std::mt19937 rng(std::random_device{}());
//     std::exponential_distribution<> expdist(0.1); // 平均10行

//     int target_success = std::max( (int)result.size()/2, 20 );
//     int success_cnt = 0, fail_cnt = 0;

//     while (success_cnt < target_success)
//     {
//         if (std::chrono::steady_clock::now()-t_start > TIME_LIMIT) break;
//         if (fail_cnt > result.size() * 10) break;
//         if (result.size() < 3) break;

//         /* ---- 区間選択：指数分布で j-i を抽選 ---- */
//         int len = std::clamp( (int)std::round(expdist(rng)*15)+2, 2, 30 );
//         std::uniform_int_distribution<> di(0, result.size()-len-1);
//         int i = di(rng), j = i + len;

//         Node n1 = result[i], n2 = result[j];

//         /* ---- Adaptive steps = ceil(dist*1.5) ---- */
//         double ang = n1.norm(n2);                // deg
//         int steps = std::clamp( (int)std::ceil(ang*1.5), 8, 80 );

//         /* ---- 最大クラスタ取得 ---- */
//         auto icls = CFreeICS(n1).extract();
//         if (icls.empty()) { ++fail_cnt; continue; }
//         PointCloud prev_cloud = *std::max_element(
//             icls.begin(), icls.end(),
//             [](auto& a, auto& b){ return a.size() < b.size(); });

//         bool valid = true;
//         std::vector<Node> mids;

//         for (int k=1; k<steps; ++k){
//             double t = double(k)/steps;
//             Node mid = n1.interpolate(n2,t);

//             if(!robot_update(mid)){ valid=false; break; }

//             auto cls = CFreeICS(mid).extract();
//             if(cls.size()!=1){ valid=false; break; }
//             prev_cloud = cls[0];

//             mids.push_back(mid);
//         }

//         if(!valid){ ++fail_cnt; continue; }

//         /* ---- 採用 ---- */
//         // result.elm.erase(result.elm.begin()+i+1, result.elm.begin()+j);
//         // result.elm.insert(result.elm.begin()+i+1, mids.begin(), mids.end());
//         // ++success_cnt;
//         /* --- ↓ 置き換え（削除より多いなら棄却） --- */
//         int removed = j - i - 1;
//         int inserted = mids.size();
//         if (inserted < removed) {                         // 行数が確実に減るときだけ採用
//             result.elm.erase(result.elm.begin()+i+1, result.elm.begin()+j);
//             result.elm.insert(result.elm.begin()+i+1, mids.begin(), mids.end());
//             ++success_cnt;
//         } else {
//             ++fail_cnt;                                   // 失敗扱い
//         }
//     }

//     std::cout << "[INFO] success " << success_cnt
//               << " / target "    << target_success
//               << "  final size = " << result.size() << '\n';
//     return result;
// }

// NodeList PathShortcut::shortcut()
// {
//     NodeList result = path;
//     shrink_phase(result, /*check_reset=*/false);   // ← クラスタ固定
//     shrink_phase(result, /*check_reset=*/true );   // ← 再抽出あり
//     std::cout << "[INFO] final size = " << result.size() << '\n';
//     return result;
// }

// NodeList PathShortcut::shortcut() 
// {
//     std::mt19937 rng(std::random_device{}());
//     NodeList result = path;

//     for (int trial = 0; trial < trials; ++trial) {
//         if (result.size() < 3) break;
//         std::uniform_int_distribution<> dist(0, result.size() - 1);
//         int i = dist(rng), j = dist(rng);
//         if (i == j) continue;
//         if (i > j) std::swap(i, j);
//         if (j - i < 2) continue;

//         Node n1 = result[i];
//         Node n2 = result[j];

//         // 線形補間
//         // const int steps = 20;
//         /* ★ここを追加：角度差に応じて steps を決定 */
//         double angle_dist = n1.norm(n2);            // ユークリッド距離[deg]
//         int steps = static_cast<int>(std::ceil(angle_dist / 2.0)); // 2°ごと
//         steps = std::clamp(steps, 10, 100);
//         //10から100の間に制限　追加
//         std::vector<Node> interpolated;
//         bool valid = true;
//         PointCloud prev_cloud;

//         // 初期クラスタを取得
//         // ──────── ここから置き換え ────────
//         // 初期クラスタを取得（最大サイズを自動選択）
//         {
//             CFreeICS ics(n1);
//             auto init_cfree = ics.extract();
//             if (init_cfree.empty()) continue;

//             // サイズが最大のクラスタを prev_cloud に採用
//             prev_cloud = *std::max_element(
//                 init_cfree.begin(), init_cfree.end(),
//                 [](const PointCloud& a, const PointCloud& b){
//                     return a.size() < b.size();
//                 });

//             // デバッグ表示（任意）
//             std::cout << "[INFO] auto-selected cluster size = "
//                       << prev_cloud.size() << '\n';
//         }
// // ──────── ここまで置き換え ────────

//         {
//             CFreeICS ics(n1);
//             auto init_cfree = ics.extract();
//             if (init_cfree.empty()) continue;

//             std::cout << "Select cluster index [0 - " << init_cfree.size() - 1 << "]: ";
//             int idx;
//             std::cin >> idx;
//             assert(0 <= idx && idx < (int)init_cfree.size());
//             prev_cloud = init_cfree[idx];
//         }

//         for (int k = 1; k < steps; ++k) {
//             double t = static_cast<double>(k) / steps;
//             Node mid = n1.interpolate(n2, t);

//             if (!robot_update(mid)) {
//                 valid = false;
//                 break;
//             }
//             if (!caging_valid(prev_cloud, mid, n2)) {
//                 valid = false;
//                 break;
//             }
//             interpolated.push_back(mid);
//         }

//         if (valid) {
//             result.elm.erase(result.elm.begin() + i + 1, result.elm.begin() + j);
//             result.elm.insert(result.elm.begin() + i + 1, interpolated.begin(), interpolated.end());
//         }
//     }
//     std::cout << "[INFO] shortcut result size = "
//               << result.size() << '\n';         // ← 追加⑤
//     return result;
// }


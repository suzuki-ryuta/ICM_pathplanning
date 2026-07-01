#pragma once
#include "PointCloud.h"
#include <vector>
#include <cmath>
#include <algorithm>

constexpr double NEIGHBOR_DIST = 1.5;

// 隣接行列（隣接リスト）を構築
std::vector<std::vector<int>> build_adjacency(const PointCloud& pc);

// Laplacian行列を作成（対角 - 隣接）
std::vector<std::vector<double>> compute_laplacian(const std::vector<std::vector<int>>& adj);

// 固有値（ダミー）計算
std::vector<double> compute_sorted_eigenvalues(const std::vector<std::vector<double>>& L);

// 固有値スペクトル取得（←これが compute_laplacian → compute_sorted_eigenvalues を使う関数）
std::vector<double> calculate_laplacian_spectrum(const PointCloud& pc);

// 上位k個の固有値を返す便利関数（これがpathshortcut.cppで使われる関数）
std::vector<double> compute_laplacian_eigenvalues(const PointCloud& pc, int num_eigen = 5);


// #pragma once
// #include "PointCloud.h"
// #include <vector>
// #include <cmath>
// #include <algorithm>

// // 距離閾値でつながってるとみなす
// constexpr double NEIGHBOR_DIST = 1.5;

// std::vector<std::vector<int>> build_adjacency(const PointCloud& pc) {
//     int n = pc.size();
//     std::vector<std::vector<int>> adj(n);
//     for (int i = 0; i < n; ++i)
//         for (int j = i + 1; j < n; ++j)
//             if (pc[i].dist(pc[j]) < NEIGHBOR_DIST)
//                 adj[i].push_back(j), adj[j].push_back(i);
//     return adj;
// }

// std::vector<std::vector<double>> compute_laplacian(const std::vector<std::vector<int>>& adj) {
//     int n = adj.size();
//     std::vector<std::vector<double>> L(n, std::vector<double>(n, 0.0));
//     for (int i = 0; i < n; ++i) {
//         L[i][i] = adj[i].size();
//         for (int j : adj[i]) {
//             L[i][j] = -1;
//         }
//     }
//     return L;
// }

// std::vector<double> compute_sorted_eigenvalues(const std::vector<std::vector<double>>& L) {
//     // ← LAPACKやEigenライブラリを使用してください
//     // ダミー：固有値が降順と仮定して返す
//     std::vector<double> eigs(L.size(), 1.0);  // 固有値計算が必要
//     return eigs;
// }
// std::vector<double> compute_laplacian_eigenvalues(const PointCloud& pc, int num_eigen = 5) {
//     // グラフ生成など省略
//     // Laplacian 行列を作成 → 固有値計算

//     std::vector<double> eigenvalues = calculate_laplacian_spectrum(pc); // ← 全固有値取得

//     // 上位 num_eigen 個のみ返す（ゼロ近傍成分除去済み）
//     if ((int)eigenvalues.size() > num_eigen)
//         eigenvalues.resize(num_eigen);
//     return eigenvalues;
// }
// std::vector<double> calculate_laplacian_spectrum(const PointCloud& pc);
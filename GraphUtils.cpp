#include "GraphUtils.h"
#include "PointCloud.h"
#include <Eigen/Dense>
#include <cmath>
#include <algorithm>

double compute_distance(const State3D& a, const State3D& b) {
    double dx = static_cast<double>(a.x - b.x);
    double dy = static_cast<double>(a.y - b.y);
    return std::sqrt(dx * dx + dy * dy);
}

// std::vector<double> calculate_laplacian_spectrum(const PointCloud& pc) {
//     const int N = pc.size();
//     if (N == 0) return {};

//     Eigen::MatrixXd W = Eigen::MatrixXd::Zero(N, N);  // 隣接行列

//     for (int i = 0; i < N; ++i) {
//         for (int j = i + 1; j < N; ++j) {
//             double dist = compute_distance(pc.get(i), pc.get(j));
//             if (dist < NEIGHBOR_DIST) {
//                 W(i, j) = 1.0;
//                 W(j, i) = 1.0;
//             }
//         }
//     }

//     Eigen::VectorXd degrees = W.rowwise().sum();
//     Eigen::MatrixXd D = degrees.asDiagonal();

//     Eigen::MatrixXd L = D - W;

//     Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigensolver(L);
//     if (eigensolver.info() != Eigen::Success) {
//         return {};
//     }

//     std::vector<double> eigenvalues;
//     Eigen::VectorXd evals = eigensolver.eigenvalues();
//     for (int i = 0; i < evals.size(); ++i) {
//         eigenvalues.push_back(evals[i]);
//     }

//     std::sort(eigenvalues.begin(), eigenvalues.end());
//     return eigenvalues;
// }

std::vector<std::vector<int>> build_adjacency(const PointCloud& pc) {
    int n = pc.size();
    std::vector<std::vector<int>> adj(n);

    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            double dist = compute_distance(pc.get(i), pc.get(j));
            if (dist < NEIGHBOR_DIST) {
                adj[i].push_back(j);
                adj[j].push_back(i);
            }
        }
    }

    return adj;
}

std::vector<std::vector<double>> compute_laplacian(const std::vector<std::vector<int>>& adj) {
    int n = adj.size();
    std::vector<std::vector<double>> L(n, std::vector<double>(n, 0.0));
    for (int i = 0; i < n; ++i) {
        L[i][i] = adj[i].size();
        for (int j : adj[i]) {
            L[i][j] = -1.0;
        }
    }
    return L;
}

// ダミーの固有値（あとでEigenに置き換える）
std::vector<double> compute_sorted_eigenvalues(const std::vector<std::vector<double>>& L) {
    std::vector<double> eigs(L.size(), 1.0);
    return eigs;
}

std::vector<double> calculate_laplacian_spectrum(const PointCloud& pc) {
    auto adj = build_adjacency(pc);
    auto lap = compute_laplacian(adj);
    return compute_sorted_eigenvalues(lap);
}

std::vector<double> compute_laplacian_eigenvalues(const PointCloud& pc, int num_eigen) {
    std::vector<double> eigenvalues = calculate_laplacian_spectrum(pc);
    if ((int)eigenvalues.size() > num_eigen)
        eigenvalues.resize(num_eigen);
    return eigenvalues;
}

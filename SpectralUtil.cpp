#include "SpectralUtil.h"
#include <Eigen/Dense>
#include <cmath>
#include <algorithm>

std::vector<double> compute_laplacian_spectrum(const PointCloud& pc, int topk) {
    const int N = pc.size();
    if (N == 0) return {};

    Eigen::MatrixXd adj = Eigen::MatrixXd::Zero(N, N);

    // 隣接行列を距離ベースで構築（閾値内なら1）
    const double threshold = 10.0;
    for (int i = 0; i < N; ++i) {
        for (int j = i + 1; j < N; ++j) {
            double dx = pc.get(i).x - pc.get(j).x;
            double dy = pc.get(i).y - pc.get(j).y;
            double d = std::sqrt(dx*dx + dy*dy);
            if (d < threshold) {
                adj(i, j) = adj(j, i) = 1.0;
            }
        }
    }

    Eigen::VectorXd degrees = adj.rowwise().sum();
    Eigen::MatrixXd deg = degrees.asDiagonal();
    Eigen::MatrixXd laplacian = deg - adj;

    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(laplacian);
    if (solver.info() != Eigen::Success) {
        return {};
    }

    std::vector<double> spectrum(topk);
    auto eigvals = solver.eigenvalues();

    for (int i = 0; i < topk && i < eigvals.size(); ++i) {
        spectrum[i] = eigvals[i];
    }

    return spectrum;
}

double eigen_distance(const std::vector<double>& a, const std::vector<double>& b) {
    if (a.size() != b.size()) return 1e6;

    double dist = 0.0;
    for (size_t i = 0; i < a.size(); ++i) {
        double diff = a[i] - b[i];
        dist += diff * diff;
    }
    return std::sqrt(dist);
}

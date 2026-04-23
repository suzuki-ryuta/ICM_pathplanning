#pragma once
#include "PointCloud.h"
#include <vector>

std::vector<double> compute_laplacian_spectrum(const PointCloud& pc, int topk = 10);
double eigen_distance(const std::vector<double>& a, const std::vector<double>& b);

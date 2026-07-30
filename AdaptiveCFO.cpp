#include <algorithm>
#include <climits>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stack>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "CFree.h"

namespace {

struct GridSpec
{
	State3D bottom;
	State3D top;
	Vector3D<int> fine_step;
	Vector3D<int> coarse_step;
	int numx;
	int numy;
	int numth;
};

struct GridKey
{
	int ix;
	int iy;
	int ith;

	bool operator==(const GridKey& other) const
	{
		return ix == other.ix && iy == other.iy && ith == other.ith;
	}
};

struct GridKeyHash
{
	std::size_t operator()(const GridKey& key) const
	{
		std::size_t h = 1469598103934665603ULL;
		h ^= (std::size_t)(key.ix + 1000003); h *= 1099511628211ULL;
		h ^= (std::size_t)(key.iy + 1000033); h *= 1099511628211ULL;
		h ^= (std::size_t)(key.ith + 1000037); h *= 1099511628211ULL;
		return h;
	}
};

struct StateKey
{
	int x;
	int y;
	int th;

	bool operator==(const StateKey& other) const
	{
		return x == other.x && y == other.y && th == other.th;
	}
};

struct StateKeyHash
{
	std::size_t operator()(const StateKey& key) const
	{
		std::size_t h = 1469598103934665603ULL;
		h ^= (std::size_t)(key.x + 1000003); h *= 1099511628211ULL;
		h ^= (std::size_t)(key.y + 1000033); h *= 1099511628211ULL;
		h ^= (std::size_t)(key.th + 1000037); h *= 1099511628211ULL;
		return h;
	}
};

enum class CellStatus
{
	Blocked,
	Free,
	EscapeEdge
};

int read_adaptive_int(const std::string& name, int default_value)
{
	bp::ptree pt;
	try {
		bp::ini_parser::read_ini("config/SpaceConfig.ini", pt);
		return pt.get<int>("adaptive_cfo." + name, default_value);
	}
	catch (...) {
		return default_value;
	}
}

bool key_less(const GridKey& lhs, const GridKey& rhs)
{
	if (lhs.ix != rhs.ix) return lhs.ix < rhs.ix;
	if (lhs.iy != rhs.iy) return lhs.iy < rhs.iy;
	return lhs.ith < rhs.ith;
}

int clamp_index(int index, int count)
{
	if (index < 0) return 0;
	if (index >= count) return count - 1;
	return index;
}

int nearest_index(int value, int bottom, int count, int step)
{
	const double raw = (double)(value - bottom) / step;
	return clamp_index((int)std::llround(raw), count);
}

int ceil_to_grid(int value, int bottom, int step)
{
	if (value <= bottom) return bottom;
	const int offset = value - bottom;
	const int rem = offset % step;
	return rem == 0 ? value : value + (step - rem);
}

int floor_to_grid(int value, int bottom, int step)
{
	if (value <= bottom) return bottom;
	const int offset = value - bottom;
	return value - (offset % step);
}

GridSpec make_grid_spec(int stride_x, int stride_y, int stride_th)
{
	CSpaceConfig* cs = CSpaceConfig::get_instance();
	GridSpec spec;
	spec.bottom = cs->getbottom();
	spec.top = cs->gettop();
	spec.fine_step = cs->getrange();
	spec.coarse_step = Vector3D<int>(
		std::max(1, spec.fine_step.x * std::max(1, stride_x)),
		std::max(1, spec.fine_step.y * std::max(1, stride_y)),
		std::max(1, spec.fine_step.z * std::max(1, stride_th)));
	spec.numx = (spec.top.x - spec.bottom.x) / spec.coarse_step.x + 1;
	spec.numy = (spec.top.y - spec.bottom.y) / spec.coarse_step.y + 1;
	spec.numth = (spec.top.th - spec.bottom.th) / spec.coarse_step.z + 1;
	return spec;
}

GridKey point_to_key(State3D st, const GridSpec& spec)
{
	return GridKey{
		nearest_index(st.x, spec.bottom.x, spec.numx, spec.coarse_step.x),
		nearest_index(st.y, spec.bottom.y, spec.numy, spec.coarse_step.y),
		nearest_index(st.th, spec.bottom.th, spec.numth, spec.coarse_step.z)
	};
}

State3D key_to_state(GridKey key, const GridSpec& spec)
{
	return State3D(
		spec.bottom.x + key.ix * spec.coarse_step.x,
		spec.bottom.y + key.iy * spec.coarse_step.y,
		spec.bottom.th + key.ith * spec.coarse_step.z);
}

bool valid_xy_key(GridKey key, const GridSpec& spec)
{
	return 0 <= key.ix && key.ix < spec.numx
	    && 0 <= key.iy && key.iy < spec.numy;
}

GridKey normalize_theta_key(GridKey key, const GridSpec& spec)
{
	if (key.ith < 0) key.ith += spec.numth;
	if (key.ith >= spec.numth) key.ith -= spec.numth;
	return key;
}

bool near_escape_edge(State3D st, const GridSpec& spec, int radius_cells)
{
	const int rx = std::max(spec.fine_step.x, spec.coarse_step.x * std::max(1, radius_cells));
	const int ry = std::max(spec.fine_step.y, spec.coarse_step.y * std::max(1, radius_cells));
	const int dx = std::min(st.x - spec.bottom.x, spec.top.x - st.x);
	const int dy = std::min(st.y - spec.bottom.y, spec.top.y - st.y);
	return dx <= rx || dy <= ry;
}

CellStatus fine_state_status(State3D st)
{
	if (edge_judge(st)) return CellStatus::EscapeEdge;

	Controller* controller = Controller::get_instance();
	controller->shape_update(st);
	if (controller->RintersectS()) return CellStatus::Blocked;
	if (controller->WintersectS()) return CellStatus::Blocked;
	return CellStatus::Free;
}

CellStatus sampled_cell_status(GridKey key, const GridSpec& spec, int strict_cell_sampling)
{
	key = normalize_theta_key(key, spec);
	if (!valid_xy_key(key, spec)) return CellStatus::Blocked;

	if (strict_cell_sampling == 0) {
		return fine_state_status(key_to_state(key, spec));
	}

	State3D center = key_to_state(key, spec);
	const int half_x = std::max(0, (spec.coarse_step.x - spec.fine_step.x) / 2);
	const int half_y = std::max(0, (spec.coarse_step.y - spec.fine_step.y) / 2);
	const int half_th = std::max(0, (spec.coarse_step.z - spec.fine_step.z) / 2);
	const int theta_period = spec.top.th + spec.fine_step.z;

	std::vector<int> x_offsets = {0};
	std::vector<int> y_offsets = {0};
	std::vector<int> th_offsets = {0};
	if (half_x > 0) x_offsets = {-half_x, 0, half_x};
	if (half_y > 0) y_offsets = {-half_y, 0, half_y};
	if (half_th > 0) th_offsets = {-half_th, 0, half_th};

	for (int dx : x_offsets) {
		const int x = center.x + dx;
		if (x < spec.bottom.x || x > spec.top.x) return CellStatus::EscapeEdge;
		for (int dy : y_offsets) {
			const int y = center.y + dy;
			if (y < spec.bottom.y || y > spec.top.y) return CellStatus::EscapeEdge;
			for (int dth : th_offsets) {
				int th = center.th + dth;
				while (th < spec.bottom.th) th += theta_period;
				while (th > spec.top.th) th -= theta_period;

				CellStatus status = fine_state_status(State3D(x, y, th));
				if (status != CellStatus::Free) return status;
			}
		}
	}

	return CellStatus::Free;
}

CellStatus cached_status(
	GridKey key,
	const GridSpec& spec,
	std::unordered_map<GridKey, CellStatus, GridKeyHash>& cache,
	int strict_cell_sampling)
{
	key = normalize_theta_key(key, spec);
	if (!valid_xy_key(key, spec)) return CellStatus::Blocked;

	auto found = cache.find(key);
	if (found != cache.end()) return found->second;

	CellStatus status = sampled_cell_status(key, spec, strict_cell_sampling);
	cache.emplace(key, status);
	return status;
}

StateKey make_state_key(State3D st)
{
	return StateKey{st.x, st.y, st.th};
}

CellStatus cached_fine_status(
	State3D st,
	std::unordered_map<StateKey, CellStatus, StateKeyHash>& cache)
{
	StateKey key = make_state_key(st);
	auto found = cache.find(key);
	if (found != cache.end()) return found->second;

	CellStatus status = fine_state_status(st);
	cache.emplace(key, status);
	return status;
}

int round_to_grid(double value, int bottom, int step)
{
	return bottom + (int)std::llround((value - bottom) / step) * step;
}

int wrapped_theta_delta(int from, int to, const GridSpec& spec)
{
	const int period = spec.top.th + spec.fine_step.z;
	int delta = to - from;
	if (delta > period / 2) delta -= period;
	if (delta < -period / 2) delta += period;
	return delta;
}

bool coarse_connection_free(
	GridKey from_key,
	GridKey to_key,
	const GridSpec& spec,
	std::unordered_map<StateKey, CellStatus, StateKeyHash>& fine_cache)
{
	State3D from = key_to_state(from_key, spec);
	State3D to = key_to_state(to_key, spec);
	const int dx = to.x - from.x;
	const int dy = to.y - from.y;
	const int dth = wrapped_theta_delta(from.th, to.th, spec);

	const int nx = std::abs(dx) / spec.fine_step.x;
	const int ny = std::abs(dy) / spec.fine_step.y;
	const int nth = std::abs(dth) / spec.fine_step.z;
	const int steps = std::max(1, std::max(nx, std::max(ny, nth)));
	const int theta_period = spec.top.th + spec.fine_step.z;

	for (int i = 1; i <= steps; ++i) {
		const double t = (double)i / steps;
		int x = round_to_grid(from.x + dx * t, spec.bottom.x, spec.fine_step.x);
		int y = round_to_grid(from.y + dy * t, spec.bottom.y, spec.fine_step.y);
		int th = round_to_grid(from.th + dth * t, spec.bottom.th, spec.fine_step.z);

		while (th < spec.bottom.th) th += theta_period;
		while (th > spec.top.th) th -= theta_period;
		if (x < spec.bottom.x || x > spec.top.x) return false;
		if (y < spec.bottom.y || y > spec.top.y) return false;

		if (cached_fine_status(State3D(x, y, th), fine_cache) != CellStatus::Free) {
			return false;
		}
	}

	return true;
}

void append_component(PointCloud& dst, const PointCloud& src)
{
	for (int i = 0; i < src.size(); ++i) {
		dst.push_from(src, i);
	}
}

void merge_overlapping_components(std::vector<PointCloud>& components)
{
	bool changed = true;
	while (changed) {
		changed = false;
		for (int i = 0; i < (int)components.size() && !changed; ++i) {
			for (int j = i + 1; j < (int)components.size(); ++j) {
				if (!components[i].overlap(components[j])) continue;
				append_component(components[i], components[j]);
				components.erase(components.begin() + j);
				changed = true;
				break;
			}
		}
	}
}

bool refine_boundary_cells(
	PointCloud& component,
	const std::unordered_set<GridKey, GridKeyHash>& boundary_keys,
	const GridSpec& spec,
	int refine_radius_cells,
	int max_boundary_seeds,
	int max_refine_checks,
	int max_refined_points)
{
	std::vector<GridKey> seeds(boundary_keys.begin(), boundary_keys.end());
	std::sort(seeds.begin(), seeds.end(), key_less);
	if ((int)seeds.size() > max_boundary_seeds) {
		seeds.resize(max_boundary_seeds);
	}

	std::unordered_set<StateKey, StateKeyHash> inserted;
	inserted.reserve(component.size() + max_refined_points);
	for (int i = 0; i < component.size(); ++i) {
		State3D st = component.get(i);
		inserted.insert(StateKey{st.x, st.y, st.th});
	}

	const int extra_cells = std::max(0, refine_radius_cells - 1);
	const int half_x = std::max(spec.fine_step.x, (spec.coarse_step.x - spec.fine_step.x) / 2);
	const int half_y = std::max(spec.fine_step.y, (spec.coarse_step.y - spec.fine_step.y) / 2);
	const int half_th = std::max(spec.fine_step.z, (spec.coarse_step.z - spec.fine_step.z) / 2);
	const int radius_x = half_x + spec.coarse_step.x * extra_cells;
	const int radius_y = half_y + spec.coarse_step.y * extra_cells;
	const int radius_th = half_th + spec.coarse_step.z * extra_cells;
	const int theta_period = spec.top.th + spec.fine_step.z;
	int checks = 0;
	int refined_points = 0;

	for (const GridKey& seed_key : seeds) {
		State3D center = key_to_state(seed_key, spec);
		const int xmin = ceil_to_grid(std::max(spec.bottom.x, center.x - radius_x), spec.bottom.x, spec.fine_step.x);
		const int xmax = floor_to_grid(std::min(spec.top.x, center.x + radius_x), spec.bottom.x, spec.fine_step.x);
		const int ymin = ceil_to_grid(std::max(spec.bottom.y, center.y - radius_y), spec.bottom.y, spec.fine_step.y);
		const int ymax = floor_to_grid(std::min(spec.top.y, center.y + radius_y), spec.bottom.y, spec.fine_step.y);

		for (int x = xmin; x <= xmax; x += spec.fine_step.x) {
			for (int y = ymin; y <= ymax; y += spec.fine_step.y) {
				for (int dth = -radius_th; dth <= radius_th; dth += spec.fine_step.z) {
					int th = center.th + dth;
					while (th < spec.bottom.th) th += theta_period;
					while (th > spec.top.th) th -= theta_period;

					StateKey exact_key{x, y, th};
					if (inserted.find(exact_key) != inserted.end()) continue;
					if (++checks > max_refine_checks) return false;

					State3D st(x, y, th);
					CellStatus status = fine_state_status(st);
					if (status == CellStatus::EscapeEdge) return true;
					if (status != CellStatus::Free) continue;

					component.push(st, spec.fine_step);
					inserted.insert(exact_key);
					if (++refined_points >= max_refined_points) return false;
				}
			}
		}
	}

	return false;
}

bool pointcloud_contains_theta_edge(const PointCloud& pc)
{
	const int edge = CSpaceConfig::get_instance()->getsymangle();
	for (int i = 0; i < pc.size(); ++i) {
		State3D st = pc.get(i);
		if (st.th == 0 || st.th == edge) return true;
	}
	return false;
}

State3D pointcloud_min_corner(const PointCloud& pc)
{
	State3D key(INT_MAX, INT_MAX, INT_MAX);
	for (int i = 0; i < pc.size(); ++i) {
		State3D st = pc.get(i);
		if (st.x < key.x) key.x = st.x;
		if (st.y < key.y) key.y = st.y;
		if (st.th < key.th) key.th = st.th;
	}
	return key;
}

void sort_initial_components(std::vector<PointCloud>& components)
{
	std::sort(components.begin(), components.end(), [](const PointCloud& lhs, const PointCloud& rhs) {
		const bool lhs_edge = pointcloud_contains_theta_edge(lhs);
		const bool rhs_edge = pointcloud_contains_theta_edge(rhs);
		if (lhs_edge != rhs_edge) return lhs_edge;

		State3D lmin = pointcloud_min_corner(lhs);
		State3D rmin = pointcloud_min_corner(rhs);
		if (lmin.x != rmin.x) return lmin.x < rmin.x;
		if (lmin.y != rmin.y) return lmin.y < rmin.y;
		return lmin.th < rmin.th;
	});
}

} // namespace

AdaptiveDfsCFO::AdaptiveDfsCFO()
	: stride_x(read_adaptive_int("stride_x", 1)),
	  stride_y(read_adaptive_int("stride_y", 1)),
	  stride_th(read_adaptive_int("stride_th", 1)),
	  refine_boundary(read_adaptive_int("refine_boundary", 1)),
	  refine_radius_cells(read_adaptive_int("refine_radius_cells", 1)),
	  max_boundary_seeds(read_adaptive_int("max_boundary_seeds", 160)),
	  max_refine_checks(read_adaptive_int("max_refine_checks", 30000)),
	  max_refined_points(read_adaptive_int("max_refined_points", 6000)),
	  check_coarse_connections(read_adaptive_int("check_coarse_connections", 0)),
	  strict_cell_sampling(read_adaptive_int("strict_cell_sampling", 1))
{
	stride_x = std::max(1, stride_x);
	stride_y = std::max(1, stride_y);
	stride_th = std::max(1, stride_th);
	refine_radius_cells = std::max(1, refine_radius_cells);
	max_boundary_seeds = std::max(1, max_boundary_seeds);
	max_refine_checks = std::max(1, max_refine_checks);
	max_refined_points = std::max(1, max_refined_points);
}

std::vector<PointCloud> AdaptiveDfsCFO::extract(PointCloud prev, Node newnode)
{
	std::vector<PointCloud> components;
	if (prev.empty()) return components;

	Controller* controller = Controller::get_instance();
	controller->robot_update(newnode);

	GridSpec spec = make_grid_spec(stride_x, stride_y, stride_th);
	std::unordered_map<GridKey, CellStatus, GridKeyHash> status_cache;
	std::unordered_map<StateKey, CellStatus, StateKeyHash> fine_status_cache;
	std::unordered_set<GridKey, GridKeyHash> seed_set;
	std::unordered_set<GridKey, GridKeyHash> mixed_seed_keys;
	std::vector<GridKey> seeds;
	seed_set.reserve(prev.size());
	mixed_seed_keys.reserve(prev.size());
	fine_status_cache.reserve(prev.size() * 2 + 128);

	for (int i = 0; i < prev.size(); ++i) {
		GridKey key = point_to_key(prev.get(i), spec);
		if (seed_set.find(key) != seed_set.end()) continue;

		CellStatus exact_status = cached_fine_status(prev.get(i), fine_status_cache);
		if (exact_status == CellStatus::Free) {
			CellStatus center_status = cached_status(key, spec, status_cache, strict_cell_sampling);
			if (center_status != CellStatus::Free) {
				status_cache[key] = CellStatus::Free;
				mixed_seed_keys.insert(key);
			}
			seed_set.insert(key);
			seeds.push_back(key);
		}
	}

	std::sort(seeds.begin(), seeds.end(), key_less);

	std::unordered_set<GridKey, GridKeyHash> visited;
	visited.reserve(seeds.size() * 4 + 128);

	static const int dirs[26][3] = {
		{-1,-1,-1},{0,-1,-1},{1,-1,-1},{-1,0,-1},{0,0,-1},{1,0,-1},{-1,1,-1},{0,1,-1},{1,1,-1},
		{-1,-1,0},{0,-1,0},{1,-1,0},{-1,0,0},{1,0,0},{-1,1,0},{0,1,0},{1,1,0},
		{-1,-1,1},{0,-1,1},{1,-1,1},{-1,0,1},{0,0,1},{1,0,1},{-1,1,1},{0,1,1},{1,1,1}
	};

	for (const GridKey& seed : seeds) {
		if (visited.find(seed) != visited.end()) continue;
		if (cached_status(seed, spec, status_cache, strict_cell_sampling) != CellStatus::Free) continue;

			PointCloud component;
			component.set_default_cell_size(strict_cell_sampling == 0 ? spec.fine_step : spec.coarse_step);
		std::unordered_set<GridKey, GridKeyHash> boundary_keys;
		std::stack<GridKey> stack;
		bool escaped = false;

		visited.insert(seed);
		stack.push(seed);

		while (!stack.empty() && !escaped) {
			GridKey current = stack.top();
			stack.pop();
			State3D current_state = key_to_state(current, spec);
			component.push(current_state, spec.coarse_step);

			if (near_escape_edge(current_state, spec, refine_radius_cells)
			    || mixed_seed_keys.find(current) != mixed_seed_keys.end()) {
				boundary_keys.insert(current);
			}

			for (const auto& dir : dirs) {
				GridKey next{current.ix + dir[0], current.iy + dir[1], current.ith + dir[2]};
				next = normalize_theta_key(next, spec);
				if (!valid_xy_key(next, spec)) {
					boundary_keys.insert(current);
					continue;
				}

				CellStatus status = cached_status(next, spec, status_cache, strict_cell_sampling);
				if (status == CellStatus::EscapeEdge) {
					escaped = true;
					boundary_keys.insert(current);
					break;
				}
				if (status == CellStatus::Blocked) {
					boundary_keys.insert(current);
					continue;
				}
				if (check_coarse_connections != 0
				    && !coarse_connection_free(current, next, spec, fine_status_cache)) {
					boundary_keys.insert(current);
					boundary_keys.insert(next);
					continue;
				}
				if (visited.insert(next).second) {
					stack.push(next);
				}
			}
		}

		if (escaped || component.empty()) continue;

		if (refine_boundary != 0) {
			escaped = refine_boundary_cells(
				component,
				boundary_keys,
				spec,
				refine_radius_cells,
				max_boundary_seeds,
				max_refine_checks,
				max_refined_points);
		}

		if (!escaped) {
			components.push_back(component);
		}
	}

	merge_overlapping_components(components);
	return components;
}

std::vector<PointCloud> extract_adaptive_initial_cfree(Node node)
{
	Controller* controller = Controller::get_instance();
	controller->robot_update(node);

	const int stride_x = std::max(1, read_adaptive_int("stride_x", 1));
	const int stride_y = std::max(1, read_adaptive_int("stride_y", 1));
	const int stride_th = std::max(1, read_adaptive_int("stride_th", 1));
	GridSpec spec = make_grid_spec(stride_x, stride_y, stride_th);

	const int coarse_grid_size = spec.numx * spec.numy * spec.numth;
	std::vector<std::vector<State3D>> cell_samples(coarse_grid_size);
	std::vector<unsigned char> free_cell(coarse_grid_size, 0);
	std::vector<unsigned char> escape_cell(coarse_grid_size, 0);
	std::vector<unsigned char> visited(coarse_grid_size, 0);

	const auto key_to_flat_index = [&spec](GridKey key) {
		return (key.ix * spec.numy + key.iy) * spec.numth + key.ith;
	};
	const auto flat_index_to_key = [&spec](int index) {
		const int ith = index % spec.numth;
		const int rest = index / spec.numth;
		const int iy = rest % spec.numy;
		const int ix = rest / spec.numy;
		return GridKey{ix, iy, ith};
	};

	const int half_x = std::max(0, (spec.coarse_step.x - spec.fine_step.x) / 2);
	const int half_y = std::max(0, (spec.coarse_step.y - spec.fine_step.y) / 2);
	const int half_th = std::max(0, (spec.coarse_step.z - spec.fine_step.z) / 2);
	const int theta_period = spec.top.th + spec.fine_step.z;

	std::vector<int> x_offsets = {0};
	std::vector<int> y_offsets = {0};
	std::vector<int> th_offsets = {0};
	if (half_x > 0) x_offsets = {-half_x, 0, half_x};
	if (half_y > 0) y_offsets = {-half_y, 0, half_y};
	if (half_th > 0) th_offsets = {-half_th, 0, half_th};

	for (int ix = 0; ix < spec.numx; ++ix) {
		for (int iy = 0; iy < spec.numy; ++iy) {
			for (int ith = 0; ith < spec.numth; ++ith) {
				GridKey key{ix, iy, ith};
				State3D center = key_to_state(key, spec);
				const int flat = key_to_flat_index(key);

				for (int dx : x_offsets) {
					const int x = center.x + dx;
					if (x < spec.bottom.x || x > spec.top.x) {
						escape_cell[flat] = 1;
						continue;
					}
					for (int dy : y_offsets) {
						const int y = center.y + dy;
						if (y < spec.bottom.y || y > spec.top.y) {
							escape_cell[flat] = 1;
							continue;
						}
						for (int dth : th_offsets) {
							int th = center.th + dth;
							while (th < spec.bottom.th) th += theta_period;
							while (th > spec.top.th) th -= theta_period;

							CellStatus status = fine_state_status(State3D(x, y, th));
							if (status == CellStatus::Free) {
								cell_samples[flat].push_back(State3D(x, y, th));
							}
							else if (status == CellStatus::EscapeEdge) {
								escape_cell[flat] = 1;
							}
						}
					}
				}

				if (!cell_samples[flat].empty()) {
					free_cell[flat] = 1;
				}
			}
		}
	}

	static const int dirs[26][3] = {
		{-1,-1,-1},{0,-1,-1},{1,-1,-1},{-1,0,-1},{0,0,-1},{1,0,-1},{-1,1,-1},{0,1,-1},{1,1,-1},
		{-1,-1,0},{0,-1,0},{1,-1,0},{-1,0,0},{1,0,0},{-1,1,0},{0,1,0},{1,1,0},
		{-1,-1,1},{0,-1,1},{1,-1,1},{-1,0,1},{0,0,1},{1,0,1},{-1,1,1},{0,1,1},{1,1,1}
	};

	struct InitialComponent
	{
		PointCloud seeds;
		int coarse_cells;
	};
	std::vector<InitialComponent> initial_components;

	for (int flat_seed = 0; flat_seed < coarse_grid_size; ++flat_seed) {
		if (visited[flat_seed] || !free_cell[flat_seed]) continue;

		InitialComponent component;
		component.seeds.set_default_cell_size(spec.fine_step);
		component.coarse_cells = 0;
		std::stack<int> stack;
		bool escaped = false;

		visited[flat_seed] = 1;
		stack.push(flat_seed);

		while (!stack.empty() && !escaped) {
			const int current_flat = stack.top();
			stack.pop();
			GridKey current = flat_index_to_key(current_flat);
			State3D current_state = key_to_state(current, spec);
			if (escape_cell[current_flat] || near_escape_edge(current_state, spec, 1)) {
				escaped = true;
				break;
			}

			++component.coarse_cells;
			for (const auto& sample : cell_samples[current_flat]) {
				component.seeds.push(sample, spec.fine_step);
			}

			for (const auto& dir : dirs) {
				GridKey next{current.ix + dir[0], current.iy + dir[1], current.ith + dir[2]};
				next = normalize_theta_key(next, spec);
				if (!valid_xy_key(next, spec)) continue;

				const int next_flat = key_to_flat_index(next);
				if (visited[next_flat] || !free_cell[next_flat]) continue;

				visited[next_flat] = 1;
				stack.push(next_flat);
			}
		}

		if (!escaped && !component.seeds.empty()) {
			initial_components.push_back(component);
		}
	}

	if (read_adaptive_int("initial_refine_clusters", 1) == 0) {
		std::vector<PointCloud> coarse_samples;
		for (const auto& component : initial_components) {
			coarse_samples.push_back(component.seeds);
		}
		return coarse_samples;
	}

	const int max_initial_coarse_component = std::max(1,
		read_adaptive_int("max_initial_coarse_component", 500));

	std::vector<PointCloud> exact_components;
	DfsCFO exact;
	for (const auto& component : initial_components) {
		if (component.coarse_cells > max_initial_coarse_component) continue;

		std::vector<PointCloud> refined = exact.extract(component.seeds, node);
		for (const auto& candidate : refined) {
			bool duplicated = false;
			for (const auto& existing : exact_components) {
				if (existing.overlap(candidate)) {
					duplicated = true;
					break;
				}
			}
			if (!duplicated) {
				exact_components.push_back(candidate);
			}
		}
	}

	sort_initial_components(exact_components);
	return exact_components;
}

CFO* make_cfo_strategy()
{
	if (read_adaptive_int("enabled", 0) != 0
	    && read_adaptive_int("transition_enabled", 0) != 0) {
		return new AdaptiveDfsCFO();
	}
	return new DfsCFO();
}

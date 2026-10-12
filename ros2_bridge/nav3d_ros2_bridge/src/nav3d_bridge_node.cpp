#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <deque>
#include <exception>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <queue>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/srv/get_plan.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#ifdef NAV3D_HAS_OCTOMAP_MSGS
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/msg/octomap.hpp>
#endif

#include "nav3d/collision/footprint_3d.h"
#include "nav3d/collision/inflation_layer.h"
#include "nav3d/collision/trajectory_checker.h"
#include "nav3d/controller/safety_monitor.h"
#include "nav3d/controller/trajectory_tracker.h"
#include "nav3d/common/types.h"
#include "nav3d/map/local_grid.h"
#include "nav3d/map/map_2d_projection.h"
#include "nav3d/map/map_builder.h"
#include "nav3d/map/map_composite.h"
#include "nav3d/map/octomap_manager.h"
#include "nav3d/map/pcd_loader.h"
#include "nav3d/map/voxel_grid_map.h"
#include "nav3d/planner/ego_planner_core.h"
#include "nav3d/planner/path_searching/astar_3d.h"

#ifdef NAV3D_HAS_PCL
#include "pcl_pcd_loader.h"
#endif

namespace {

enum class PcdLoaderBackend {
  Builtin,
  Pcl,
};

enum class PlanningTraversabilityMode {
  Uav,
  Ground,
};

struct GroundIndex {
  int x = 0;
  int y = 0;
  int z = 0;

  bool operator==(const GroundIndex& other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }

  bool operator!=(const GroundIndex& other) const
  {
    return !(*this == other);
  }
};

struct GroundIndexHash {
  std::size_t operator()(const GroundIndex& idx) const
  {
    const std::size_t h1 = std::hash<int>{}(idx.x);
    const std::size_t h2 = std::hash<int>{}(idx.y);
    const std::size_t h3 = std::hash<int>{}(idx.z);
    return h1 ^ (h2 << 1U) ^ (h3 << 2U);
  }
};

struct GroundQueueNode {
  GroundIndex idx;
  double f = 0.0;
  double g = 0.0;
};

struct GroundQueueNodeCompare {
  bool operator()(const GroundQueueNode& a, const GroundQueueNode& b) const
  {
    constexpr double kEpsilon = 1e-12;
    if (std::abs(a.f - b.f) > kEpsilon) {
      return a.f > b.f;
    }
    return a.g < b.g;
  }
};

struct GroundCornerKeepout {
  double x = 0.0;
  double y = 0.0;
  double radius = 0.0;
};

struct GroundSearchOptions {
  nav3d::planner::PlanningMode mode = nav3d::planner::PlanningMode::Mode3D;
  int max_iterations = 200000;
  // Keep snap small: large radii block the planning callback for a long time.
  int snap_radius_cells = 6;
  int support_xy_radius_cells = 1;
  int support_depth_cells = 1;
  double robot_radius = 0.25;
  bool allow_diagonal = true;
  bool strict_direct_ground_support = false;
  bool footprint_enabled = false;
  nav3d::collision::Footprint3DConfig footprint =
    nav3d::collision::makeDefaultDogFootprintConfig();
  // Soft ring around auto-detected / explicit corners. Hard ban (if any) is
  // only for explicit keepout_xyr cylinders — see canMoveGround.
  double corner_keepout_radius = 1.5;
  double corner_keepout_z_min = -1.0;
  double corner_keepout_z_max = 8.0;
  double corner_keepout_height_span_min = 0.5;
  double corner_keepout_penalty = 80.0;
  // Used only when corner_keepouts (explicit xyr) is non-empty.
  double corner_keepout_hard_core_scale = 0.5;
  std::vector<GroundCornerKeepout> corner_keepouts;
  // Extra cost when the path centerline is closer than this to occupied walls
  // (compensates for A* using a sphere while the dog footprint is long/wide).
  double wall_clearance = 0.6;
  double wall_clearance_weight = 30.0;
  // Filled once per A* search — avoids O(radius²·seedTest) per expansion.
  const std::vector<nav3d::common::Point3D>* auto_corner_seeds = nullptr;
  // Multi-floor plans: optional via-points (off / empty by default).
  bool landing_via_enabled = false;
  double landing_via_min_dz = 0.7;
  double landing_via_radius = 0.8;
  std::vector<nav3d::common::Point3D> landing_via_points;
  double climb_penalty = 0.85;
  // Soft cost for leaving the start/goal z band when |goal.z-start.z| is small.
  // Hard z-band caused no_path on building2_9 west wing (L1 islands only join upstairs).
  double leave_floor_penalty = 25.0;
};

struct GroundSearchResult {
  nav3d::planner::SearchStatus status = nav3d::planner::SearchStatus::NoPath;
  nav3d::common::Path3D path;
  int iterations = 0;
  double path_cost = 0.0;
};

struct GroundSnapCandidate {
  GroundIndex idx;
  double distance = std::numeric_limits<double>::infinity();
  double xy_distance = std::numeric_limits<double>::infinity();
};

constexpr int kGroundWarmStartMultiplicity = 3;

nav3d::common::Point3D toPoint(const geometry_msgs::msg::PoseStamped& msg)
{
  return {
    msg.pose.position.x,
    msg.pose.position.y,
    msg.pose.position.z,
  };
}

nav3d::common::Point3D toPoint(const geometry_msgs::msg::PointStamped& msg)
{
  return {
    msg.point.x,
    msg.point.y,
    msg.point.z,
  };
}

std::string toLower(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return value;
}

std::string trim(const std::string& value)
{
  auto begin = value.begin();
  while (begin != value.end() &&
         std::isspace(static_cast<unsigned char>(*begin))) {
    ++begin;
  }
  auto end = value.end();
  while (end != begin &&
         std::isspace(static_cast<unsigned char>(*(end - 1)))) {
    --end;
  }
  return {begin, end};
}

double pathLength(const nav3d::common::Path3D& path)
{
  double length = 0.0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    length += nav3d::common::distance(path[i - 1], path[i]);
  }
  return length;
}

GroundIndex worldToGroundIndex(const nav3d::common::Point3D& point, double resolution)
{
  return {
    static_cast<int>(std::floor(point.x / resolution + 1e-9)),
    static_cast<int>(std::floor(point.y / resolution + 1e-9)),
    static_cast<int>(std::floor(point.z / resolution + 1e-9)),
  };
}

nav3d::common::Point3D groundIndexToWorld(const GroundIndex& idx, double resolution)
{
  return {
    (static_cast<double>(idx.x) + 0.5) * resolution,
    (static_cast<double>(idx.y) + 0.5) * resolution,
    (static_cast<double>(idx.z) + 0.5) * resolution,
  };
}

double groundIndexDistance(const GroundIndex& a, const GroundIndex& b)
{
  const double dx = static_cast<double>(a.x - b.x);
  const double dy = static_cast<double>(a.y - b.y);
  const double dz = static_cast<double>(a.z - b.z);
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

bool hasGroundSupport(
  const nav3d::map::IMap& map,
  const GroundIndex& idx,
  const GroundSearchOptions& options)
{
  const double resolution = map.getResolution();
  if (options.strict_direct_ground_support) {
    const auto below = groundIndexToWorld({idx.x, idx.y, idx.z - 1}, resolution);
    return map.isInBounds(below) && map.isOccupied(below);
  }

  for (int dz = 1; dz <= std::max(1, options.support_depth_cells); ++dz) {
    for (int dx = -options.support_xy_radius_cells; dx <= options.support_xy_radius_cells; ++dx) {
      for (int dy = -options.support_xy_radius_cells; dy <= options.support_xy_radius_cells; ++dy) {
        const auto below = groundIndexToWorld({idx.x + dx, idx.y + dy, idx.z - dz}, resolution);
        if (map.isInBounds(below) && map.isOccupied(below)) {
          return true;
        }
      }
    }
  }
  return false;
}

bool hasBodyClearance(
  const nav3d::map::IMap& map,
  const GroundIndex& idx,
  const GroundSearchOptions& options,
  double yaw = 0.0,
  const std::optional<nav3d::collision::BodyAttitude>& imu_override = std::nullopt)
{
  (void)yaw;
  (void)imu_override;
  // Oriented footprint is enforced on trajectories / tracking. A* cell filter
  // keeps the configured sphere radius so stair connectivity stays intact.
  const double resolution = map.getResolution();
  const int radius_cells =
    std::max(0, static_cast<int>(std::ceil(options.robot_radius / resolution)));
  const double radius_sq = options.robot_radius * options.robot_radius;
  for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
    for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
      for (int dz = 0; dz <= radius_cells; ++dz) {
        const double dist_x = static_cast<double>(dx) * resolution;
        const double dist_y = static_cast<double>(dy) * resolution;
        const double dist_z = static_cast<double>(dz) * resolution;
        if (dist_x * dist_x + dist_y * dist_y + dist_z * dist_z > radius_sq + 1e-12) {
          continue;
        }
        const auto nearby = groundIndexToWorld({idx.x + dx, idx.y + dy, idx.z + dz}, resolution);
        if (map.isInBounds(nearby) && map.isOccupied(nearby)) {
          return false;
        }
      }
    }
  }
  return true;
}

bool columnHasOccupied(
  const nav3d::map::IMap& map,
  int x,
  int y,
  int z_center,
  double resolution,
  int z_pad = 2)
{
  for (int dz = -z_pad; dz <= z_pad; ++dz) {
    const auto p = groundIndexToWorld({x, y, z_center + dz}, resolution);
    if (map.isInBounds(p) && map.isOccupied(p)) {
      return true;
    }
  }
  return false;
}

bool hasNearbySupportHeightSpan(
  const nav3d::map::IMap& map,
  const GroundIndex& idx,
  const GroundSearchOptions& options)
{
  const double resolution = map.getResolution();
  // Cheap stair-context probe: sample a few columns instead of a dense cube.
  const int step = std::max(1, static_cast<int>(std::lround(0.6 / std::max(1e-6, resolution))));
  const int offsets[][2] = {
    {0, 0}, {step, 0}, {-step, 0}, {0, step}, {0, -step},
    {step, step}, {step, -step}, {-step, step}, {-step, -step},
  };
  double z_min = std::numeric_limits<double>::infinity();
  double z_max = -std::numeric_limits<double>::infinity();
  for (const auto& offset : offsets) {
    for (int dz = -step; dz <= step; dz += step) {
      const GroundIndex candidate{idx.x + offset[0], idx.y + offset[1], idx.z + dz};
      if (!hasGroundSupport(map, candidate, options)) {
        continue;
      }
      const double z = groundIndexToWorld(candidate, resolution).z;
      z_min = std::min(z_min, z);
      z_max = std::max(z_max, z);
    }
  }
  if (!std::isfinite(z_min) || !std::isfinite(z_max)) {
    return false;
  }
  return (z_max - z_min) >= options.corner_keepout_height_span_min;
}

bool hasStrictConcaveLPattern(
  const nav3d::map::IMap& map,
  const GroundIndex& idx,
  double resolution,
  int* out_dx = nullptr,
  int* out_dy = nullptr)
{
  const bool n = columnHasOccupied(map, idx.x, idx.y + 1, idx.z, resolution);
  const bool s = columnHasOccupied(map, idx.x, idx.y - 1, idx.z, resolution);
  const bool e = columnHasOccupied(map, idx.x + 1, idx.y, idx.z, resolution);
  const bool w = columnHasOccupied(map, idx.x - 1, idx.y, idx.z, resolution);
  const int occupied_orth = static_cast<int>(n) + static_cast<int>(s) +
    static_cast<int>(e) + static_cast<int>(w);
  if (occupied_orth != 2) {
    return false;
  }
  int dx = 0;
  int dy = 0;
  if (n && e) {
    dx = 1;
    dy = 1;
  } else if (e && s) {
    dx = 1;
    dy = -1;
  } else if (s && w) {
    dx = -1;
    dy = -1;
  } else if (w && n) {
    dx = -1;
    dy = 1;
  } else {
    return false;
  }
  if (!columnHasOccupied(map, idx.x + dx, idx.y + dy, idx.z, resolution) ||
      columnHasOccupied(map, idx.x - dx, idx.y - dy, idx.z, resolution)) {
    return false;
  }
  if (out_dx != nullptr) {
    *out_dx = dx;
  }
  if (out_dy != nullptr) {
    *out_dy = dy;
  }
  return true;
}

bool isInnerWallCornerSeed(
  const nav3d::map::IMap& map,
  const GroundIndex& idx,
  const GroundSearchOptions& options)
{
  // Strict concave corner only. Cheap L-pattern first — height-span is expensive
  // and used to be first, which made every A* step scan the whole neighborhood.
  const double resolution = map.getResolution();
  const auto point = groundIndexToWorld(idx, resolution);
  if (!map.isInBounds(point) || map.isOccupied(point)) {
    return false;
  }
  if (!hasStrictConcaveLPattern(map, idx, resolution)) {
    return false;
  }
  return hasNearbySupportHeightSpan(map, idx, options);
}

std::vector<nav3d::common::Point3D> collectAutoCornerSeeds(
  const nav3d::map::IMap& map,
  const GroundSearchOptions& options)
{
  std::vector<nav3d::common::Point3D> seeds;
  if (options.corner_keepout_radius <= 1e-9 || options.corner_keepout_penalty <= 0.0) {
    return seeds;
  }
  const double resolution = map.getResolution();
  const auto bounds = map.getBounds();
  if (!bounds.valid) {
    return seeds;
  }
  const double z_lo = std::max(bounds.min.z, options.corner_keepout_z_min);
  const double z_hi = std::min(bounds.max.z, options.corner_keepout_z_max);
  if (z_hi < z_lo) {
    return seeds;
  }
  const GroundIndex min_idx = worldToGroundIndex({bounds.min.x, bounds.min.y, z_lo}, resolution);
  const GroundIndex max_idx = worldToGroundIndex({bounds.max.x, bounds.max.y, z_hi}, resolution);
  seeds.reserve(256);
  for (int z = min_idx.z; z <= max_idx.z; ++z) {
    for (int y = min_idx.y; y <= max_idx.y; ++y) {
      for (int x = min_idx.x; x <= max_idx.x; ++x) {
        const GroundIndex idx{x, y, z};
        const auto point = groundIndexToWorld(idx, resolution);
        if (!map.isInBounds(point) || map.isOccupied(point) || !map.isFree(point)) {
          continue;
        }
        if (!hasStrictConcaveLPattern(map, idx, resolution)) {
          continue;
        }
        if (!hasNearbySupportHeightSpan(map, idx, options)) {
          continue;
        }
        seeds.push_back(point);
      }
    }
  }
  return seeds;
}

bool isInExplicitCornerKeepout(
  const nav3d::common::Point3D& point,
  const GroundSearchOptions& options)
{
  if (point.z < options.corner_keepout_z_min || point.z > options.corner_keepout_z_max) {
    return false;
  }
  for (const auto& keepout : options.corner_keepouts) {
    if (keepout.radius <= 1e-9) {
      continue;
    }
    const double dx = point.x - keepout.x;
    const double dy = point.y - keepout.y;
    if ((dx * dx + dy * dy) < keepout.radius * keepout.radius) {
      return true;
    }
  }
  return false;
}

double nearestCornerKeepoutXyDistance(
  const nav3d::map::IMap& map,
  const GroundIndex& idx,
  const GroundSearchOptions& options)
{
  const double resolution = map.getResolution();
  const auto point = groundIndexToWorld(idx, resolution);
  double best = std::numeric_limits<double>::infinity();

  if (point.z >= options.corner_keepout_z_min && point.z <= options.corner_keepout_z_max) {
    for (const auto& keepout : options.corner_keepouts) {
      if (keepout.radius <= 1e-9) {
        continue;
      }
      const double dx = point.x - keepout.x;
      const double dy = point.y - keepout.y;
      best = std::min(best, std::sqrt(dx * dx + dy * dy));
    }
  }

  if (options.corner_keepout_radius <= 1e-9 ||
      point.z < options.corner_keepout_z_min ||
      point.z > options.corner_keepout_z_max) {
    return best;
  }

  // Fast path: precomputed seeds (same-floor band |Δz|≤0.6).
  if (options.auto_corner_seeds != nullptr) {
    for (const auto& seed : *options.auto_corner_seeds) {
      if (std::abs(seed.z - point.z) > 0.6) {
        continue;
      }
      const double dx = point.x - seed.x;
      const double dy = point.y - seed.y;
      best = std::min(best, std::sqrt(dx * dx + dy * dy));
    }
    return best;
  }

  // Slow fallback (should be rare once search precomputes seeds).
  if (!hasNearbySupportHeightSpan(map, idx, options)) {
    return best;
  }
  const int radius_cells = std::max(
    1,
    static_cast<int>(
      std::ceil(options.corner_keepout_radius / std::max(1e-6, resolution))));
  if (isInnerWallCornerSeed(map, idx, options)) {
    return 0.0;
  }
  for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
    for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
      if (dx == 0 && dy == 0) {
        continue;
      }
      const GroundIndex seed{idx.x + dx, idx.y + dy, idx.z};
      if (!isInnerWallCornerSeed(map, seed, options)) {
        continue;
      }
      const auto seed_point = groundIndexToWorld(seed, resolution);
      const double ddx = point.x - seed_point.x;
      const double ddy = point.y - seed_point.y;
      best = std::min(best, std::sqrt(ddx * ddx + ddy * ddy));
    }
  }
  return best;
}

bool isInCornerKeepout(
  const nav3d::map::IMap& map,
  const GroundIndex& idx,
  const GroundSearchOptions& options,
  double radius_scale = 1.0)
{
  const double effective_radius = options.corner_keepout_radius * std::max(0.0, radius_scale);
  const auto point = groundIndexToWorld(idx, map.getResolution());

  // Explicit cylinders: compare against each cylinder radius * scale.
  if (!options.corner_keepouts.empty() &&
      point.z >= options.corner_keepout_z_min &&
      point.z <= options.corner_keepout_z_max) {
    for (const auto& keepout : options.corner_keepouts) {
      if (keepout.radius <= 1e-9) {
        continue;
      }
      const double r = keepout.radius * std::max(0.0, radius_scale);
      const double dx = point.x - keepout.x;
      const double dy = point.y - keepout.y;
      if ((dx * dx + dy * dy) < r * r) {
        return true;
      }
    }
  }

  if (effective_radius <= 1e-9) {
    return false;
  }
  const double dist = nearestCornerKeepoutXyDistance(map, idx, options);
  return std::isfinite(dist) && dist < effective_radius;
}

double nearestOccupiedXyDistance(
  const nav3d::map::IMap& map,
  const GroundIndex& idx,
  double search_radius)
{
  const double resolution = map.getResolution();
  if (search_radius <= 1e-9) {
    return std::numeric_limits<double>::infinity();
  }
  const int radius_cells =
    std::max(1, static_cast<int>(std::ceil(search_radius / std::max(1e-6, resolution))));
  double best_sq = std::numeric_limits<double>::infinity();
  for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
    for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
      if (dx == 0 && dy == 0) {
        continue;
      }
      if (!columnHasOccupied(map, idx.x + dx, idx.y + dy, idx.z, resolution)) {
        continue;
      }
      const double dist_x = static_cast<double>(dx) * resolution;
      const double dist_y = static_cast<double>(dy) * resolution;
      best_sq = std::min(best_sq, dist_x * dist_x + dist_y * dist_y);
    }
  }
  return std::sqrt(best_sq);
}

bool isGroundTraversable(
  const nav3d::map::IMap& map,
  const GroundIndex& idx,
  const GroundSearchOptions& options,
  bool /*enforce_corner_keepout*/ = true)
{
  const auto point = groundIndexToWorld(idx, map.getResolution());
  return map.isFree(point) &&
         hasGroundSupport(map, idx, options) &&
         hasBodyClearance(map, idx, options);
}

GroundSnapCandidate makeGroundSnapCandidate(
  const GroundIndex& requested,
  const GroundIndex& candidate,
  double resolution)
{
  const auto requested_world = groundIndexToWorld(requested, resolution);
  const auto candidate_world = groundIndexToWorld(candidate, resolution);
  const double dx = candidate_world.x - requested_world.x;
  const double dy = candidate_world.y - requested_world.y;
  return {
    candidate,
    nav3d::common::distance(requested_world, candidate_world),
    std::sqrt(dx * dx + dy * dy),
  };
}

bool isBetterGroundSnapCandidate(
  const GroundSnapCandidate& candidate,
  const GroundSnapCandidate& best,
  const GroundIndex& requested)
{
  constexpr double kEpsilon = 1e-12;
  if (candidate.distance < best.distance - kEpsilon) {
    return true;
  }
  if (candidate.distance > best.distance + kEpsilon) {
    return false;
  }
  if (candidate.xy_distance < best.xy_distance - kEpsilon) {
    return true;
  }
  if (candidate.xy_distance > best.xy_distance + kEpsilon) {
    return false;
  }
  // Prefer the tread just above the click (not a higher free cell that can
  // land on a poorly-connected ribbon and make A* crawl for minutes).
  const int cand_above = std::max(0, candidate.idx.z - requested.z);
  const int best_above = std::max(0, best.idx.z - requested.z);
  if (cand_above != best_above) {
    return cand_above < best_above;
  }
  if (candidate.idx.z != best.idx.z) {
    return candidate.idx.z < best.idx.z;
  }
  if (candidate.idx.y != best.idx.y) {
    return candidate.idx.y < best.idx.y;
  }
  return candidate.idx.x < best.idx.x;
}

GroundIndex resolveGroundIndex(
  const nav3d::map::IMap& map,
  const GroundIndex& requested,
  const GroundSearchOptions& options)
{
  // Endpoints may sit near a stair inner corner; keep-out applies to expansion only.
  constexpr bool kEnforceKeepout = false;
  if (isGroundTraversable(map, requested, options, kEnforceKeepout)) {
    return requested;
  }

  std::optional<GroundSnapCandidate> best;
  const int radius = options.snap_radius_cells;
  // Expand by Chebyshev shell; keep the global best across shells so we do not
  // lock onto a high free cell in an early shell that disconnects the graph.
  for (int shell = 1; shell <= radius; ++shell) {
    for (int dx = -shell; dx <= shell; ++dx) {
      for (int dy = -shell; dy <= shell; ++dy) {
        for (int dz = -shell; dz <= shell; ++dz) {
          if (std::max({std::abs(dx), std::abs(dy), std::abs(dz)}) != shell) {
            continue;
          }
          const GroundIndex candidate{
            requested.x + dx,
            requested.y + dy,
            requested.z + dz,
          };
          if (!isGroundTraversable(map, candidate, options, kEnforceKeepout)) {
            continue;
          }
          const auto snap_candidate =
            makeGroundSnapCandidate(requested, candidate, map.getResolution());
          if (!best.has_value() ||
              isBetterGroundSnapCandidate(snap_candidate, *best, requested)) {
            best = snap_candidate;
          }
        }
      }
    }
    // Good enough: same XY column, one cell above the click (typical floor→tread).
    if (best.has_value() &&
        best->idx.x == requested.x &&
        best->idx.y == requested.y &&
        best->idx.z == requested.z + 1) {
      return best->idx;
    }
  }
  return best.has_value() ? best->idx : requested;
}

std::vector<GroundIndex> makeGroundDirections(
  nav3d::planner::PlanningMode mode,
  bool allow_diagonal)
{
  std::vector<GroundIndex> directions;
  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dz = -1; dz <= 1; ++dz) {
        if (dx == 0 && dy == 0 && dz == 0) {
          continue;
        }
        if (mode == nav3d::planner::PlanningMode::Mode2D && dz != 0) {
          continue;
        }
        if (!allow_diagonal && std::abs(dx) + std::abs(dy) + std::abs(dz) != 1) {
          continue;
        }
        directions.push_back({dx, dy, dz});
      }
    }
  }
  return directions;
}

bool isNearGroundEndpoint(
  const GroundIndex& idx,
  const GroundIndex& endpoint,
  double radius_m,
  double resolution)
{
  if (radius_m <= 1e-9) {
    return false;
  }
  const double dx = static_cast<double>(idx.x - endpoint.x) * resolution;
  const double dy = static_cast<double>(idx.y - endpoint.y) * resolution;
  const double dz = static_cast<double>(idx.z - endpoint.z) * resolution;
  // Same-floor escape/entry bubble around snapped start/goal.
  return (dx * dx + dy * dy) < radius_m * radius_m && std::abs(dz) <= 0.6;
}

bool canMoveGround(
  const nav3d::map::IMap& map,
  const GroundIndex& from,
  const GroundIndex& direction,
  const GroundSearchOptions& options,
  const GroundIndex* start_idx = nullptr,
  const GroundIndex* goal_idx = nullptr)
{
  const GroundIndex target{
    from.x + direction.x,
    from.y + direction.y,
    from.z + direction.z,
  };
  // Block stairwell corner cuts: any simultaneous horizontal + vertical step
  // can climb from floor-1 into floor-2 without using the landing. Stairs still
  // connect via pure vertical then pure horizontal (6-connect) with support radius.
  if (direction.z != 0 && (direction.x != 0 || direction.y != 0)) {
    return false;
  }
  if (!isGroundTraversable(map, target, options)) {
    return false;
  }

  // Hard ban applies ONLY to explicit keepout cylinders (keepout_xyr). Auto
  // L-corner detection stays soft-cost only: hard-banning auto seeds seals
  // entire stair landings into islands (no_path at ~20-30 iterations).
  const double resolution = map.getResolution();
  const double endpoint_exempt_radius = std::max(options.corner_keepout_radius, 1.0);
  const auto hard_core_blocked = [&](const GroundIndex& idx) {
    if (options.corner_keepout_hard_core_scale <= 1e-9 || options.corner_keepouts.empty()) {
      return false;
    }
    if (goal_idx != nullptr && idx == *goal_idx) {
      return false;
    }
    if (start_idx != nullptr &&
        isNearGroundEndpoint(idx, *start_idx, endpoint_exempt_radius, resolution)) {
      return false;
    }
    if (goal_idx != nullptr &&
        isNearGroundEndpoint(idx, *goal_idx, endpoint_exempt_radius, resolution)) {
      return false;
    }
    const auto point = groundIndexToWorld(idx, resolution);
    if (point.z < options.corner_keepout_z_min || point.z > options.corner_keepout_z_max) {
      return false;
    }
    const double scale = options.corner_keepout_hard_core_scale;
    for (const auto& keepout : options.corner_keepouts) {
      if (keepout.radius <= 1e-9) {
        continue;
      }
      const double core_r = keepout.radius * scale;
      const double dx = point.x - keepout.x;
      const double dy = point.y - keepout.y;
      if ((dx * dx + dy * dy) < core_r * core_r) {
        return true;
      }
    }
    return false;
  };

  if (hard_core_blocked(target)) {
    return false;
  }

  for (int x_step = 0; x_step <= (direction.x != 0 ? 1 : 0); ++x_step) {
    for (int y_step = 0; y_step <= (direction.y != 0 ? 1 : 0); ++y_step) {
      for (int z_step = 0; z_step <= (direction.z != 0 ? 1 : 0); ++z_step) {
        if (x_step == 0 && y_step == 0 && z_step == 0) {
          continue;
        }
        const GroundIndex intermediate{
          from.x + x_step * direction.x,
          from.y + y_step * direction.y,
          from.z + z_step * direction.z,
        };
        if (!isGroundTraversable(map, intermediate, options)) {
          return false;
        }
        if (hard_core_blocked(intermediate)) {
          return false;
        }
      }
    }
  }

  // Oriented footprint is NOT checked here: the dog box (~1.5m) false-blocks
  // stair connectivity in A* (no_path in a few thousand iterations). Footprint
  // is enforced on the smoothed trajectory + tracking instead.
  return true;
}

double cornerKeepoutSoftExtraCost(
  const nav3d::map::IMap& map,
  const GroundIndex& idx,
  const GroundSearchOptions& options)
{
  if (options.corner_keepout_penalty <= 0.0) {
    return 0.0;
  }
  const double resolution = map.getResolution();
  const auto point = groundIndexToWorld(idx, resolution);
  double extra = 0.0;

  if (point.z >= options.corner_keepout_z_min && point.z <= options.corner_keepout_z_max) {
    for (const auto& keepout : options.corner_keepouts) {
      if (keepout.radius <= 1e-9) {
        continue;
      }
      const double dx = point.x - keepout.x;
      const double dy = point.y - keepout.y;
      const double dist = std::sqrt(dx * dx + dy * dy);
      if (dist < keepout.radius) {
        const double t = 1.0 - dist / keepout.radius;
        extra = std::max(extra, options.corner_keepout_penalty * t * t);
      }
    }
  }

  if (options.corner_keepout_radius > 1e-9) {
    const double dist = nearestCornerKeepoutXyDistance(map, idx, options);
    // Ignore explicit-only distances when auto radius is the falloff scale: if the
    // nearest hit is an explicit cylinder already counted above, auto falloff with
    // corner_keepout_radius still pushes away from L seeds (same query).
    if (std::isfinite(dist) && dist < options.corner_keepout_radius) {
      const double t = 1.0 - dist / options.corner_keepout_radius;
      extra = std::max(extra, options.corner_keepout_penalty * t * t);
      // Near-seed spike: soft falloff alone still left min_seed_xy≈0.2 on planned
      // stair turns (centerline OK for 0.25m sphere, long footprint clips railings).
      // A sharp extra band (~dog half-width) forces outer arcs when an alternate exists.
      constexpr double kNearSeedSpikeRadius = 0.45;
      if (dist < kNearSeedSpikeRadius) {
        const double u = 1.0 - dist / kNearSeedSpikeRadius;
        extra += options.corner_keepout_penalty * 4.0 * u * u;
      }
    }
  }
  return extra;
}

double groundStepCost(
  const nav3d::map::IMap& map,
  const GroundIndex& from,
  const GroundIndex& to,
  const GroundSearchOptions& options)
{
  (void)from;
  const double geometric = groundIndexDistance(from, to);
  const int dz = std::abs(to.z - from.z);
  double cost = geometric + options.climb_penalty * static_cast<double>(dz);
  // Distance-weighted soft keepout: flat penalty inside the whole landing makes
  // inner and outer arcs equally expensive, so A* still hugs the railing. Cost
  // peaks at the L-corner seed and falls to 0 at radius.
  cost += cornerKeepoutSoftExtraCost(map, to, options);
  // Wall proximity: keep centerline away from railings for the wide dog body.
  if (options.wall_clearance_weight > 0.0 && options.wall_clearance > 1e-9) {
    const double dist = nearestOccupiedXyDistance(map, to, options.wall_clearance);
    if (std::isfinite(dist) && dist < options.wall_clearance) {
      const double gap = (options.wall_clearance - dist) / options.wall_clearance;
      cost += options.wall_clearance_weight * gap * gap;
    }
  }
  return cost;
}

std::vector<GroundIndex> reconstructGroundPath(
  const std::unordered_map<GroundIndex, GroundIndex, GroundIndexHash>& came_from,
  GroundIndex current)
{
  std::vector<GroundIndex> path{current};
  while (came_from.find(current) != came_from.end()) {
    current = came_from.at(current);
    path.push_back(current);
  }
  std::reverse(path.begin(), path.end());
  return path;
}

std::vector<nav3d::common::Point3D> indexPathToWarmStart(
  const std::vector<GroundIndex>& index_path,
  double resolution)
{
  nav3d::common::Path3D path;
  path.reserve(index_path.size() * kGroundWarmStartMultiplicity);
  for (const auto& idx : index_path) {
    const auto point = groundIndexToWorld(idx, resolution);
    for (int repeat = 0; repeat < kGroundWarmStartMultiplicity; ++repeat) {
      path.push_back(point);
    }
  }
  return path;
}

GroundSearchResult searchGroundAStarBetween(
  const nav3d::map::IMap& map,
  const GroundIndex& start_idx,
  const GroundIndex& goal_idx,
  const GroundSearchOptions& options,
  double goal_xy_radius = 0.0)
{
  if (!isGroundTraversable(map, start_idx, options, false) ||
      !isGroundTraversable(map, goal_idx, options, false)) {
    return {nav3d::planner::SearchStatus::InvalidInput, {}, 0, 0.0};
  }

  // Precompute strict corner seeds once. Per-expansion neighborhood seed tests
  // previously hung planning for minutes (needed SIGKILL on Ctrl+C).
  GroundSearchOptions search_options = options;
  const auto auto_seeds = collectAutoCornerSeeds(map, options);
  search_options.auto_corner_seeds = &auto_seeds;
  RCLCPP_INFO(
    rclcpp::get_logger("nav3d_bridge_node"),
    "ground_astar corner_seeds=%zu keepout_r=%.2f penalty=%.1f",
    auto_seeds.size(),
    options.corner_keepout_radius,
    options.corner_keepout_penalty);

  const double resolution = map.getResolution();
  const auto goal_point = groundIndexToWorld(goal_idx, resolution);
  const double goal_radius_sq = goal_xy_radius * goal_xy_radius;
  auto reached_goal = [&](const GroundIndex& idx) {
    if (idx == goal_idx) {
      return true;
    }
    if (goal_xy_radius <= 1e-9) {
      return false;
    }
    const auto p = groundIndexToWorld(idx, resolution);
    const double dx = p.x - goal_point.x;
    const double dy = p.y - goal_point.y;
    const double dz = p.z - goal_point.z;
    return (dx * dx + dy * dy) <= goal_radius_sq && std::abs(dz) <= std::max(resolution, 0.35);
  };

  std::priority_queue<GroundQueueNode, std::vector<GroundQueueNode>, GroundQueueNodeCompare> open_set;
  std::unordered_map<GroundIndex, double, GroundIndexHash> g_score;
  std::unordered_map<GroundIndex, GroundIndex, GroundIndexHash> came_from;
  std::unordered_set<GroundIndex, GroundIndexHash> closed_set;

  g_score[start_idx] = 0.0;
  open_set.push({start_idx, groundIndexDistance(start_idx, goal_idx), 0.0});
  const auto directions =
    makeGroundDirections(search_options.mode, search_options.allow_diagonal);
  int iterations = 0;

  // Prefer staying near start/goal z when they look same-floor, but do not hard-cut:
  // on building2_9 the stair landing and west wing are separate L1 islands — only
  // connected upstairs (path z up to ~13.7). A hard band caused no_path with
  // closed≈2950 while start/goal were both traversable.
  const auto start_point = groundIndexToWorld(start_idx, resolution);
  constexpr double kSameFloorDz = 0.6;
  constexpr double kFloorBandMargin = 0.45;
  const bool same_floor = std::abs(goal_point.z - start_point.z) <= kSameFloorDz;
  const double z_band_lo = std::min(start_point.z, goal_point.z) - kFloorBandMargin;
  const double z_band_hi = std::max(start_point.z, goal_point.z) + kFloorBandMargin;
  const double leave_floor_pen =
    same_floor ? std::max(0.0, search_options.leave_floor_penalty) : 0.0;
  if (same_floor && leave_floor_pen > 1e-9) {
    RCLCPP_INFO(
      rclcpp::get_logger("nav3d_bridge_node"),
      "ground_astar same_floor_soft_z=[%.2f,%.2f] leave_floor_penalty=%.1f",
      z_band_lo,
      z_band_hi,
      leave_floor_pen);
  }

  while (!open_set.empty() && iterations < search_options.max_iterations) {
    const auto current = open_set.top();
    open_set.pop();
    ++iterations;

    const auto best_score = g_score.find(current.idx);
    if (best_score != g_score.end() && current.g > best_score->second + 1e-12) {
      continue;
    }
    if (closed_set.find(current.idx) != closed_set.end()) {
      continue;
    }
    closed_set.insert(current.idx);

    if (reached_goal(current.idx)) {
      const auto index_path = reconstructGroundPath(came_from, current.idx);
      int soft_hits = 0;
      int near_seed_hits = 0;
      int explicit_hard_hits = 0;
      int near_wall = 0;
      int leave_floor_cells = 0;
      double min_wall = std::numeric_limits<double>::infinity();
      double min_seed = std::numeric_limits<double>::infinity();
      double soft_cost_sum = 0.0;
      double path_z_min = std::numeric_limits<double>::infinity();
      double path_z_max = -std::numeric_limits<double>::infinity();
      constexpr double kNearSeedReportRadius = 0.45;
      for (const auto& idx : index_path) {
        const auto wp = groundIndexToWorld(idx, resolution);
        path_z_min = std::min(path_z_min, wp.z);
        path_z_max = std::max(path_z_max, wp.z);
        if (same_floor && (wp.z < z_band_lo || wp.z > z_band_hi)) {
          ++leave_floor_cells;
        }
        const double soft_extra = cornerKeepoutSoftExtraCost(map, idx, search_options);
        soft_cost_sum += soft_extra;
        if (soft_extra > 1e-6) {
          ++soft_hits;
        }
        // Explicit hard-core only (auto L stays soft). Old "hard_core=" counted
        // auto seeds at hard_core_scale and looked like a ban that does not exist.
        if (!search_options.corner_keepouts.empty() &&
            search_options.corner_keepout_hard_core_scale > 1e-9 &&
            isInCornerKeepout(
              map, idx, search_options, search_options.corner_keepout_hard_core_scale)) {
          ++explicit_hard_hits;
        }
        const double seed_dist = nearestCornerKeepoutXyDistance(map, idx, search_options);
        if (std::isfinite(seed_dist)) {
          min_seed = std::min(min_seed, seed_dist);
          if (seed_dist < kNearSeedReportRadius) {
            ++near_seed_hits;
          }
        }
        const double wall_dist = nearestOccupiedXyDistance(
          map, idx, std::max(search_options.wall_clearance, 1.0));
        if (std::isfinite(wall_dist)) {
          min_wall = std::min(min_wall, wall_dist);
          if (wall_dist < search_options.wall_clearance) {
            ++near_wall;
          }
        }
      }
      auto path = indexPathToWarmStart(index_path, resolution);
      const double cost = pathLength(path);
      RCLCPP_INFO(
        rclcpp::get_logger("nav3d_bridge_node"),
        "ground_astar path_cells=%zu soft_hits=%d soft_cost_avg=%.2f min_seed_xy=%.2f "
        "near_seed=%d explicit_hard=%d near_wall=%d min_wall_xy=%.2f z=[%.2f,%.2f] "
        "leave_floor=%d cost=%.2f iters=%d",
        index_path.size(),
        soft_hits,
        index_path.empty() ? 0.0 : soft_cost_sum / static_cast<double>(index_path.size()),
        std::isfinite(min_seed) ? min_seed : -1.0,
        near_seed_hits,
        explicit_hard_hits,
        near_wall,
        std::isfinite(min_wall) ? min_wall : -1.0,
        path_z_min,
        path_z_max,
        leave_floor_cells,
        cost,
        iterations);
      if (leave_floor_cells > 0) {
        RCLCPP_WARN(
          rclcpp::get_logger("nav3d_bridge_node"),
          "ground_astar same_floor_endpoints_require_climb leave_floor_cells=%d "
          "z=[%.2f,%.2f] (L1 islands not connected on this map)",
          leave_floor_cells,
          path_z_min,
          path_z_max);
      }
      return {nav3d::planner::SearchStatus::Success, std::move(path), iterations, cost};
    }

    for (const auto& direction : directions) {
      const GroundIndex neighbor{
        current.idx.x + direction.x,
        current.idx.y + direction.y,
        current.idx.z + direction.z,
      };
      if (closed_set.find(neighbor) != closed_set.end() ||
          !canMoveGround(
            map, current.idx, direction, search_options, &start_idx, &goal_idx)) {
        continue;
      }
      double step = groundStepCost(map, current.idx, neighbor, search_options);
      if (leave_floor_pen > 1e-9) {
        const double nz = groundIndexToWorld(neighbor, resolution).z;
        if (nz < z_band_lo) {
          step += leave_floor_pen * (z_band_lo - nz) / std::max(resolution, 1e-6);
        } else if (nz > z_band_hi) {
          step += leave_floor_pen * (nz - z_band_hi) / std::max(resolution, 1e-6);
        }
      }
      const double tentative_g = current.g + step;
      const auto existing = g_score.find(neighbor);
      if (existing == g_score.end() || tentative_g < existing->second) {
        came_from[neighbor] = current.idx;
        g_score[neighbor] = tentative_g;
        open_set.push(
          {neighbor, tentative_g + groundIndexDistance(neighbor, goal_idx), tentative_g});
      }
    }
  }

  const bool start_soft = isInCornerKeepout(map, start_idx, search_options);
  const bool goal_soft = isInCornerKeepout(map, goal_idx, search_options);
  const bool start_ok = isGroundTraversable(map, start_idx, search_options);
  const bool goal_ok = isGroundTraversable(map, goal_idx, search_options);
  RCLCPP_WARN(
    rclcpp::get_logger("nav3d_bridge_node"),
    "ground_astar no_path iters=%d closed=%zu start_trav=%d goal_trav=%d "
    "start_soft_keepout=%d goal_soft_keepout=%d robot_r=%.2f seeds=%zu",
    iterations,
    closed_set.size(),
    start_ok ? 1 : 0,
    goal_ok ? 1 : 0,
    start_soft ? 1 : 0,
    goal_soft ? 1 : 0,
    search_options.robot_radius,
    auto_seeds.size());
  return {
    iterations >= search_options.max_iterations
      ? nav3d::planner::SearchStatus::IterationLimit
      : nav3d::planner::SearchStatus::NoPath,
    {},
    iterations,
    0.0,
  };
}

std::vector<nav3d::common::Point3D> selectLandingViaPoints(
  const nav3d::common::Point3D& start,
  const nav3d::common::Point3D& goal,
  const GroundSearchOptions& options)
{
  if (!options.landing_via_enabled || options.landing_via_points.empty()) {
    return {};
  }
  if (std::abs(goal.z - start.z) < options.landing_via_min_dz) {
    return {};
  }

  // Keep vias whose height sits between start/goal (with small margin), ordered
  // by climb direction so we visit landings bottom→top or top→bottom.
  const double z_lo = std::min(start.z, goal.z) + 0.15;
  const double z_hi = std::max(start.z, goal.z) - 0.15;
  std::vector<nav3d::common::Point3D> selected;
  for (const auto& via : options.landing_via_points) {
    if (via.z < z_lo || via.z > z_hi) {
      continue;
    }
    selected.push_back(via);
  }
  if (selected.empty()) {
    // Fall back to all configured vias if none lie strictly between floors.
    selected = options.landing_via_points;
  }
  const bool climbing = goal.z >= start.z;
  std::sort(
    selected.begin(),
    selected.end(),
    [climbing](const nav3d::common::Point3D& a, const nav3d::common::Point3D& b) {
      return climbing ? (a.z < b.z) : (a.z > b.z);
    });
  return selected;
}

GroundSearchResult searchGroundSupportedPath(
  const nav3d::map::IMap& map,
  const nav3d::common::Point3D& start,
  const nav3d::common::Point3D& goal,
  const GroundSearchOptions& options)
{
  if (map.getResolution() <= 0.0) {
    return {nav3d::planner::SearchStatus::InvalidInput, {}, 0, 0.0};
  }

  const double resolution = map.getResolution();
  const GroundIndex start_idx = resolveGroundIndex(map, worldToGroundIndex(start, resolution), options);
  const GroundIndex goal_idx = resolveGroundIndex(map, worldToGroundIndex(goal, resolution), options);
  if (!isGroundTraversable(map, start_idx, options, false) ||
      !isGroundTraversable(map, goal_idx, options, false)) {
    return {nav3d::planner::SearchStatus::InvalidInput, {}, 0, 0.0};
  }

  const auto vias = selectLandingViaPoints(start, goal, options);
  if (vias.empty()) {
    return searchGroundAStarBetween(map, start_idx, goal_idx, options);
  }

  nav3d::common::Path3D combined;
  int total_iterations = 0;
  double total_cost = 0.0;
  GroundIndex current_idx = start_idx;
  auto append_leg = [&](const GroundSearchResult& leg) {
    total_iterations += leg.iterations;
    total_cost += leg.path_cost;
    if (leg.path.empty()) {
      return;
    }
    if (combined.empty()) {
      combined = leg.path;
      return;
    }
    // Skip duplicated junction waypoint warm-start repeats.
    combined.insert(combined.end(), leg.path.begin() + 1, leg.path.end());
  };

  for (const auto& via : vias) {
    const GroundIndex via_idx = resolveGroundIndex(map, worldToGroundIndex(via, resolution), options);
    // Skip vias that collapse onto the current endpoint.
    if (via_idx == current_idx) {
      continue;
    }
    const auto leg = searchGroundAStarBetween(
      map, current_idx, via_idx, options, options.landing_via_radius);
    if (leg.status != nav3d::planner::SearchStatus::Success) {
      // Via unreachable: fall back to direct search rather than failing hard.
      return searchGroundAStarBetween(map, start_idx, goal_idx, options);
    }
    append_leg(leg);
    current_idx = via_idx;
  }

  if (current_idx != goal_idx) {
    const auto last = searchGroundAStarBetween(map, current_idx, goal_idx, options);
    if (last.status != nav3d::planner::SearchStatus::Success) {
      return searchGroundAStarBetween(map, start_idx, goal_idx, options);
    }
    append_leg(last);
  }

  if (combined.empty()) {
    return searchGroundAStarBetween(map, start_idx, goal_idx, options);
  }
  return {
    nav3d::planner::SearchStatus::Success,
    std::move(combined),
    total_iterations,
    total_cost > 0.0 ? total_cost : pathLength(combined),
  };
}

class GroundTraversabilityMap final : public nav3d::map::IMap {
public:
  GroundTraversabilityMap(
    const nav3d::map::IMap& base,
    GroundSearchOptions options)
    : base_(base),
      options_(std::move(options)),
      resolution_(base.getResolution()),
      bounds_(base.getBounds())
  {
    if (resolution_ <= 0.0) {
      throw std::invalid_argument("GroundTraversabilityMap requires a positive resolution");
    }
  }

  bool isOccupied(const nav3d::common::Point3D& point) const override
  {
    return base_.isOccupied(point);
  }

  bool isFree(const nav3d::common::Point3D& point) const override
  {
    return isCellTraversable(worldToGroundIndex(point, resolution_));
  }

  double getDistance(const nav3d::common::Point3D& point) const override
  {
    return base_.getDistance(point);
  }

  bool isInBounds(const nav3d::common::Point3D& point) const override
  {
    return base_.isInBounds(point);
  }

  double getResolution() const override
  {
    return resolution_;
  }

  nav3d::common::BoundingBox getBounds() const override
  {
    return bounds_;
  }

private:
  bool isInsideIndex(const GroundIndex& idx) const
  {
    return base_.isInBounds(groundIndexToWorld(idx, resolution_));
  }

  bool isBaseOccupiedCell(const GroundIndex& idx) const
  {
    return isInsideIndex(idx) && base_.isOccupied(groundIndexToWorld(idx, resolution_));
  }

  bool hasGroundSupportAt(const GroundIndex& idx) const
  {
    if (options_.strict_direct_ground_support) {
      const GroundIndex below{idx.x, idx.y, idx.z - 1};
      return isInsideIndex(below) && isBaseOccupiedCell(below);
    }

    for (int dz = 1; dz <= std::max(1, options_.support_depth_cells); ++dz) {
      for (int dx = -options_.support_xy_radius_cells; dx <= options_.support_xy_radius_cells; ++dx) {
        for (int dy = -options_.support_xy_radius_cells; dy <= options_.support_xy_radius_cells; ++dy) {
          const GroundIndex below{idx.x + dx, idx.y + dy, idx.z - dz};
          if (isInsideIndex(below) && isBaseOccupiedCell(below)) {
            return true;
          }
        }
      }
    }
    return false;
  }

  bool hasBodyClearanceAt(const GroundIndex& idx) const
  {
    const int radius_cells =
      std::max(0, static_cast<int>(std::ceil(options_.robot_radius / resolution_)));
    const double radius_sq = options_.robot_radius * options_.robot_radius;
    for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
      for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
        for (int dz = 0; dz <= radius_cells; ++dz) {
          const double dist_x = static_cast<double>(dx) * resolution_;
          const double dist_y = static_cast<double>(dy) * resolution_;
          const double dist_z = static_cast<double>(dz) * resolution_;
          if (dist_x * dist_x + dist_y * dist_y + dist_z * dist_z > radius_sq + 1e-12) {
            continue;
          }

          const GroundIndex nearby{idx.x + dx, idx.y + dy, idx.z + dz};
          if (isBaseOccupiedCell(nearby)) {
            return false;
          }
        }
      }
    }
    return true;
  }

  bool isCellTraversable(const GroundIndex& idx) const
  {
    if (!isInsideIndex(idx) || isBaseOccupiedCell(idx)) {
      return false;
    }
    if (!hasGroundSupportAt(idx)) {
      return false;
    }
    // Corner keep-out is an A* soft cost, not a hard free-space block.
    // Hard-blocking here isolates stair landings into no_path islands.
    return hasBodyClearanceAt(idx);
  }

  const nav3d::map::IMap& base_;
  GroundSearchOptions options_;
  double resolution_ = 1.0;
  nav3d::common::BoundingBox bounds_;
};

class GroundSupportedSearcher final : public nav3d::planner::IPathSearcher {
public:
  explicit GroundSupportedSearcher(GroundSearchOptions options)
    : options_(std::move(options))
  {
  }

  nav3d::planner::SearchResult search(
    const nav3d::map::IMap& map,
    const nav3d::common::Point3D& start,
    const nav3d::common::Point3D& goal,
    const nav3d::planner::SearchOptions& search_options) const override
  {
    auto options = options_;
    options.mode = search_options.mode;
    options.max_iterations = search_options.max_iterations;
    options.allow_diagonal = search_options.allow_diagonal;
    const auto ground_result = searchGroundSupportedPath(map, start, goal, options);
    return {
      ground_result.status,
      ground_result.path,
      ground_result.iterations,
      ground_result.path_cost,
    };
  }

private:
  GroundSearchOptions options_;
};

nav3d::planner::PlanningMode parsePlanningMode(const std::string& raw_mode)
{
  const std::string mode = toLower(raw_mode);
  if (mode == "2d") {
    return nav3d::planner::PlanningMode::Mode2D;
  }
  if (mode == "3d") {
    return nav3d::planner::PlanningMode::Mode3D;
  }
  throw std::invalid_argument("planning.mode must be '2d' or '3d'");
}

PlanningTraversabilityMode parsePlanningTraversabilityMode(const std::string& raw_mode)
{
  const std::string mode = toLower(raw_mode);
  if (mode == "uav" || mode == "air" || mode == "free_space" || mode == "free-space") {
    return PlanningTraversabilityMode::Uav;
  }
  if (mode == "ground" || mode == "grounded" || mode == "terrain") {
    return PlanningTraversabilityMode::Ground;
  }
  throw std::invalid_argument("planning.traversability must be 'uav' or 'ground'");
}

const char* planningTraversabilityModeName(PlanningTraversabilityMode mode)
{
  switch (mode) {
    case PlanningTraversabilityMode::Uav:
      return "uav";
    case PlanningTraversabilityMode::Ground:
      return "ground";
  }
  return "unknown";
}

nav3d::planner::SearchAlgorithm parseSearchAlgorithm(const std::string& raw_algorithm)
{
  const std::string algorithm = toLower(raw_algorithm);
  if (algorithm == "astar" || algorithm == "a_star" || algorithm == "a*") {
    return nav3d::planner::SearchAlgorithm::AStar;
  }
  if (algorithm == "jps") {
    return nav3d::planner::SearchAlgorithm::Jps;
  }
  throw std::invalid_argument("planning.search_algorithm must be 'astar' or 'jps'");
}

nav3d::planner::DynamicFeasibilityMode parseDynamicFeasibilityMode(
  const std::string& raw_mode)
{
  const std::string mode = toLower(raw_mode);
  if (mode == "sampled" || mode == "sampling") {
    return nav3d::planner::DynamicFeasibilityMode::Sampled;
  }
  if (mode == "analytic" || mode == "analytic_bounds" || mode == "bounds") {
    return nav3d::planner::DynamicFeasibilityMode::AnalyticBounds;
  }
  if (mode == "analytic_exact" || mode == "exact" || mode == "continuous") {
    return nav3d::planner::DynamicFeasibilityMode::AnalyticExact;
  }
  throw std::invalid_argument(
    "planning.dynamic_feasibility_mode must be 'sampled', 'analytic_bounds', or 'analytic_exact'");
}

PcdLoaderBackend parsePcdLoaderBackend(const std::string& raw_backend)
{
  const std::string backend = toLower(raw_backend);
  if (backend == "builtin") {
    return PcdLoaderBackend::Builtin;
  }
  if (backend == "pcl") {
    return PcdLoaderBackend::Pcl;
  }
  throw std::invalid_argument("map.pcd_loader must be 'builtin' or 'pcl'");
}

const char* pcdLoaderBackendName(PcdLoaderBackend backend)
{
  switch (backend) {
    case PcdLoaderBackend::Builtin:
      return "builtin";
    case PcdLoaderBackend::Pcl:
      return "pcl";
  }
  return "unknown";
}

nav3d::controller::CommandFrame parseCommandFrame(const std::string& raw_frame)
{
  const std::string frame = toLower(raw_frame);
  if (frame == "map") {
    return nav3d::controller::CommandFrame::Map;
  }
  if (frame == "body" || frame == "base" || frame == "base_link") {
    return nav3d::controller::CommandFrame::Body;
  }
  throw std::invalid_argument("controller.command_frame must be 'body' or 'map'");
}

nav3d::controller::MotionModelType parseMotionModelType(const std::string& raw_model)
{
  const std::string model = toLower(raw_model);
  if (model == "differential" || model == "differential_drive" || model == "diff_drive") {
    return nav3d::controller::MotionModelType::DifferentialDrive;
  }
  if (model == "omni" || model == "holonomic") {
    return nav3d::controller::MotionModelType::Omni;
  }
  if (model == "uav" || model == "quadrotor") {
    return nav3d::controller::MotionModelType::Uav;
  }
  if (model == "ackermann" || model == "car") {
    return nav3d::controller::MotionModelType::Ackermann;
  }
  throw std::invalid_argument(
    "controller.motion_model must be 'differential', 'omni', 'uav', or 'ackermann'");
}

std::string statusToString(nav3d::planner::EgoPlanStatus status)
{
  switch (status) {
    case nav3d::planner::EgoPlanStatus::Success:
      return "success";
    case nav3d::planner::EgoPlanStatus::SearchFailed:
      return "search_failed";
    case nav3d::planner::EgoPlanStatus::OptimizationFailed:
      return "optimization_failed";
    case nav3d::planner::EgoPlanStatus::DynamicFeasibilityViolation:
      return "dynamic_feasibility_violation";
    case nav3d::planner::EgoPlanStatus::TrajectoryCollision:
      return "trajectory_collision";
    case nav3d::planner::EgoPlanStatus::EmergencyStop:
      return "emergency_stop";
  }
  return "unknown";
}

std::string searchStatusToString(nav3d::planner::SearchStatus status)
{
  switch (status) {
    case nav3d::planner::SearchStatus::Success:
      return "success";
    case nav3d::planner::SearchStatus::NoPath:
      return "no_path";
    case nav3d::planner::SearchStatus::InvalidInput:
      return "invalid_input";
    case nav3d::planner::SearchStatus::IterationLimit:
      return "iteration_limit";
  }
  return "unknown";
}

double snappedGoalTolerance(double resolution)
{
  if (!std::isfinite(resolution) || resolution <= 0.0) {
    return 1e-6;
  }
  return std::sqrt(3.0) * resolution * 0.5 + 1e-6;
}

bool isPartialPlan(const nav3d::planner::EgoPlanResult& result, double resolution)
{
  return nav3d::common::distance(result.requested_goal, result.planned_goal) >
         snappedGoalTolerance(resolution);
}

void appendPlanEndpointStatus(
  std::ostringstream& status,
  const nav3d::planner::EgoPlanResult& result,
  double resolution)
{
  status << " planned_goal="
         << result.planned_goal.x << ","
         << result.planned_goal.y << ","
         << result.planned_goal.z
         << " requested_goal="
         << result.requested_goal.x << ","
         << result.requested_goal.y << ","
         << result.requested_goal.z
         << " partial=" << (isPartialPlan(result, resolution) ? "true" : "false");
}

void appendPlanFailureDebug(
  std::ostringstream& status,
  const nav3d::planner::EgoPlanResult& result,
  double resolution)
{
  status << " search_status=" << searchStatusToString(result.search.status)
         << " search_waypoints=" << result.search.path.size()
         << " search_iterations=" << result.search.iterations
         << " optimization_success=" << (result.optimization.success ? "true" : "false")
         << " optimization_rebound=" << (result.optimization.used_rebound ? "true" : "false")
         << " rebound_segments=" << result.optimization.rebound_segments;
  appendPlanEndpointStatus(status, result, resolution);
  if (result.collision.first_collision_time.has_value()) {
    status << " collision_time=" << *result.collision.first_collision_time;
  }
  if (result.collision.first_collision_point.has_value()) {
    const auto& point = *result.collision.first_collision_point;
    status << " collision_point="
           << point.x << ","
           << point.y << ","
           << point.z;
  }
}

}  // namespace

class Nav3DBridgeNode final : public rclcpp::Node {
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using NavigateGoalHandle = rclcpp_action::ServerGoalHandle<NavigateToPose>;
  using GetPlan = nav_msgs::srv::GetPlan;

  Nav3DBridgeNode() : Node("nav3d_bridge_node")
  {
    // Planning is synchronous and can take minutes on large ground maps. Put
    // cancel/status on a separate callback group so MultiThreadedExecutor can
    // invalidate an in-flight plan without waiting for it to finish first.
    planning_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    control_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    const std::string clicked_point_topic =
      declare_parameter<std::string>("clicked_point_topic", "/clicked_point");
    publish_sample_dt_ = declare_parameter<double>("trajectory.sample_dt", 0.05);
    if (publish_sample_dt_ <= 0.0) {
      throw std::invalid_argument("trajectory.sample_dt must be positive");
    }
    trajectory_publish_max_poses_ =
      declare_parameter<int>("trajectory.publish_max_poses", 600);
    if (trajectory_publish_max_poses_ < 2) {
      throw std::invalid_argument("trajectory.publish_max_poses must be >= 2");
    }

    auto latched_visualization_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    latched_visualization_qos.transient_local().reliable();

    status_pub_ = create_publisher<std_msgs::msg::String>("/nav3d/status", 10);
    trajectory_pub_ =
      create_publisher<nav_msgs::msg::Path>("/nav3d/trajectory", latched_visualization_qos);
    trajectory_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "/nav3d/trajectory_marker",
      latched_visualization_qos);
    start_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "/nav3d/start_marker",
      latched_visualization_qos);
    goal_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "/nav3d/goal_marker",
      latched_visualization_qos);
    occupied_grid_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/nav3d/occupied_grid",
      latched_visualization_qos);
    planning_occupied_markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/nav3d/planning_occupied_markers",
      latched_visualization_qos);
    footprint_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "/nav3d/footprint_marker",
      latched_visualization_qos);
#ifdef NAV3D_HAS_OCTOMAP_MSGS
    octomap_pub_ = create_publisher<octomap_msgs::msg::Octomap>(
      "/octomap",
      latched_visualization_qos);
#endif
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    pose_set_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/nav3d/pose_set", 10);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    start_ = {
      declare_parameter<double>("start.x", 0.0),
      declare_parameter<double>("start.y", 0.0),
      declare_parameter<double>("start.z", 0.0),
    };
    occupancy_grid_max_cells_ =
      declare_parameter<int>("map.occupancy_grid_max_cells", occupancy_grid_max_cells_);
    occupancy_grid_min_z_ =
      declare_parameter<double>("map.occupancy_grid_min_z", occupancy_grid_min_z_);
    occupancy_grid_max_z_ =
      declare_parameter<double>("map.occupancy_grid_max_z", occupancy_grid_max_z_);
    if (!std::isfinite(occupancy_grid_min_z_) || !std::isfinite(occupancy_grid_max_z_) ||
        occupancy_grid_max_z_ < occupancy_grid_min_z_) {
      throw std::invalid_argument(
        "map.occupancy_grid_min_z/max_z must be finite and max_z must be >= min_z");
    }

    map_build_config_ = readMapBuildConfig();
    map_ = std::make_unique<nav3d::map::VoxelGridMap>(buildMap(map_build_config_));
    octomap_ = std::make_unique<nav3d::map::OctomapManager>(buildOctomap(map_build_config_));
    local_grid_ray_clearing_enabled_ =
      declare_parameter<bool>("local_grid.ray_clearing.enabled", true);
    local_grid_enabled_ = declare_parameter<bool>("local_grid.enabled", true);
    local_grid_resolution_ =
      declare_parameter<double>("local_grid.resolution", map_->getResolution());
    if (local_grid_resolution_ <= 0.0) {
      throw std::invalid_argument("local_grid.resolution must be positive");
    }
    local_grid_retention_sec_ =
      declare_parameter<double>("local_grid.retention_sec", local_grid_retention_sec_);
    local_grid_max_retained_frames_ =
      declare_parameter<int>("local_grid.max_retained_frames", local_grid_max_retained_frames_);
    local_grid_max_observation_age_sec_ =
      declare_parameter<double>("local_grid.max_observation_age_sec", local_grid_max_observation_age_sec_);
    local_grid_max_future_stamp_sec_ =
      declare_parameter<double>("local_grid.max_future_stamp_sec", local_grid_max_future_stamp_sec_);
    if (local_grid_retention_sec_ < 0.0) {
      throw std::invalid_argument("local_grid.retention_sec must be non-negative");
    }
    if (local_grid_max_retained_frames_ <= 0) {
      throw std::invalid_argument("local_grid.max_retained_frames must be positive");
    }
    if (local_grid_max_observation_age_sec_ < 0.0) {
      throw std::invalid_argument("local_grid.max_observation_age_sec must be non-negative");
    }
    if (local_grid_max_future_stamp_sec_ < 0.0) {
      throw std::invalid_argument("local_grid.max_future_stamp_sec must be non-negative");
    }
    if (local_grid_enabled_) {
      local_grid_ = makeLocalGrid();
    }
    auto planner_config = makePlannerConfig();
    if (groundTraversabilityEnabled()) {
      planner_ = std::make_unique<nav3d::planner::EgoPlannerCore>(
        planner_config,
        std::make_shared<GroundSupportedSearcher>(ground_search_options_));
    } else {
      planner_ = std::make_unique<nav3d::planner::EgoPlannerCore>(planner_config);
    }
    publishStaticMapVisualization();
    if (declare_parameter<bool>("safety.enabled", true)) {
      safety_monitor_ = makeSafetyMonitor();
    }
    configureController();
    if (controller_enabled_) {
      controller_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(controller_control_dt_)),
        [this]() {
          onControllerTimer();
        });
    }
    startStaticVisualizationRepublish();

    if (local_grid_) {
      local_pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "/nav3d/local_pointcloud",
        1,
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
          onLocalPointCloud(*msg);
        });
    }
    map_load_sub_ = create_subscription<std_msgs::msg::String>(
      "/nav3d/load_pcd_path",
      10,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        onLoadPcdPath(*msg);
      });
    rclcpp::SubscriptionOptions planning_sub_options;
    planning_sub_options.callback_group = planning_callback_group_;
    rclcpp::SubscriptionOptions control_sub_options;
    control_sub_options.callback_group = control_callback_group_;

    start_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/nav3d/start",
      10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        onStart(*msg);
      },
      planning_sub_options);
    // RViz "2D Pose Estimate" publishes PoseWithCovarianceStamped on /initialpose.
    initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose",
      10,
      [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        onInitialPose(*msg);
      },
      planning_sub_options);
    if (safety_monitor_ || controller_enabled_) {
      current_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/nav3d/current_pose",
        10,
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
          onCurrentPose(*msg);
        },
        control_sub_options);
    }
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/nav3d/goal",
      10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        onGoal(*msg);
      },
      planning_sub_options);
    cancel_sub_ = create_subscription<std_msgs::msg::Empty>(
      "/nav3d/cancel",
      10,
      [this](const std_msgs::msg::Empty::SharedPtr) {
        onCancelPlan();
      },
      control_sub_options);
    clicked_point_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      clicked_point_topic,
      10,
      [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        onClickedPoint(*msg);
      },
      planning_sub_options);
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_,
      50,
      [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
        onImu(*msg);
      },
      control_sub_options);
    plan_service_ = create_service<GetPlan>(
      "/nav3d/plan_path",
      [this](
        const std::shared_ptr<GetPlan::Request> request,
        std::shared_ptr<GetPlan::Response> response) {
        onPlanPath(request, response);
      });
    navigate_action_server_ = rclcpp_action::create_server<NavigateToPose>(
      this,
      "/nav3d/navigate_to_pose",
      [this](
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const NavigateToPose::Goal> goal) {
        return handleNavigateGoal(uuid, std::move(goal));
      },
      [this](const std::shared_ptr<NavigateGoalHandle> goal_handle) {
        return handleNavigateCancel(goal_handle);
      },
      [this](const std::shared_ptr<NavigateGoalHandle> goal_handle) {
        handleNavigateAccepted(goal_handle);
      });

    RCLCPP_INFO(
      get_logger(),
      "nav3d bridge ready: frame_id=%s start=(%.3f, %.3f, %.3f) traversability=%s planning_map=%s",
      frame_id_.c_str(),
      start_.x,
      start_.y,
      start_.z,
      planningTraversabilityModeName(planning_traversability_mode_),
      planningMapBackendName());
  }

private:
  nav3d::map::MapBuildConfig readMapBuildConfig()
  {
    nav3d::map::MapBuildConfig config;
    config.pcd_path = declare_parameter<std::string>("map.pcd_path", "");
    pcd_loader_backend_ =
      parsePcdLoaderBackend(declare_parameter<std::string>("map.pcd_loader", "builtin"));
    config.preprocessor.resolution = declare_parameter<double>("map.resolution", 0.5);
    config.preprocessor.min_points_per_voxel =
      declare_parameter<int>("map.min_points_per_voxel", 1);
    config.preprocessor.min_cluster_voxels =
      declare_parameter<int>("map.min_cluster_voxels", 1);
    config.insert_free_space_rays =
      declare_parameter<bool>("map.insert_free_space_rays", false);
    if (config.insert_free_space_rays) {
      config.sensor_origin = nav3d::common::Point3D{
        declare_parameter<double>("map.sensor_origin.x", start_.x),
        declare_parameter<double>("map.sensor_origin.y", start_.y),
        declare_parameter<double>("map.sensor_origin.z", start_.z),
      };
    }

    validateMapBuildConfig(config);
    validatePcdLoaderBackend();
    return config;
  }

  void validateMapBuildConfig(const nav3d::map::MapBuildConfig& config) const
  {
    if (config.pcd_path.empty()) {
      throw std::invalid_argument("map.pcd_path parameter is required");
    }
    if (config.preprocessor.resolution <= 0.0) {
      throw std::invalid_argument("map.resolution must be positive");
    }
    if (config.preprocessor.min_points_per_voxel <= 0) {
      throw std::invalid_argument("map.min_points_per_voxel must be positive");
    }
    if (config.preprocessor.min_cluster_voxels <= 0) {
      throw std::invalid_argument("map.min_cluster_voxels must be positive");
    }
    if (config.insert_free_space_rays && !config.sensor_origin.has_value()) {
      throw std::invalid_argument("map.sensor_origin is required when map.insert_free_space_rays is true");
    }
  }

  void validatePcdLoaderBackend() const
  {
#ifndef NAV3D_HAS_PCL
    if (pcd_loader_backend_ == PcdLoaderBackend::Pcl) {
      throw std::invalid_argument("map.pcd_loader=pcl requested but bridge was built without PCL");
    }
#endif
  }

  nav3d::common::Result<nav3d::map::PointCloud> loadPointCloud(
    const nav3d::map::MapBuildConfig& config) const
  {
    if (pcd_loader_backend_ == PcdLoaderBackend::Builtin) {
      return nav3d::map::PcdLoader::load(config.pcd_path);
    }

#ifdef NAV3D_HAS_PCL
    return nav3d::tools::PclPcdLoader::load(config.pcd_path);
#else
    (void)config;
    return nav3d::common::Result<nav3d::map::PointCloud>::failure(
      "map.pcd_loader=pcl requested but bridge was built without PCL");
#endif
  }

  nav3d::map::VoxelGridMap buildMap(const nav3d::map::MapBuildConfig& config)
  {
    validateMapBuildConfig(config);
    validatePcdLoaderBackend();
    auto loaded = loadPointCloud(config);
    if (!loaded.ok()) {
      throw std::runtime_error("failed to load point cloud: " + loaded.error());
    }
    auto result = nav3d::map::MapBuilder::buildVoxelMapFromPointCloud(loaded.value(), config);

    std::ostringstream status;
    status << "map_loaded pcd_loader=" << pcdLoaderBackendName(pcd_loader_backend_)
           << " raw_points=" << result.raw_point_count
           << " filtered_points=" << result.filtered_point_count
           << " occupied_voxels=" << result.occupied_voxel_count
           << " resolution=" << config.preprocessor.resolution;
    publishStatus(status.str());
    RCLCPP_INFO(get_logger(), "%s", status.str().c_str());
    return std::move(result.map);
  }

  nav3d::map::OctomapManager buildOctomap(const nav3d::map::MapBuildConfig& config)
  {
    validateMapBuildConfig(config);
    validatePcdLoaderBackend();
    auto loaded = loadPointCloud(config);
    if (!loaded.ok()) {
      throw std::runtime_error("failed to load point cloud for octomap: " + loaded.error());
    }
    auto result = nav3d::map::OctomapManager::buildFromPointCloud(loaded.value(), config);
    if (!result.ok()) {
      throw std::runtime_error("failed to build octomap: " + result.error());
    }

    std::ostringstream status;
    status << "octomap_loaded pcd_loader=" << pcdLoaderBackendName(pcd_loader_backend_)
           << " raw_points=" << result.value().raw_point_count
           << " filtered_points=" << result.value().filtered_point_count
           << " occupied_leafs=" << result.value().occupied_leaf_count
           << " resolution=" << config.preprocessor.resolution;
    publishStatus(status.str());
    RCLCPP_INFO(get_logger(), "%s", status.str().c_str());
    return std::move(result.value().map);
  }

  std::unique_ptr<nav3d::map::LocalGrid> makeLocalGrid()
  {
    return makeLocalGridForMap(*map_);
  }

  std::unique_ptr<nav3d::map::LocalGrid> makeLocalGridForMap(
    const nav3d::map::VoxelGridMap& map) const
  {
    const auto bounds = map.getBounds();
    if (!bounds.valid) {
      throw std::runtime_error("local grid requires a valid global map bounds");
    }

    return std::make_unique<nav3d::map::LocalGrid>(
      local_grid_resolution_,
      bounds.min,
      bounds.max);
  }

  std::unique_ptr<nav3d::controller::SafetyMonitor> makeSafetyMonitor()
  {
    nav3d::controller::SafetyMonitorConfig config;
    config.trajectory_sample_step_seconds =
      declare_parameter<double>("safety.trajectory_sample_dt", publish_sample_dt_);
    config.emergency_stop_time_horizon =
      declare_parameter<double>("safety.emergency_stop_horizon", config.emergency_stop_time_horizon);
    config.lookahead_time_horizon =
      declare_parameter<double>("safety.lookahead_horizon", config.lookahead_time_horizon);
    safety_replan_cooldown_sec_ =
      declare_parameter<double>("safety.replan_cooldown_sec", safety_replan_cooldown_sec_);
    safety_local_replan_lookahead_distance_ = declare_parameter<double>(
      "safety.local_replan_lookahead_distance", safety_local_replan_lookahead_distance_);
    safety_local_replan_enabled_ = declare_parameter<bool>(
      "safety.local_replan_enabled", safety_local_replan_enabled_);
    safety_local_use_raw_astar_ = declare_parameter<bool>(
      "safety.local_use_raw_astar", safety_local_use_raw_astar_);
    safety_local_astar_max_iterations_ = declare_parameter<int>(
      "safety.local_astar_max_iterations", safety_local_astar_max_iterations_);
    return std::make_unique<nav3d::controller::SafetyMonitor>(config);
  }

  void configureController()
  {
    controller_enabled_ = declare_parameter<bool>("controller.enabled", true);
    // Ground stair paths place free cells above each tread; B-spline chords
    // between them routinely clip occupied riser voxels. Blocking those
    // commands freezes the robot at the stair mouth, so default the guard
    // off for ground mode (UAV keeps the wall puncture protection).
    controller_block_occupied_voxels_ = declare_parameter<bool>(
      "controller.block_occupied_voxels",
      !groundTraversabilityEnabled());
    if (occupancy_grid_max_cells_ <= 0) {
      throw std::invalid_argument("map.occupancy_grid_max_cells must be positive");
    }

    nav3d::controller::TrajectoryTrackerConfig config;
    config.sample_step_seconds = publish_sample_dt_;
    config.control_dt =
      declare_parameter<double>("controller.control_dt", config.control_dt);
    config.lookahead_time =
      declare_parameter<double>("controller.lookahead_time", config.lookahead_time);
    config.max_linear_speed =
      declare_parameter<double>("controller.max_speed", config.max_linear_speed);
    config.goal_tolerance =
      declare_parameter<double>("controller.goal_tolerance", config.goal_tolerance);
    controller_goal_tolerance_ = config.goal_tolerance;
    config.yaw_gain =
      declare_parameter<double>("controller.yaw_gain", config.yaw_gain);
    config.max_yaw_rate =
      declare_parameter<double>("controller.max_yaw_rate", config.max_yaw_rate);
    config.command_path_sample_step =
      declare_parameter<double>(
        "controller.command_path_sample_step",
        config.command_path_sample_step);
    config.command_frame =
      parseCommandFrame(declare_parameter<std::string>("controller.command_frame", "body"));
    const std::string motion_model_name =
      declare_parameter<std::string>("controller.motion_model", "omni");
    config.motion_model.type = parseMotionModelType(motion_model_name);
    // Ground stair paths require vertical velocity. Omni/diff/ackermann zero vz
    // in MotionModel::constrain, which makes the robot skate under stairwells.
    if (groundTraversabilityEnabled() &&
        config.motion_model.type != nav3d::controller::MotionModelType::Uav) {
      RCLCPP_WARN(
        get_logger(),
        "controller.motion_model=%s zeros linear.z; forcing uav for ground traversability",
        motion_model_name.c_str());
      config.motion_model.type = nav3d::controller::MotionModelType::Uav;
    }
    // Stair wells are narrow: cap speed/lookahead so the tracker cannot cut
    // corners through risers or drift out of the flight.
    if (groundTraversabilityEnabled()) {
      config.max_linear_speed = std::min(config.max_linear_speed, 0.55);
      config.lookahead_time = std::min(config.lookahead_time, 0.25);
      RCLCPP_INFO(
        get_logger(),
        "ground controller caps: max_speed=%.2f lookahead=%.2f",
        config.max_linear_speed,
        config.lookahead_time);
    }
    config.motion_model.max_linear_acceleration =
      declare_parameter<double>(
        "controller.max_linear_acceleration",
        config.motion_model.max_linear_acceleration);
    config.motion_model.min_turning_radius =
      declare_parameter<double>(
        "controller.min_turning_radius",
        config.motion_model.min_turning_radius);
    config.motion_model.max_yaw_acceleration =
      declare_parameter<double>(
        "controller.max_yaw_acceleration",
        config.motion_model.max_yaw_acceleration);
    trajectory_tracker_ =
      std::make_unique<nav3d::controller::TrajectoryTracker>(config);
    controller_control_dt_ = trajectory_tracker_->config().control_dt;
  }

  nav3d::planner::EgoPlannerCoreConfig makePlannerConfig()
  {
    nav3d::planner::EgoPlannerCoreConfig config;
    config.search.algorithm =
      parseSearchAlgorithm(declare_parameter<std::string>("planning.search_algorithm", "astar"));
    config.search.mode =
      parsePlanningMode(declare_parameter<std::string>("planning.mode", "3d"));
    config.search.allow_diagonal =
      declare_parameter<bool>("planning.allow_diagonal", true);
    config.search.max_iterations =
      declare_parameter<int>("planning.max_iterations", config.search.max_iterations);
    ground_search_options_.mode = config.search.mode;
    ground_search_options_.max_iterations = config.search.max_iterations;
    planning_traversability_mode_ =
      parsePlanningTraversabilityMode(
        declare_parameter<std::string>("planning.traversability", "uav"));
    planning_use_octomap_map_ =
      declare_parameter<bool>("planning.use_octomap_map", planning_use_octomap_map_);
    ground_search_options_.snap_radius_cells =
      declare_parameter<int>("planning.ground_snap_radius_cells", ground_search_options_.snap_radius_cells);
    ground_search_options_.support_xy_radius_cells =
      declare_parameter<int>(
        "planning.ground_support_xy_radius_cells",
        ground_search_options_.support_xy_radius_cells);
    ground_search_options_.support_depth_cells =
      declare_parameter<int>(
        "planning.ground_support_depth_cells",
        ground_search_options_.support_depth_cells);
    ground_search_options_.robot_radius =
      declare_parameter<double>("planning.ground_robot_radius", ground_search_options_.robot_radius);
    ground_search_options_.strict_direct_ground_support =
      declare_parameter<bool>(
        "planning.ground_strict_direct_support",
        ground_search_options_.strict_direct_ground_support);
    ground_search_options_.footprint_enabled =
      declare_parameter<bool>("planning.ground_footprint_enabled", false);
    ground_search_options_.footprint.body_clearance =
      declare_parameter<double>(
        "planning.ground_body_clearance",
        ground_search_options_.footprint.body_clearance);
    ground_search_options_.footprint.body_height =
      declare_parameter<double>(
        "planning.ground_body_height",
        ground_search_options_.footprint.body_height);
    ground_search_options_.footprint.max_pitch_rad =
      declare_parameter<double>(
        "planning.ground_max_pitch_rad",
        ground_search_options_.footprint.max_pitch_rad);
    ground_search_options_.footprint.terrain_ignore_margin =
      declare_parameter<double>(
        "planning.ground_terrain_ignore_margin",
        ground_search_options_.footprint.terrain_ignore_margin);
    ground_search_options_.footprint.sample_step =
      declare_parameter<double>(
        "planning.ground_footprint_sample_step",
        ground_search_options_.footprint.sample_step);
    ground_search_options_.corner_keepout_radius =
      declare_parameter<double>(
        "planning.ground_corner_keepout_radius",
        ground_search_options_.corner_keepout_radius);
    ground_search_options_.corner_keepout_z_min =
      declare_parameter<double>(
        "planning.ground_corner_keepout_z_min",
        ground_search_options_.corner_keepout_z_min);
    ground_search_options_.corner_keepout_z_max =
      declare_parameter<double>(
        "planning.ground_corner_keepout_z_max",
        ground_search_options_.corner_keepout_z_max);
    ground_search_options_.corner_keepout_height_span_min =
      declare_parameter<double>(
        "planning.ground_corner_keepout_height_span_min",
        ground_search_options_.corner_keepout_height_span_min);
    ground_search_options_.corner_keepout_penalty =
      declare_parameter<double>(
        "planning.ground_corner_keepout_penalty",
        ground_search_options_.corner_keepout_penalty);
    ground_search_options_.corner_keepout_hard_core_scale =
      declare_parameter<double>(
        "planning.ground_corner_keepout_hard_core_scale",
        ground_search_options_.corner_keepout_hard_core_scale);
    ground_search_options_.wall_clearance =
      declare_parameter<double>(
        "planning.ground_wall_clearance",
        ground_search_options_.wall_clearance);
    ground_search_options_.wall_clearance_weight =
      declare_parameter<double>(
        "planning.ground_wall_clearance_weight",
        ground_search_options_.wall_clearance_weight);
    ground_search_options_.climb_penalty =
      declare_parameter<double>(
        "planning.ground_climb_penalty",
        ground_search_options_.climb_penalty);
    ground_search_options_.leave_floor_penalty =
      declare_parameter<double>(
        "planning.ground_leave_floor_penalty",
        ground_search_options_.leave_floor_penalty);
    ground_search_options_.landing_via_enabled =
      declare_parameter<bool>(
        "planning.ground_landing_via_enabled",
        ground_search_options_.landing_via_enabled);
    ground_search_options_.landing_via_min_dz =
      declare_parameter<double>(
        "planning.ground_landing_via_min_dz",
        ground_search_options_.landing_via_min_dz);
    ground_search_options_.landing_via_radius =
      declare_parameter<double>(
        "planning.ground_landing_via_radius",
        ground_search_options_.landing_via_radius);
    {
      // ROS2 cannot load empty YAML arrays (`key: []`) — omit the key instead.
      std::vector<double> keepout_xyr;
      if (has_parameter("planning.ground_corner_keepout_xyr")) {
        keepout_xyr = get_parameter("planning.ground_corner_keepout_xyr").as_double_array();
      } else {
        keepout_xyr = declare_parameter<std::vector<double>>(
          "planning.ground_corner_keepout_xyr", std::vector<double>{});
      }
      if ((keepout_xyr.size() % 3) != 0) {
        throw std::invalid_argument(
          "planning.ground_corner_keepout_xyr must contain interleaved x,y,radius triples");
      }
      ground_search_options_.corner_keepouts.clear();
      ground_search_options_.corner_keepouts.reserve(keepout_xyr.size() / 3);
      for (std::size_t i = 0; i + 2 < keepout_xyr.size(); i += 3) {
        if (!(keepout_xyr[i + 2] > 0.0) || !std::isfinite(keepout_xyr[i + 2])) {
          throw std::invalid_argument(
            "planning.ground_corner_keepout_xyr radius must be positive and finite");
        }
        ground_search_options_.corner_keepouts.push_back(
          {keepout_xyr[i], keepout_xyr[i + 1], keepout_xyr[i + 2]});
      }
    }
    {
      // Optional; omit from yaml when unused (ROS2 rejects empty [] arrays).
      std::vector<double> via_xyz;
      if (has_parameter("planning.ground_landing_via_points")) {
        via_xyz = get_parameter("planning.ground_landing_via_points").as_double_array();
      } else {
        via_xyz = declare_parameter<std::vector<double>>(
          "planning.ground_landing_via_points", std::vector<double>{});
      }
      if ((via_xyz.size() % 3) != 0) {
        throw std::invalid_argument(
          "planning.ground_landing_via_points must contain interleaved x,y,z triples");
      }
      ground_search_options_.landing_via_points.clear();
      ground_search_options_.landing_via_points.reserve(via_xyz.size() / 3);
      for (std::size_t i = 0; i + 2 < via_xyz.size(); i += 3) {
        ground_search_options_.landing_via_points.push_back(
          {via_xyz[i], via_xyz[i + 1], via_xyz[i + 2]});
      }
    }
    {
      const auto default_xy = std::vector<double>{
        -0.67, -0.4, -0.67, 0.4, 0.87, 0.4, 0.87, -0.4};
      const auto footprint_xy =
        declare_parameter<std::vector<double>>("planning.ground_footprint_xy", default_xy);
      if (footprint_xy.size() < 6 || (footprint_xy.size() % 2) != 0) {
        throw std::invalid_argument(
          "planning.ground_footprint_xy must contain interleaved x,y pairs (>= 3 vertices)");
      }
      ground_search_options_.footprint.polygon.clear();
      ground_search_options_.footprint.polygon.reserve(footprint_xy.size() / 2);
      for (std::size_t i = 0; i + 1 < footprint_xy.size(); i += 2) {
        ground_search_options_.footprint.polygon.push_back(
          {footprint_xy[i], footprint_xy[i + 1]});
      }
      nav3d::collision::validateFootprint3DConfig(ground_search_options_.footprint);
    }
    imu_topic_ = declare_parameter<std::string>("imu.topic", "/imu");
    imu_use_for_body_attitude_ =
      declare_parameter<bool>("imu.use_for_body_attitude", false);
    uav_endpoint_snap_radius_cells_ =
      declare_parameter<int>(
        "planning.uav_endpoint_snap_radius_cells",
        uav_endpoint_snap_radius_cells_);
    config.trajectory_sample_step_seconds = publish_sample_dt_;
    config.optimizer.interval =
      declare_parameter<double>("optimizer.interval", config.optimizer.interval);
    config.max_fallback_attempts =
      declare_parameter<int>("fallback.max_attempts", config.max_fallback_attempts);
    config.enable_dynamic_feasibility_check =
      declare_parameter<bool>(
        "planning.enable_dynamic_feasibility_check",
        config.enable_dynamic_feasibility_check);
    config.dynamic_feasibility_mode =
      parseDynamicFeasibilityMode(
        declare_parameter<std::string>(
          "planning.dynamic_feasibility_mode",
          "sampled"));
    config.dynamic_limits.max_velocity =
      declare_parameter<double>(
        "planning.max_velocity",
        config.dynamic_limits.max_velocity);
    config.dynamic_limits.max_acceleration =
      declare_parameter<double>(
        "planning.max_acceleration",
        config.dynamic_limits.max_acceleration);
    config.feasibility_sample_step_seconds =
      declare_parameter<double>(
        "planning.feasibility_sample_dt",
        config.feasibility_sample_step_seconds);
    config.max_dynamic_time_scale =
      declare_parameter<double>(
        "planning.max_dynamic_time_scale",
        config.max_dynamic_time_scale);
    config.enable_initial_time_allocation =
      declare_parameter<bool>(
        "planning.enable_initial_time_allocation",
        config.enable_initial_time_allocation);
    config.min_time_allocation_interval =
      declare_parameter<double>(
        "planning.min_time_allocation_interval",
        config.min_time_allocation_interval);
    planning_inflation_radius_ =
      declare_parameter<double>("planning.inflation_radius", planning_inflation_radius_);

    if (config.search.max_iterations <= 0) {
      throw std::invalid_argument("planning.max_iterations must be positive");
    }
    if (ground_search_options_.snap_radius_cells <= 0) {
      throw std::invalid_argument("planning.ground_snap_radius_cells must be positive");
    }
    if (ground_search_options_.support_xy_radius_cells < 0) {
      throw std::invalid_argument("planning.ground_support_xy_radius_cells must be non-negative");
    }
    if (ground_search_options_.support_depth_cells <= 0) {
      throw std::invalid_argument("planning.ground_support_depth_cells must be positive");
    }
    if (!std::isfinite(ground_search_options_.robot_radius) ||
        ground_search_options_.robot_radius < 0.0) {
      throw std::invalid_argument("planning.ground_robot_radius must be non-negative and finite");
    }
    if (!std::isfinite(ground_search_options_.corner_keepout_radius) ||
        ground_search_options_.corner_keepout_radius < 0.0) {
      throw std::invalid_argument(
        "planning.ground_corner_keepout_radius must be non-negative and finite");
    }
    if (!std::isfinite(ground_search_options_.corner_keepout_penalty) ||
        ground_search_options_.corner_keepout_penalty < 0.0) {
      throw std::invalid_argument(
        "planning.ground_corner_keepout_penalty must be non-negative and finite");
    }
    if (!std::isfinite(ground_search_options_.corner_keepout_hard_core_scale) ||
        ground_search_options_.corner_keepout_hard_core_scale < 0.0 ||
        ground_search_options_.corner_keepout_hard_core_scale > 1.0) {
      throw std::invalid_argument(
        "planning.ground_corner_keepout_hard_core_scale must be in [0, 1]");
    }
    if (!std::isfinite(ground_search_options_.wall_clearance) ||
        ground_search_options_.wall_clearance < 0.0) {
      throw std::invalid_argument(
        "planning.ground_wall_clearance must be non-negative and finite");
    }
    if (!std::isfinite(ground_search_options_.wall_clearance_weight) ||
        ground_search_options_.wall_clearance_weight < 0.0) {
      throw std::invalid_argument(
        "planning.ground_wall_clearance_weight must be non-negative and finite");
    }
    if (!std::isfinite(ground_search_options_.climb_penalty) ||
        ground_search_options_.climb_penalty < 0.0) {
      throw std::invalid_argument("planning.ground_climb_penalty must be non-negative and finite");
    }
    if (!std::isfinite(ground_search_options_.leave_floor_penalty) ||
        ground_search_options_.leave_floor_penalty < 0.0) {
      throw std::invalid_argument(
        "planning.ground_leave_floor_penalty must be non-negative and finite");
    }
    if (!std::isfinite(ground_search_options_.landing_via_min_dz) ||
        ground_search_options_.landing_via_min_dz < 0.0) {
      throw std::invalid_argument(
        "planning.ground_landing_via_min_dz must be non-negative and finite");
    }
    if (!std::isfinite(ground_search_options_.landing_via_radius) ||
        ground_search_options_.landing_via_radius < 0.0) {
      throw std::invalid_argument(
        "planning.ground_landing_via_radius must be non-negative and finite");
    }
    if (config.optimizer.interval <= 0.0) {
      throw std::invalid_argument("optimizer.interval must be positive");
    }
    if (config.max_fallback_attempts <= 0) {
      throw std::invalid_argument("fallback.max_attempts must be positive");
    }
    if (config.dynamic_limits.max_velocity < 0.0) {
      throw std::invalid_argument("planning.max_velocity must be non-negative");
    }
    if (config.dynamic_limits.max_acceleration < 0.0) {
      throw std::invalid_argument("planning.max_acceleration must be non-negative");
    }
    if (config.feasibility_sample_step_seconds <= 0.0) {
      throw std::invalid_argument("planning.feasibility_sample_dt must be positive");
    }
    if (config.max_dynamic_time_scale <= 0.0) {
      throw std::invalid_argument("planning.max_dynamic_time_scale must be positive");
    }
    if (config.min_time_allocation_interval <= 0.0) {
      throw std::invalid_argument("planning.min_time_allocation_interval must be positive");
    }
    if (!std::isfinite(planning_inflation_radius_) || planning_inflation_radius_ < 0.0) {
      throw std::invalid_argument("planning.inflation_radius must be non-negative and finite");
    }
    if (uav_endpoint_snap_radius_cells_ < 0) {
      throw std::invalid_argument("planning.uav_endpoint_snap_radius_cells must be non-negative");
    }
    // Ground: never cubic-smooth the supported A* polyline — smoothness chords
    // L1→L2 through the stairwell and invents a pitch that jams the body box.
    if (planning_traversability_mode_ == PlanningTraversabilityMode::Ground) {
      config.prefer_path_following_trajectory = true;
    }
    return config;
  }

  bool isExpectedFrame(const geometry_msgs::msg::PoseStamped& msg) const
  {
    return msg.header.frame_id == frame_id_;
  }

  bool isExpectedFrame(const geometry_msgs::msg::PointStamped& msg) const
  {
    return msg.header.frame_id == frame_id_;
  }

  bool isExpectedFrame(const sensor_msgs::msg::PointCloud2& msg) const
  {
    return msg.header.frame_id == frame_id_;
  }

  std::optional<nav_msgs::msg::OccupancyGrid> makeOccupancyGrid()
  {
    nav3d::map::Map2DProjectionOptions projection_options;
    projection_options.max_cells = occupancy_grid_max_cells_;
    projection_options.min_z = occupancy_grid_min_z_;
    projection_options.max_z = occupancy_grid_max_z_;
    std::optional<GroundTraversabilityMap> ground_traversability_map;
    if (planning_traversability_mode_ == PlanningTraversabilityMode::Ground) {
      // Visualization only: skip corner keep-out. Auto L-corner detection is
      // far too expensive to run per cell while projecting the full 2D grid
      // (would stall startup so RViz never receives map markers).
      auto viz_options = ground_search_options_;
      viz_options.corner_keepout_radius = 0.0;
      viz_options.corner_keepouts.clear();
      ground_traversability_map.emplace(*map_, std::move(viz_options));
      projection_options.free_cell_z = occupancy_grid_min_z_ + map_->getResolution() * 0.5;
      projection_options.is_free_cell =
        [&ground_traversability_map](const nav3d::common::Point3D& point) {
          return ground_traversability_map->isFree(point);
        };
    }
    const auto projected =
      nav3d::map::Map2DProjection::projectOccupiedVoxels(*map_, projection_options);
    if (!projected.has_value()) {
      const auto bounds = map_->getBounds();
      if (bounds.valid) {
        const double resolution = map_->getResolution();
        const int min_x = static_cast<int>(std::floor(bounds.min.x / resolution + 1e-9));
        const int max_x = static_cast<int>(std::floor(bounds.max.x / resolution + 1e-9));
        const int min_y = static_cast<int>(std::floor(bounds.min.y / resolution + 1e-9));
        const int max_y = static_cast<int>(std::floor(bounds.max.y / resolution + 1e-9));
        const auto cells =
          static_cast<std::uint64_t>(max_x - min_x + 1) *
          static_cast<std::uint64_t>(max_y - min_y + 1);
        if (cells > static_cast<std::uint64_t>(occupancy_grid_max_cells_)) {
          std::ostringstream status;
          status << "occupied_grid_skipped cells=" << cells
                 << " limit=" << occupancy_grid_max_cells_;
          publishStatus(status.str());
          RCLCPP_WARN(get_logger(), "%s", status.str().c_str());
        }
      }
      return std::nullopt;
    }

    nav_msgs::msg::OccupancyGrid grid;
    grid.header.stamp = now();
    grid.header.frame_id = frame_id_;
    grid.info.map_load_time = grid.header.stamp;
    grid.info.resolution = static_cast<float>(projected->resolution);
    grid.info.width = static_cast<std::uint32_t>(projected->width);
    grid.info.height = static_cast<std::uint32_t>(projected->height);
    grid.info.origin.position.x = projected->origin.x;
    grid.info.origin.position.y = projected->origin.y;
    grid.info.origin.position.z = projected->origin.z;
    grid.info.origin.orientation.w = 1.0;
    grid.data = projected->data;
    return grid;
  }

  void publishOccupancyGrid()
  {
    const auto grid = makeOccupancyGrid();
    if (!grid.has_value()) {
      return;
    }
    occupied_grid_pub_->publish(*grid);
    std::ostringstream status;
    status << "occupied_grid_published width=" << grid->info.width
           << " height=" << grid->info.height
           << " occupied=" << std::count(grid->data.begin(), grid->data.end(), 100)
           << " free=" << std::count(grid->data.begin(), grid->data.end(), 0)
           << " unknown=" << std::count(grid->data.begin(), grid->data.end(), -1)
           << " min_z=" << occupancy_grid_min_z_
           << " max_z=" << occupancy_grid_max_z_;
    publishStatus(status.str());
    RCLCPP_INFO(get_logger(), "%s", status.str().c_str());
  }

  visualization_msgs::msg::Marker makeVoxelCubeList(
    const std_msgs::msg::Header& header,
    const std::string& ns,
    int id,
    double resolution,
    const std::vector<geometry_msgs::msg::Point>& points,
    float red,
    float green,
    float blue,
    float alpha) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header = header;
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = resolution;
    marker.scale.y = resolution;
    marker.scale.z = resolution;
    marker.color.r = red;
    marker.color.g = green;
    marker.color.b = blue;
    marker.color.a = alpha;
    marker.points = points;
    return marker;
  }

  visualization_msgs::msg::MarkerArray makePlanningOccupiedMarkerArray() const
  {
    std_msgs::msg::Header header;
    header.stamp = now();
    header.frame_id = frame_id_;

    visualization_msgs::msg::MarkerArray array;
    visualization_msgs::msg::Marker cleanup;
    cleanup.header = header;
    cleanup.ns = "nav3d_planning_occupied_voxels_cleanup";
    cleanup.id = 0;
    cleanup.action = visualization_msgs::msg::Marker::DELETEALL;
    array.markers.push_back(cleanup);

    const double resolution = map_->getResolution();
    std::vector<geometry_msgs::msg::Point> points;
    points.reserve(map_->occupiedCells().size());
    for (const auto& cell : map_->occupiedCells()) {
      const auto corner = map_->gridToWorld(cell);
      geometry_msgs::msg::Point point;
      point.x = corner.x + resolution * 0.5;
      point.y = corner.y + resolution * 0.5;
      point.z = corner.z + resolution * 0.5;
      points.push_back(point);
    }

    array.markers.push_back(makeVoxelCubeList(
      header,
      "nav3d_planning_occupied_voxels",
      0,
      resolution,
      points,
      0.1F,
      0.85F,
      0.25F,
      0.78F));

    if (local_grid_) {
      const double local_resolution = local_grid_->getResolution();
      std::vector<geometry_msgs::msg::Point> local_points;
      local_points.reserve(local_grid_->observedCells().size());
      for (const auto& [cell, state] : local_grid_->observedCells()) {
        if (state != nav3d::map::CellState::Occupied) {
          continue;
        }
        const auto corner = local_grid_->gridToWorld(cell);
        geometry_msgs::msg::Point point;
        point.x = corner.x + local_resolution * 0.5;
        point.y = corner.y + local_resolution * 0.5;
        point.z = corner.z + local_resolution * 0.5;
        local_points.push_back(point);
      }
      if (!local_points.empty()) {
        array.markers.push_back(makeVoxelCubeList(
          header,
          "nav3d_local_occupied_voxels",
          1,
          local_resolution,
          local_points,
          0.0F,
          0.65F,
          1.0F,
          0.92F));
      }
    }
    return array;
  }

  std::size_t localOccupiedVoxelCount() const
  {
    if (!local_grid_) {
      return 0;
    }
    return static_cast<std::size_t>(std::count_if(
      local_grid_->observedCells().begin(),
      local_grid_->observedCells().end(),
      [](const auto& entry) {
        return entry.second == nav3d::map::CellState::Occupied;
      }));
  }

  void publishPlanningOccupiedMarkers()
  {
    if (!map_) {
      publishStatus("planning_occupied_markers_skipped no_map");
      return;
    }

    const auto markers = makePlanningOccupiedMarkerArray();
    planning_occupied_markers_pub_->publish(markers);
    std::ostringstream status;
    status << "planning_occupied_markers_published voxels=" << map_->occupiedCells().size()
           << " local_voxels=" << localOccupiedVoxelCount();
    publishStatus(status.str());
    RCLCPP_INFO(get_logger(), "%s", status.str().c_str());
  }

  void publishOctomapBinary()
  {
#ifdef NAV3D_HAS_OCTOMAP_MSGS
    if (!octomap_) {
      publishStatus("octomap_binary_skipped no_map");
      return;
    }

    octomap_msgs::msg::Octomap msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;
    if (!octomap_msgs::binaryMapToMsg(octomap_->tree(), msg)) {
      publishStatus("octomap_binary_failed conversion_failed");
      RCLCPP_WARN(get_logger(), "Failed to convert OctoMap to octomap_msgs/Octomap");
      return;
    }

    octomap_pub_->publish(msg);
    std::ostringstream status;
    status << "octomap_binary_published bytes=" << msg.data.size()
           << " leafs=" << octomap_->occupiedLeafCount();
    publishStatus(status.str());
    RCLCPP_INFO(get_logger(), "%s", status.str().c_str());
#else
    publishStatus("octomap_binary_unavailable octomap_msgs_not_found");
#endif
  }

  void publishStaticMapVisualization()
  {
    publishOccupancyGrid();
    publishPlanningOccupiedMarkers();
    publishOctomapBinary();
  }

  void startStaticVisualizationRepublish()
  {
    static_visualization_republish_count_ = 0;
    static_visualization_timer_ = create_wall_timer(
      std::chrono::seconds(1),
      [this]() {
        publishStaticMapVisualization();
        ++static_visualization_republish_count_;
        if (static_visualization_republish_count_ >= 5 && static_visualization_timer_) {
          static_visualization_timer_->cancel();
        }
      });
  }

  nav3d::common::Point3D transformPointToMap(
    const nav3d::common::Point3D& point,
    const std::optional<geometry_msgs::msg::TransformStamped>& transform) const
  {
    if (!transform.has_value()) {
      return point;
    }

    geometry_msgs::msg::PointStamped source;
    geometry_msgs::msg::PointStamped target;
    source.header.frame_id = transform->child_frame_id;
    source.point.x = point.x;
    source.point.y = point.y;
    source.point.z = point.z;
    tf2::doTransform(source, target, *transform);
    return {target.point.x, target.point.y, target.point.z};
  }

  nav3d::common::Point3D transformOriginToMap(
    const geometry_msgs::msg::TransformStamped& transform) const
  {
    return {
      transform.transform.translation.x,
      transform.transform.translation.y,
      transform.transform.translation.z,
    };
  }

  void onLocalPointCloud(const sensor_msgs::msg::PointCloud2& msg)
  {
    if (!local_grid_) {
      publishStatus("local_pointcloud_ignored local_grid_disabled");
      return;
    }

    const rclcpp::Time cloud_stamp(msg.header.stamp);
    const bool has_cloud_stamp = cloud_stamp.nanoseconds() != 0;
    const rclcpp::Time received_at = now();
    if (has_cloud_stamp) {
      const double age_sec = (received_at - cloud_stamp).seconds();
      if (local_grid_max_observation_age_sec_ > 0.0 &&
          age_sec > local_grid_max_observation_age_sec_) {
        std::ostringstream status;
        status << "local_pointcloud_rejected stale_stamp age_sec=" << age_sec
               << " max_age_sec=" << local_grid_max_observation_age_sec_;
        publishStatus(status.str());
        RCLCPP_WARN(get_logger(), "%s", status.str().c_str());
        return;
      }
      if (local_grid_max_future_stamp_sec_ > 0.0 &&
          -age_sec > local_grid_max_future_stamp_sec_) {
        std::ostringstream status;
        status << "local_pointcloud_rejected future_stamp lead_sec=" << -age_sec
               << " max_future_sec=" << local_grid_max_future_stamp_sec_;
        publishStatus(status.str());
        RCLCPP_WARN(get_logger(), "%s", status.str().c_str());
        return;
      }
    }

    std::optional<geometry_msgs::msg::TransformStamped> pointcloud_transform;
    if (msg.header.frame_id != frame_id_) {
      try {
        pointcloud_transform = tf_buffer_->lookupTransform(
          frame_id_,
          msg.header.frame_id,
          has_cloud_stamp ? cloud_stamp : rclcpp::Time(0));
      } catch (const tf2::TransformException& error) {
        publishStatus("local_pointcloud_rejected frame_transform_unavailable");
        RCLCPP_WARN(
          get_logger(),
          "Rejected local pointcloud in frame '%s'; expected '%s' or a TF transform: %s",
          msg.header.frame_id.c_str(),
          frame_id_.c_str(),
          error.what());
        return;
      }
    }

    std::size_t occupied = 0;
    std::size_t ray_cleared = 0;
    std::size_t skipped = 0;
    std::vector<nav3d::common::Point3D> frame_points;
    std::optional<nav3d::common::Point3D> ray_origin;
    if (pointcloud_transform.has_value()) {
      ray_origin = transformOriginToMap(*pointcloud_transform);
    } else if (current_position_.has_value()) {
      ray_origin = current_position_;
    }
    const bool can_ray_clear =
      local_grid_ray_clearing_enabled_ &&
      ray_origin.has_value() &&
      local_grid_->isInBounds(*ray_origin);
    try {
      sensor_msgs::PointCloud2ConstIterator<float> x_iter(msg, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y_iter(msg, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z_iter(msg, "z");
      for (; x_iter != x_iter.end(); ++x_iter, ++y_iter, ++z_iter) {
        const nav3d::common::Point3D raw_point{*x_iter, *y_iter, *z_iter};
        const nav3d::common::Point3D point =
          transformPointToMap(raw_point, pointcloud_transform);
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z) ||
            !local_grid_->isInBounds(point)) {
          ++skipped;
          continue;
        }
        frame_points.push_back(point);
        if (can_ray_clear) {
          ++ray_cleared;
        }
        ++occupied;
      }
    } catch (const std::runtime_error& error) {
      publishStatus("local_pointcloud_rejected invalid_xyz_fields");
      RCLCPP_WARN(get_logger(), "Rejected local pointcloud: %s", error.what());
      return;
    }

    updateLocalObservationWindow(
      frame_points,
      can_ray_clear ? ray_origin : std::nullopt,
      has_cloud_stamp ? cloud_stamp : received_at);

    std::ostringstream status;
    status << "local_pointcloud_updated occupied=" << occupied
           << " ray_cleared=" << ray_cleared
           << " skipped=" << skipped
           << " retained_frames=" << local_pointcloud_frames_.size();
    publishStatus(status.str());
    RCLCPP_INFO(get_logger(), "%s", status.str().c_str());
    publishPlanningOccupiedMarkers();
    // A fresh local cloud is an obstacle update, not just visualization data.
    // Evaluate immediately so a newly spawned travel anchor can trigger a
    // replan without waiting for the next pose tick; safety_replan_cooldown_sec
    // still prevents repeated replans during bursty pointcloud publishes.
    if (current_position_.has_value() && active_trajectory_.has_value()) {
      evaluateActiveTrajectorySafety(*current_position_);
    }
  }

  void onLoadPcdPath(const std_msgs::msg::String& msg)
  {
    const std::string pcd_path = trim(msg.data);
    if (pcd_path.empty()) {
      publishStatus("map_reload_rejected empty_pcd_path");
      RCLCPP_WARN(get_logger(), "Rejected map reload with an empty PCD path");
      return;
    }

    nav3d::map::MapBuildConfig next_config = map_build_config_;
    next_config.pcd_path = pcd_path;
    try {
      auto next_map = std::make_unique<nav3d::map::VoxelGridMap>(buildMap(next_config));
      auto next_octomap = std::make_unique<nav3d::map::OctomapManager>(buildOctomap(next_config));
      std::unique_ptr<nav3d::map::LocalGrid> next_local_grid;
      if (local_grid_enabled_) {
        next_local_grid = makeLocalGridForMap(*next_map);
      }
      const bool had_active_tracking = tracking_active_;
      tracking_active_ = false;
      active_trajectory_.reset();
      active_goal_.reset();
      clearPublishedTrajectory();
      abortPendingNavigateGoal("navigate_aborted_map_reload");
      map_ = std::move(next_map);
      octomap_ = std::move(next_octomap);
      map_build_config_ = next_config;
      if (local_grid_enabled_) {
        local_grid_ = std::move(next_local_grid);
        clearLocalObservationWindow();
      } else {
        local_grid_.reset();
        local_pointcloud_frames_.clear();
      }
      publishStaticMapVisualization();
      startStaticVisualizationRepublish();
      if (had_active_tracking) {
        publishZeroCommand("map_reload_stop_tracking");
      }

      std::ostringstream status;
      status << "map_reload_success pcd_path=" << pcd_path
             << " occupied=" << map_->occupiedCells().size()
             << " resolution=" << map_->getResolution();
      publishStatus(status.str());
      RCLCPP_INFO(get_logger(), "%s", status.str().c_str());
    } catch (const std::exception& error) {
      std::ostringstream status;
      status << "map_reload_failed pcd_path=" << pcd_path
             << " error=" << error.what();
      publishStatus(status.str());
      RCLCPP_WARN(get_logger(), "%s", status.str().c_str());
    }
  }

  void onInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped& msg)
  {
    geometry_msgs::msg::PoseStamped start;
    start.header = msg.header;
    start.pose = msg.pose.pose;
    onStart(start);
  }

  void onStart(const geometry_msgs::msg::PoseStamped& msg)
  {
    if (!isExpectedFrame(msg)) {
      publishStatus("start_rejected frame_mismatch");
      RCLCPP_WARN(
        get_logger(),
        "Rejected start in frame '%s'; expected '%s'",
        msg.header.frame_id.c_str(),
        frame_id_.c_str());
      return;
    }

    const auto requested = toPoint(msg);
    RCLCPP_INFO(
      get_logger(),
      "start_received x=%.3f y=%.3f z=%.3f (snapping...)",
      requested.x,
      requested.y,
      requested.z);
    // Snap start onto a free/traversable cell (esp. ground mode: floor voxels
    // are occupied; standing on z=0 often leaves the tracker with zero cmd_vel).
    start_ = resolveClickedPoint(requested, "start");
    // v3.8: setting a fresh start invalidates the previous current_position_.
    // Otherwise W-A's planToGoal would use the stale current_position_ (left
    // over from a prior follow-trajectory run that walked far past start_)
    // and the next /nav3d/goal would plan from there, producing an empty or
    // trivial path when the operator's intent was "robot is at this start
    // now, plan from here".
    current_position_ = start_;
    publishEndpointMarker(start_, "nav3d_start", *start_marker_pub_, 0.1F, 0.95F, 0.25F);

    // Tell pose_sim to teleport to the *snapped* start (not the raw click).
    {
      geometry_msgs::msg::PoseStamped snapped;
      snapped.header.stamp = now();
      snapped.header.frame_id = frame_id_;
      snapped.pose.position.x = start_.x;
      snapped.pose.position.y = start_.y;
      snapped.pose.position.z = start_.z;
      snapped.pose.orientation = msg.pose.orientation;
      pose_set_pub_->publish(snapped);
    }

    std::ostringstream status;
    status << "start_updated x=" << start_.x << " y=" << start_.y << " z=" << start_.z;
    publishStatus(status.str());
    RCLCPP_INFO(get_logger(), "%s", status.str().c_str());
  }

  nav3d::common::Point3D resolveUavEndpoint(
    const nav3d::map::IMap& planning_map,
    const nav3d::common::Point3D& requested) const
  {
    if (planning_traversability_mode_ != PlanningTraversabilityMode::Uav ||
        ground_search_options_.mode != nav3d::planner::PlanningMode::Mode3D ||
        uav_endpoint_snap_radius_cells_ == 0 ||
        planning_map.getResolution() <= 0.0) {
      return requested;
    }

    const double resolution = planning_map.getResolution();
    const auto requested_idx = worldToGroundIndex(requested, resolution);
    bool has_best = false;
    nav3d::common::Point3D best = requested;
    int best_below_rank = 1;
    int best_clearance_rank = 1;
    double best_distance = std::numeric_limits<double>::infinity();
    for (int radius = 0; radius <= uav_endpoint_snap_radius_cells_; ++radius) {
      for (int dx = -radius; dx <= radius; ++dx) {
        for (int dy = -radius; dy <= radius; ++dy) {
          for (int dz = -radius; dz <= radius; ++dz) {
            if (std::max({std::abs(dx), std::abs(dy), std::abs(dz)}) != radius) {
              continue;
            }

            const auto candidate = groundIndexToWorld(
              {requested_idx.x + dx, requested_idx.y + dy, requested_idx.z + dz},
              resolution);
            if (!planning_map.isFree(candidate)) {
              continue;
            }

            const int below_rank = candidate.z + 1e-9 >= requested.z ? 0 : 1;
            const double distance = nav3d::common::distance(candidate, requested);
            const int clearance_rank =
              hasOccupiedNeighbor(planning_map, candidate, resolution) ? 1 : 0;
            constexpr double kEpsilon = 1e-12;
            const bool better =
              !has_best ||
              below_rank < best_below_rank ||
              (below_rank == best_below_rank && clearance_rank < best_clearance_rank) ||
              (below_rank == best_below_rank && clearance_rank == best_clearance_rank &&
               distance < best_distance - kEpsilon);
            if (better) {
              has_best = true;
              best = candidate;
              best_below_rank = below_rank;
              best_clearance_rank = clearance_rank;
              best_distance = distance;
            }
          }
        }
      }
      if (has_best && best_below_rank == 0 && best_clearance_rank == 0) {
        return best;
      }
    }

    return has_best ? best : requested;
  }

  nav3d::common::Point3D resolveGroundEndpoint(
    const nav3d::map::IMap& planning_map,
    const nav3d::common::Point3D& requested) const
  {
    if (!groundTraversabilityEnabled() || planning_map.getResolution() <= 0.0) {
      return requested;
    }

    const double resolution = planning_map.getResolution();
    const auto requested_idx = worldToGroundIndex(requested, resolution);
    // Endpoints must snap even near stair inner corners; keep-out applies to
    // path expansion, not to placing start/goal (RViz 2D tools always send z=0
    // into floor voxels).
    constexpr bool kEnforceKeepout = false;
    auto try_idx = [&](const GroundIndex& idx) -> std::optional<nav3d::common::Point3D> {
      if (!isGroundTraversable(planning_map, idx, ground_search_options_, kEnforceKeepout)) {
        return std::nullopt;
      }
      return groundIndexToWorld(idx, resolution);
    };

    const auto requested_world = groundIndexToWorld(requested_idx, resolution);
    const bool requested_occupied =
      planning_map.isInBounds(requested_world) && planning_map.isOccupied(requested_world);

    // Keep an already-valid free tread click (do not lift z=0.3 → 0.5).
    // Never accept an occupied voxel as "valid" — RViz Pose Estimate / Nav2 Goal
    // publish z=0 which lands inside the floor slab.
    if (!requested_occupied) {
      if (auto p = try_idx(requested_idx)) {
        return *p;
      }
    }

    // Prefer the first free tread above the click (typical RViz z=0 → stance).
    constexpr int kMaxLiftCells = 8;
    for (int dz = 1; dz <= kMaxLiftCells; ++dz) {
      if (auto p = try_idx({requested_idx.x, requested_idx.y, requested_idx.z + dz})) {
        return *p;
      }
      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          if (dx == 0 && dy == 0) {
            continue;
          }
          if (auto p = try_idx(
                {requested_idx.x + dx, requested_idx.y + dy, requested_idx.z + dz})) {
            return *p;
          }
        }
      }
    }

    const auto resolved_idx =
      resolveGroundIndex(planning_map, requested_idx, ground_search_options_);
    if (auto p = try_idx(resolved_idx)) {
      return *p;
    }

    RCLCPP_WARN(
      get_logger(),
      "ground_endpoint_snap_failed requested=%.3f,%.3f,%.3f (still inside/near occupied); "
      "increase planning.ground_snap_radius_cells",
      requested.x,
      requested.y,
      requested.z);
    return requested;
  }

  bool hasOccupiedNeighbor(
    const nav3d::map::IMap& planning_map,
    const nav3d::common::Point3D& point,
    double resolution) const
  {
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dz = -1; dz <= 1; ++dz) {
          if (dx == 0 && dy == 0 && dz == 0) {
            continue;
          }
          const nav3d::common::Point3D neighbor{
            point.x + static_cast<double>(dx) * resolution,
            point.y + static_cast<double>(dy) * resolution,
            point.z + static_cast<double>(dz) * resolution,
          };
          if (planning_map.isInBounds(neighbor) && planning_map.isOccupied(neighbor)) {
            return true;
          }
        }
      }
    }
    return false;
  }

  void onClickedPoint(const geometry_msgs::msg::PointStamped& msg)
  {
    if (!isExpectedFrame(msg)) {
      publishStatus("clicked_point_rejected frame_mismatch");
      RCLCPP_WARN(
        get_logger(),
        "Rejected clicked point in frame '%s'; expected '%s'",
        msg.header.frame_id.c_str(),
        frame_id_.c_str());
      return;
    }

    const auto requested_point = toPoint(msg);
    if (next_clicked_point_is_start_) {
      const auto point = resolveClickedPoint(requested_point, "start");
      start_ = point;
      next_clicked_point_is_start_ = false;
      clearPublishedTrajectory();
      publishEndpointMarker(start_, "nav3d_start", *start_marker_pub_, 0.1F, 0.95F, 0.25F);
      clearEndpointMarker("nav3d_goal", *goal_marker_pub_);
      std::ostringstream status;
      status << "clicked_start_updated x=" << start_.x
             << " y=" << start_.y
             << " z=" << start_.z;
      publishStatus(status.str());
      RCLCPP_INFO(get_logger(), "%s; click the goal point next", status.str().c_str());
      return;
    }

    next_clicked_point_is_start_ = true;
    const auto point = resolveClickedPoint(requested_point, "goal");
    planToGoal(point, "clicked_plan");
  }

  void onCurrentPose(const geometry_msgs::msg::PoseStamped& msg)
  {
    if (!isExpectedFrame(msg)) {
      publishStatus("current_pose_rejected frame_mismatch");
      RCLCPP_WARN(
        get_logger(),
        "Rejected current pose in frame '%s'; expected '%s'",
        msg.header.frame_id.c_str(),
        frame_id_.c_str());
      return;
    }

    current_position_ = toPoint(msg);
    current_yaw_ = tf2::getYaw(msg.pose.orientation);
    evaluateActiveTrajectorySafety(*current_position_);
    if (handleCurrentPoseGoalReached(*current_position_)) {
      return;
    }
    // Throttle: republishing the full Path on every pose (~20 Hz) floods
    // rosbridge fragments and leaves the Web UI stuck on "等待 bridge 规划".
    publishRemainingTrajectory(*current_position_);
  }

  void onPlanPath(
    const std::shared_ptr<GetPlan::Request> request,
    std::shared_ptr<GetPlan::Response> response)
  {
    response->plan.header.stamp = now();
    response->plan.header.frame_id = frame_id_;
    if (!isExpectedFrame(request->start) || !isExpectedFrame(request->goal)) {
      publishStatus("service_plan_rejected frame_mismatch");
      RCLCPP_WARN(
        get_logger(),
        "Rejected GetPlan request start_frame='%s' goal_frame='%s'; expected '%s'",
        request->start.header.frame_id.c_str(),
        request->goal.header.frame_id.c_str(),
        frame_id_.c_str());
      return;
    }

    const auto result = planWithActiveMap(toPoint(request->start), toPoint(request->goal));
    if (!result.success) {
      std::ostringstream status;
      status << "service_plan_failed status=" << statusToString(result.status)
             << " attempts=" << result.attempts
             << " tolerance=" << request->tolerance;
      appendPlanFailureDebug(status, result, planningGoalTolerance());
      publishStatus(status.str());
      RCLCPP_WARN(get_logger(), "%s", status.str().c_str());
      return;
    }

    response->plan = sampleTrajectoryPath(result.trajectory);
    std::ostringstream status;
    status << "service_plan_success poses=" << response->plan.poses.size()
           << " attempts=" << result.attempts
           << " tolerance=" << request->tolerance;
    appendPlanEndpointStatus(status, result, planningGoalTolerance());
    publishStatus(status.str());
    RCLCPP_INFO(get_logger(), "%s", status.str().c_str());
  }

  void onGoal(const geometry_msgs::msg::PoseStamped& msg)
  {
    if (!isExpectedFrame(msg)) {
      publishStatus("goal_rejected frame_mismatch");
      RCLCPP_WARN(
        get_logger(),
        "Rejected goal in frame '%s'; expected '%s'",
        msg.header.frame_id.c_str(),
        frame_id_.c_str());
      return;
    }

    const auto requested = toPoint(msg);
    RCLCPP_INFO(
      get_logger(),
      "goal_received x=%.3f y=%.3f z=%.3f (snapping...)",
      requested.x,
      requested.y,
      requested.z);
    // Snap goal to a free, traversable cell when the requested point is
    // occupied / unsupported. Web UI publishes /nav3d/goal directly without
    // going through /clicked_point, so without this call a click that lands
    // on an obstacle voxel falls through to plan_failed status=no_path with
    // no recovery. Match onClickedPoint behaviour: snap, log, then plan.
    const auto resolved = resolveClickedPoint(requested, "goal");
    planToGoal(resolved, "plan");
  }

  void onCancelPlan()
  {
    const auto epoch = plan_epoch_.fetch_add(1) + 1;
    tracking_active_ = false;
    active_trajectory_.reset();
    active_goal_.reset();
    last_controller_command_.reset();
    clearPublishedTrajectory();
    publishZeroCommand("plan_canceled");
    std::ostringstream status;
    status << "plan_canceled epoch=" << epoch;
    publishStatus(status.str());
    RCLCPP_WARN(get_logger(), "%s", status.str().c_str());
  }

  void planToGoal(const nav3d::common::Point3D& goal, const std::string& status_prefix)
  {
    publishEndpointMarker(goal, "nav3d_goal", *goal_marker_pub_, 0.95F, 0.2F, 0.12F);
    // W-A (v3.8): plan from the live current_position_ when available so a
    // mid-flight /nav3d/goal click does not re-issue the path from a stale
    // start_ (the original origin). Without this, when the robot has moved
    // past obstacles the planner re-plans through cells that now hold local
    // obstacle voxels, producing slow/no-path replans. action API at L2165
    // already does this; only the topic-driven onGoal path was wrong.
    const auto plan_start = current_position_.value_or(start_);
    const std::uint64_t my_epoch = plan_epoch_.fetch_add(1) + 1;
    {
      std::ostringstream status;
      status << status_prefix << "_started epoch=" << my_epoch
             << " start=" << plan_start.x << "," << plan_start.y << "," << plan_start.z
             << " goal=" << goal.x << "," << goal.y << "," << goal.z;
      publishStatus(status.str());
      RCLCPP_INFO(get_logger(), "%s (large ground maps can take a while)", status.str().c_str());
    }
    const auto result = planWithActiveMap(plan_start, goal);
    if (my_epoch != plan_epoch_.load()) {
      std::ostringstream status;
      status << status_prefix << "_superseded epoch=" << my_epoch
             << " current_epoch=" << plan_epoch_.load();
      publishStatus(status.str());
      RCLCPP_WARN(get_logger(), "%s", status.str().c_str());
      return;
    }
    if (!result.success) {
      tracking_active_ = false;
      active_trajectory_.reset();
      active_goal_.reset();
      clearPublishedTrajectory();
      std::ostringstream status;
      status << status_prefix << "_failed status=" << statusToString(result.status)
             << " attempts=" << result.attempts;
      appendPlanFailureDebug(status, result, planningGoalTolerance());
      publishStatus(status.str());
      RCLCPP_WARN(get_logger(), "%s", status.str().c_str());
      if (result.status == nav3d::planner::EgoPlanStatus::EmergencyStop ||
          result.fallback.action == nav3d::planner::FallbackAction::EmergencyStop) {
        publishZeroCommand("plan_emergency_stop");
      }
      return;
    }

    active_trajectory_ = result.trajectory;
    active_goal_ = result.planned_goal;
    tracking_active_ = controller_enabled_;
    tracking_path_time_ = 0.0;
    publishTrajectory(result);
    std::ostringstream status;
    status << status_prefix << "_success poses=" << last_published_pose_count_
           << " attempts=" << result.attempts
           << " time_scale=" << result.time_scale
           << " epoch=" << my_epoch;
    appendPlanEndpointStatus(status, result, planningGoalTolerance());
    publishStatus(status.str());
    RCLCPP_INFO(get_logger(), "%s", status.str().c_str());
  }

  rclcpp_action::GoalResponse handleNavigateGoal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const NavigateToPose::Goal> goal)
  {
    (void)uuid;
    if (goal->pose.header.frame_id != frame_id_) {
      publishStatus("navigate_goal_rejected frame_mismatch");
      RCLCPP_WARN(
        get_logger(),
        "Rejected NavigateToPose goal in frame '%s'; expected '%s'",
        goal->pose.header.frame_id.c_str(),
        frame_id_.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (hasPendingNavigateGoal()) {
      publishStatus("navigate_goal_rejected active_goal");
      RCLCPP_WARN(get_logger(), "Rejected NavigateToPose goal because another goal is active");
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleNavigateCancel(
    const std::shared_ptr<NavigateGoalHandle> goal_handle)
  {
    (void)goal_handle;
    tracking_active_ = false;
    active_trajectory_.reset();
    active_goal_.reset();
    publishZeroCommand("navigate_cancel");
    std::thread{[this, goal_handle]() {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      cancelPendingNavigateGoal(goal_handle, "navigate_canceled");
    }}.detach();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleNavigateAccepted(const std::shared_ptr<NavigateGoalHandle> goal_handle)
  {
    std::thread{[this, goal_handle]() {
      executeNavigateGoal(goal_handle);
    }}.detach();
  }

  void executeNavigateGoal(const std::shared_ptr<NavigateGoalHandle> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    const auto goal_point = toPoint(goal->pose);
    const auto start = current_position_.value_or(start_);

    auto feedback = std::make_shared<NavigateToPose::Feedback>();
    std_msgs::msg::Header feedback_header;
    feedback_header.stamp = now();
    feedback_header.frame_id = frame_id_;
    feedback->current_pose = toPoseStamped(start, feedback_header);
    feedback->distance_remaining = static_cast<float>(nav3d::common::distance(start, goal_point));
    goal_handle->publish_feedback(feedback);

    const auto plan = planWithActiveMap(start, goal_point);
    auto result = std::make_shared<NavigateToPose::Result>();
    if (goal_handle->is_canceling()) {
      goal_handle->canceled(result);
      publishStatus("navigate_canceled");
      return;
    }

    if (plan.success) {
      active_goal_ = plan.planned_goal;
      active_trajectory_ = plan.trajectory;
      tracking_active_ = controller_enabled_;
      tracking_path_time_ = 0.0;
      publishTrajectory(plan);
      if (controller_enabled_) {
        setPendingNavigateGoal(
          goal_handle,
          plan.requested_goal,
          plan.planned_goal);
        std::ostringstream status;
        status << "navigate_tracking_active poses=" << last_published_pose_count_
               << " attempts=" << plan.attempts;
        appendPlanEndpointStatus(status, plan, planningGoalTolerance());
        publishStatus(status.str());
        RCLCPP_INFO(get_logger(), "%s", status.str().c_str());
        return;
      }
      std::ostringstream status;
      status << (isPartialPlan(plan, planningGoalTolerance()) ? "navigate_partial_goal_reached" : "navigate_success")
             << " poses=" << last_published_pose_count_
             << " attempts=" << plan.attempts
             << " tracking_disabled=true";
      appendPlanEndpointStatus(status, plan, planningGoalTolerance());
      publishStatus(status.str());
      if (isPartialPlan(plan, planningGoalTolerance())) {
        goal_handle->abort(result);
        RCLCPP_WARN(get_logger(), "%s", status.str().c_str());
      } else {
        goal_handle->succeed(result);
        RCLCPP_INFO(get_logger(), "%s", status.str().c_str());
      }
      return;
    }

    std::ostringstream status;
    status << "navigate_failed status=" << statusToString(plan.status)
           << " attempts=" << plan.attempts;
    publishStatus(status.str());
    tracking_active_ = false;
    if (plan.status == nav3d::planner::EgoPlanStatus::EmergencyStop ||
        plan.fallback.action == nav3d::planner::FallbackAction::EmergencyStop) {
      publishZeroCommand("navigate_emergency_stop");
    }
    goal_handle->abort(result);
    RCLCPP_WARN(get_logger(), "%s", status.str().c_str());
  }

  void evaluateActiveTrajectorySafety(const nav3d::common::Point3D& current_position)
  {
    if (!safety_monitor_ || !active_trajectory_.has_value() || !active_goal_.has_value()) {
      return;
    }

    // Ground stair ribbons: free cells sit above each tread, but the B-spline
    // chord between them routinely clips occupied riser voxels, and pose_sim
    // often rests on/near floor-occupied cells. Any safety action here
    // (ahead-collision local A* stitches OR current-pose emergency stop)
    // aborts climb and leaves the robot skating/frozen on the tread.
    // Skip BEFORE evaluateSafetyWithActiveMap: that path wraps the full
    // GroundTraversabilityMap and scans the whole spline each pose tick
    // (~20 Hz), which starved the controller timer (~0.6 Hz cmd_vel) and
    // produced stop-start motion via pose_sim cmd_timeout.
    // Rely on the monotonic tracker + tracking_lost_path instead.
    if (groundTraversabilityEnabled()) {
      return;
    }

    const auto decision = evaluateSafetyWithActiveMap(current_position);

    if (decision.action == nav3d::controller::SafetyAction::EmergencyStop) {
      // W-B (v3.8): before declaring emergency stop, try one bounded local
      // replan with raw A* feasibility probe + EGO local ladder. The original
      // logic abandoned the trajectory immediately on EmergencyStop, which
      // produced the "click an obstacle in front of the robot and bridge just
      // stops, no replan" symptom the user reported. The cooldown gate inside
      // replanFromCurrentPose already prevents thrashing when the obstacle
      // genuinely cannot be avoided locally; if the local + global replan
      // both fail there, the existing failure path still publishes
      // safety_replan_emergency_stop.
      if (safety_local_replan_enabled_ && active_trajectory_.has_value()) {
        std::ostringstream pre;
        pre << "safety_replan_attempt_before_emergency_stop";
        if (decision.first_collision_time.has_value()) {
          pre << " first_collision_time=" << *decision.first_collision_time;
        }
        publishStatus(pre.str());
        RCLCPP_WARN(get_logger(), "%s", pre.str().c_str());
        replanFromCurrentPose(current_position);
        // If replanFromCurrentPose succeeded it set active_trajectory_; if it
        // failed it already cleared everything + published the appropriate
        // failure status. Either way we are done with this pose tick.
        return;
      }
      // Without local replan: hard-abort only if the robot is already inside
      // an occupied voxel. Trajectory-ahead collisions (common on stair
      // B-splines that clip risers) become a soft zero-velocity hold so the
      // planned path stays visible instead of flashing safety_emergency_stop.
      if (decision.current_pose_in_collision) {
        tracking_active_ = false;
        active_trajectory_.reset();
        active_goal_.reset();
        clearPublishedTrajectory();
        abortPendingNavigateGoal("navigate_aborted_safety_emergency_stop");
        publishZeroCommand("safety_emergency_stop");
        return;
      }
      publishZeroCommand("safety_soft_stop_ahead_collision");
      return;
    }
    if (decision.action == nav3d::controller::SafetyAction::ReplanNeeded) {
      std::ostringstream status;
      status << "safety_replan_needed";
      if (decision.first_collision_time.has_value()) {
        status << " first_collision_time=" << *decision.first_collision_time;
      }
      publishStatus(status.str());
      RCLCPP_WARN(get_logger(), "%s", status.str().c_str());
      replanFromCurrentPose(current_position);
    }
  }

  template <typename Callback>
  decltype(auto) makePlanningMap(Callback&& callback) const
  {
    return makeCollisionPlanningMap(
      [&](const nav3d::map::IMap& planning_map) -> decltype(auto) {
        return withGroundTraversability(planning_map, std::forward<Callback>(callback));
      });
  }

  template <typename Callback>
  decltype(auto) makeCollisionPlanningMap(Callback&& callback) const
  {
    const nav3d::map::IMap& global_map = activeGlobalPlanningMap();
    // Ground mode already enforces support + body clearance via
    // ground_robot_radius. Applying UAV-style inflation (e.g. 0.5 m) lifts
    // free cells off the tread onto a thin disconnected shell and makes A*
    // crawl / hang on building-scale maps.
    const bool apply_inflation =
      planning_inflation_radius_ > 0.0 && !groundTraversabilityEnabled();
    if (hasLocalObservations()) {
      nav3d::map::MapComposite composite(global_map, *local_grid_);
      if (apply_inflation) {
        nav3d::collision::InflationLayer planning_map(
          composite,
          planning_inflation_radius_);
        return std::forward<Callback>(callback)(planning_map);
      }
      return std::forward<Callback>(callback)(composite);
    }
    if (apply_inflation) {
      nav3d::collision::InflationLayer planning_map(
        global_map,
        planning_inflation_radius_);
      return std::forward<Callback>(callback)(planning_map);
    }
    return std::forward<Callback>(callback)(global_map);
  }

  template <typename Callback>
  decltype(auto) withGroundTraversability(
    const nav3d::map::IMap& planning_map,
    Callback&& callback) const
  {
    if (!groundTraversabilityEnabled()) {
      return std::forward<Callback>(callback)(planning_map);
    }

    GroundTraversabilityMap traversability_map(planning_map, ground_search_options_);
    return std::forward<Callback>(callback)(traversability_map);
  }

  bool groundTraversabilityEnabled() const
  {
    return planning_traversability_mode_ == PlanningTraversabilityMode::Ground;
  }

  nav3d::common::Point3D resolveClickedPoint(
    const nav3d::common::Point3D& requested,
    const std::string& role)
  {
    pruneExpiredLocalObservations();
    const auto resolved = makeCollisionPlanningMap([&](const nav3d::map::IMap& planning_map) {
      if (groundTraversabilityEnabled()) {
        return resolveGroundEndpoint(planning_map, requested);
      }
      return resolveUavEndpoint(planning_map, requested);
    });
    if (nav3d::common::distance(resolved, requested) > 1e-9) {
      std::ostringstream status;
      status << (groundTraversabilityEnabled() ? "ground_endpoint_snapped" : "uav_endpoint_snapped")
             << " role=" << role
             << " requested=" << requested.x << "," << requested.y << "," << requested.z
             << " snapped=" << resolved.x << "," << resolved.y << "," << resolved.z;
      publishStatus(status.str());
      RCLCPP_INFO(get_logger(), "%s", status.str().c_str());
    }
    return resolved;
  }

  const nav3d::map::IMap& activeGlobalPlanningMap() const
  {
    if (planning_use_octomap_map_ &&
        ground_search_options_.mode == nav3d::planner::PlanningMode::Mode3D &&
        octomap_) {
      return *octomap_;
    }
    return *map_;
  }

  const char* planningMapBackendName() const
  {
    return planning_use_octomap_map_ ? "octomap" : "voxel_grid";
  }

  double planningGoalTolerance() const
  {
    return snappedGoalTolerance(activeGlobalPlanningMap().getResolution());
  }

  nav3d::planner::EgoPlanResult planWithActiveMap(
    const nav3d::common::Point3D& start,
    const nav3d::common::Point3D& goal)
  {
    pruneExpiredLocalObservations();
    auto result = makePlanningMap([&](const nav3d::map::IMap& planning_map) {
      return planner_->planWithFallbacks(planning_map, start, goal);
    });
    if (result.success &&
        groundTraversabilityEnabled() &&
        ground_search_options_.footprint_enabled) {
      const auto footprint_hit = checkTrajectoryFootprint(result.trajectory);
      if (footprint_hit.in_collision) {
        // Do not discard the geometric path: stair risers still false-positive
        // often. Publish warning + pitched markers; tracking hard-stops if the
        // body truly clips a corner.
        std::ostringstream warn;
        warn << "plan_footprint_warning";
        if (footprint_hit.first_collision_time.has_value()) {
          warn << " time=" << *footprint_hit.first_collision_time;
        }
        if (footprint_hit.first_collision_point.has_value()) {
          warn << " point=" << footprint_hit.first_collision_point->x << ","
               << footprint_hit.first_collision_point->y << ","
               << footprint_hit.first_collision_point->z;
        }
        publishStatus(warn.str());
        RCLCPP_WARN(get_logger(), "%s", warn.str().c_str());
      }
    }
    return result;
  }

  nav3d::controller::SafetyDecision evaluateSafetyWithActiveMap(
    const nav3d::common::Point3D& current_position)
  {
    pruneExpiredLocalObservations();
    return makePlanningMap([&](const nav3d::map::IMap& planning_map) {
      return safety_monitor_->evaluate(
        planning_map,
        current_position,
        *active_trajectory_,
        *active_goal_);
    });
  }

  nav3d::controller::TrajectoryCommand computeControllerCommandWithActiveMap(
    const nav3d::common::Point3D& current_position)
  {
    pruneExpiredLocalObservations();
    if (groundTraversabilityEnabled()) {
      if (!tracking_path_time_.has_value()) {
        tracking_path_time_ = 0.0;
      }
      double path_time = *tracking_path_time_;
      auto command = trajectory_tracker_->computeCommandMonotonic(
        *active_trajectory_,
        current_position,
        current_yaw_.value_or(0.0),
        *active_goal_,
        path_time,
        last_controller_command_);
      tracking_path_time_ = path_time;
      return command;
    }
    return makePlanningMap([&](const nav3d::map::IMap& planning_map) {
      return trajectory_tracker_->computeCommand(
        *active_trajectory_,
        current_position,
        current_yaw_.value_or(0.0),
        *active_goal_,
        planning_map,
        last_controller_command_);
    });
  }

  bool hasLocalObservations() const
  {
    return local_grid_ && !local_grid_->observedCells().empty();
  }

  struct LocalPointCloudFrame {
    rclcpp::Time received_at;
    std::vector<nav3d::common::Point3D> occupied_points;
    std::optional<nav3d::common::Point3D> ray_origin;
  };

  struct LookaheadTarget {
    nav3d::common::Point3D point;
    double time = 0.0;
  };

  void clearLocalObservationWindow()
  {
    local_pointcloud_frames_.clear();
    if (local_grid_) {
      local_grid_->clear();
    }
  }

  void updateLocalObservationWindow(
    const std::vector<nav3d::common::Point3D>& frame_points,
    const std::optional<nav3d::common::Point3D>& ray_origin,
    const rclcpp::Time& observation_time)
  {
    if (!local_grid_) {
      return;
    }

    if (frame_points.empty()) {
      clearLocalObservationWindow();
      return;
    }

    if (local_grid_retention_sec_ <= 0.0 || local_grid_max_retained_frames_ <= 1) {
      local_pointcloud_frames_.clear();
    }

    local_pointcloud_frames_.push_back(LocalPointCloudFrame{
      observation_time,
      frame_points,
      ray_origin,
    });
    pruneLocalObservationWindow();
    rebuildLocalGridFromObservationWindow();
  }

  void pruneLocalObservationWindow()
  {
    const rclcpp::Time prune_time = now();
    while (local_pointcloud_frames_.size() >
           static_cast<std::size_t>(local_grid_max_retained_frames_)) {
      local_pointcloud_frames_.pop_front();
    }

    if (local_grid_retention_sec_ <= 0.0) {
      return;
    }

    while (!local_pointcloud_frames_.empty() &&
           (prune_time - local_pointcloud_frames_.front().received_at).seconds() >
             local_grid_retention_sec_) {
      local_pointcloud_frames_.pop_front();
    }
  }

  void pruneExpiredLocalObservations()
  {
    if (local_grid_retention_sec_ <= 0.0 || local_pointcloud_frames_.empty()) {
      return;
    }
    const std::size_t before = local_pointcloud_frames_.size();
    pruneLocalObservationWindow();
    if (local_pointcloud_frames_.size() != before) {
      rebuildLocalGridFromObservationWindow();
    }
  }

  void rebuildLocalGridFromObservationWindow()
  {
    if (!local_grid_) {
      return;
    }

    local_grid_->clear();
    for (const auto& frame : local_pointcloud_frames_) {
      for (const auto& point : frame.occupied_points) {
        if (frame.ray_origin.has_value()) {
          local_grid_->markRayFreeAndOccupied(*frame.ray_origin, point);
        } else {
          local_grid_->markOccupied(point);
        }
      }
    }
  }

  // Pick a point along the currently-active B-spline that lies approximately
  // `lookahead_distance` past `current_position`. Returns std::nullopt when the
  // active trajectory is too short (use the real goal instead).
  std::optional<LookaheadTarget> pickLookaheadGoal(
    const nav3d::common::Point3D& current_position,
    double lookahead_distance) const
  {
    if (!active_trajectory_.has_value()) return std::nullopt;
    const auto& bspline = *active_trajectory_;
    const double duration = bspline.duration();
    if (!std::isfinite(duration) || duration <= 0.0) return std::nullopt;
    const double start_t = estimateTrajectoryTime(current_position, bspline);
    const double step = std::max(0.05, publish_sample_dt_);
    double traveled = 0.0;
    nav3d::common::Point3D prev = bspline.evaluate(start_t);
    for (double t = start_t + step; t <= duration; t += step) {
      const auto p = bspline.evaluate(t);
      traveled += nav3d::common::distance(prev, p);
      if (traveled >= lookahead_distance) return LookaheadTarget{p, t};
      prev = p;
    }
    return std::nullopt;  // remaining trajectory shorter than lookahead → use real goal
  }

  nav3d::common::Path3D sampleTrajectoryPoints(
    const nav3d::planner::UniformBspline& trajectory,
    double start_time,
    double end_time) const
  {
    nav3d::common::Path3D points;
    const double duration = trajectory.duration();
    if (duration <= 0.0 || !std::isfinite(duration)) {
      return points;
    }

    start_time = std::clamp(start_time, 0.0, duration);
    end_time = std::clamp(end_time, start_time, duration);
    points.push_back(trajectory.evaluate(start_time));
    for (double t = start_time + publish_sample_dt_; t < end_time; t += publish_sample_dt_) {
      points.push_back(trajectory.evaluate(t));
    }
    if (end_time > start_time + 1e-9) {
      points.push_back(trajectory.evaluate(end_time));
    }
    return points;
  }

  std::optional<nav3d::planner::UniformBspline> stitchLocalReplanTrajectory(
    const nav3d::planner::UniformBspline& local_trajectory,
    const nav3d::planner::UniformBspline& previous_trajectory,
    double previous_resume_time) const
  {
    auto stitched = sampleTrajectoryPoints(
      local_trajectory,
      0.0,
      local_trajectory.duration());
    auto remainder = sampleTrajectoryPoints(
      previous_trajectory,
      previous_resume_time,
      previous_trajectory.duration());
    if (stitched.empty()) {
      stitched = std::move(remainder);
    } else {
      for (const auto& point : remainder) {
        if (!stitched.empty() &&
            nav3d::common::distance(stitched.back(), point) <= map_->getResolution() * 0.25) {
          continue;
        }
        stitched.push_back(point);
      }
    }

    if (stitched.size() < 2) {
      return std::nullopt;
    }
    return nav3d::planner::UniformBspline(std::move(stitched), 1, publish_sample_dt_);
  }

  void replanFromCurrentPose(const nav3d::common::Point3D& current_position)
  {
    if (!active_goal_.has_value()) return;
    const auto now = std::chrono::steady_clock::now();
    const auto elapsed = std::chrono::duration<double>(now - last_safety_replan_at_).count();
    if (elapsed < safety_replan_cooldown_sec_) return;
    last_safety_replan_at_ = now;

    // Try a small-window local replan first to keep replan latency low.
    // active_goal_ stays untouched; the controller still tracks the real final
    // goal because handleCurrentPoseGoalReached uses active_goal_ for completion.
    const auto final_goal = *active_goal_;
    const auto previous_trajectory = active_trajectory_;
    bool local_attempted = false;
    if (safety_local_replan_enabled_) {
      const auto lookahead_goal = pickLookaheadGoal(
        current_position, safety_local_replan_lookahead_distance_);
      if (lookahead_goal.has_value()) {
        // W-B (v3.8): raw-A* feasibility probe. If A* finds no discrete corridor
        // within the lookahead window in <50 ms, we skip the expensive local
        // EGO ladder and go straight to the global replan. This is the main
        // "fast local replan" optimization the user noticed missing.
        int astar_iter = 0;
        const bool feasible = localAStarFeasible(current_position, lookahead_goal->point, astar_iter);
        if (!feasible) {
          std::ostringstream probe;
          probe << "safety_replan_local_astar_skip iterations=" << astar_iter
                << " lookahead=" << safety_local_replan_lookahead_distance_;
          publishStatus(probe.str());
          RCLCPP_INFO(get_logger(), "%s", probe.str().c_str());
        } else {
          local_attempted = true;
          const auto local = planWithActiveMap(current_position, lookahead_goal->point);
          if (local.success) {
            const auto stitched = previous_trajectory.has_value()
              ? stitchLocalReplanTrajectory(
                  local.trajectory,
                  *previous_trajectory,
                  lookahead_goal->time)
              : std::optional<nav3d::planner::UniformBspline>{};
            if (stitched.has_value()) {
              const auto stitched_collision = makePlanningMap(
                [&](const nav3d::map::IMap& planning_map) {
                  const nav3d::collision::TrajectoryChecker checker(publish_sample_dt_);
                  return checker.check(planning_map, *stitched);
                });
              if (!stitched_collision.in_collision) {
                auto stitched_plan = local;
                stitched_plan.requested_goal = final_goal;
                stitched_plan.planned_goal = final_goal;
                stitched_plan.trajectory = *stitched;
                stitched_plan.collision = stitched_collision;
                active_trajectory_ = stitched_plan.trajectory;
                active_goal_ = final_goal;
                tracking_active_ = controller_enabled_;
                tracking_path_time_ = 0.0;
                publishTrajectory(stitched_plan);
                std::ostringstream status;
                status << "safety_replan_success_local_astar_stitched poses="
                       << last_published_pose_count_
                       << " attempts=" << local.attempts
                       << " astar_iter=" << astar_iter
                       << " lookahead=" << safety_local_replan_lookahead_distance_
                       << " resume_time=" << lookahead_goal->time;
                publishStatus(status.str());
                RCLCPP_INFO(get_logger(), "%s", status.str().c_str());
                return;
              }
              std::ostringstream collision_debug;
              collision_debug << "safety_replan_local_stitch_collision falling_back_to_global";
              if (stitched_collision.first_collision_time.has_value()) {
                collision_debug << " first_collision_time="
                                << *stitched_collision.first_collision_time;
              }
              publishStatus(collision_debug.str());
              RCLCPP_WARN(get_logger(), "%s", collision_debug.str().c_str());
            } else {
              publishStatus("safety_replan_local_stitch_failed falling_back_to_global");
              RCLCPP_WARN(get_logger(), "safety_replan_local_stitch_failed falling_back_to_global");
            }
          } else {
            std::ostringstream debug;
            debug << "safety_replan_local_failed status=" << statusToString(local.status)
                  << " astar_iter=" << astar_iter
                  << " falling_back_to_global";
            publishStatus(debug.str());
            RCLCPP_WARN(get_logger(), "%s", debug.str().c_str());
          }
        }
      }
    }

    // Global fallback: original behavior, replan all the way to final goal.
    const auto result = planWithActiveMap(current_position, final_goal);
    if (result.success) {
      active_trajectory_ = result.trajectory;
      active_goal_ = result.planned_goal;
      updatePendingNavigatePlannedGoal(result.planned_goal);
      tracking_active_ = controller_enabled_;
      tracking_path_time_ = 0.0;
      publishTrajectory(result);
      std::ostringstream status;
      status << "safety_replan_success poses=" << last_published_pose_count_
             << " attempts=" << result.attempts
             << " local_attempted=" << (local_attempted ? "true" : "false");
      appendPlanEndpointStatus(status, result, planningGoalTolerance());
      publishStatus(status.str());
      RCLCPP_INFO(get_logger(), "%s", status.str().c_str());
      return;
    }

    std::ostringstream status;
    status << "safety_replan_failed status=" << statusToString(result.status)
           << " attempts=" << result.attempts;
    publishStatus(status.str());
    RCLCPP_WARN(get_logger(), "%s", status.str().c_str());
    tracking_active_ = false;
    active_trajectory_.reset();
    active_goal_.reset();
    clearPublishedTrajectory();
    abortPendingNavigateGoal("navigate_aborted_safety_replan_failed");
    publishZeroCommand("safety_replan_emergency_stop");
  }

  double estimateTrajectoryTime(
    const nav3d::common::Point3D& current_position,
    const nav3d::planner::UniformBspline& trajectory,
    std::optional<double> hint_time = std::nullopt) const
  {
    const double duration = trajectory.duration();
    if (duration <= 0.0 || !std::isfinite(duration)) {
      return 0.0;
    }

    // Full-spline scans every control tick (~10 Hz) starve cmd_vel down to
    // ~1 Hz on long ground stair plans. Prefer a local window around the
    // monotonic path time / last estimate.
    const double step = std::max(0.05, publish_sample_dt_);
    double search_begin = 0.0;
    double search_end = duration;
    if (hint_time.has_value() && std::isfinite(*hint_time)) {
      const double hint = std::clamp(*hint_time, 0.0, duration);
      constexpr double kWindowSec = 3.0;
      search_begin = std::max(0.0, hint - 0.5);
      search_end = std::min(duration, hint + kWindowSec);
    }

    double best_time = search_begin;
    double best_distance = std::numeric_limits<double>::infinity();
    for (double t = search_begin; t <= search_end + 1e-12; t += step) {
      const double sample_t = std::min(t, duration);
      const double candidate_distance =
        nav3d::common::distance(current_position, trajectory.evaluate(sample_t));
      if (candidate_distance < best_distance) {
        best_distance = candidate_distance;
        best_time = sample_t;
      }
    }
    const double end_distance =
      nav3d::common::distance(current_position, trajectory.evaluate(search_end));
    if (end_distance < best_distance) {
      best_time = search_end;
    }
    return best_time;
  }

  void onControllerTimer()
  {
    if (!controller_enabled_ || !tracking_active_ ||
        !active_trajectory_.has_value() || !active_goal_.has_value() ||
        !current_position_.has_value() || !trajectory_tracker_) {
      return;
    }

    const auto command = computeControllerCommandWithActiveMap(*current_position_);
    if (command.goal_reached) {
      tracking_active_ = false;
      active_trajectory_.reset();
      active_goal_.reset();
      last_controller_command_.reset();
      clearPublishedTrajectory();
      publishZeroCommand("tracking_goal_reached");
      completeNavigateGoalReached();
      return;
    }

    // If the simulated pose has drifted far from the active spline (common when
    // body-frame tracking runs away past the map edge), stop instead of
    // continuing to publish max-speed cmd_vel with no visible path on the UI.
    {
      const double nearest_t = estimateTrajectoryTime(
        *current_position_,
        *active_trajectory_,
        tracking_path_time_);
      const double off_path = nav3d::common::distance(
        *current_position_, active_trajectory_->evaluate(nearest_t));
      constexpr double kMaxOffPathMeters = 4.0;
      if (off_path > kMaxOffPathMeters) {
        tracking_active_ = false;
        last_controller_command_.reset();
        publishZeroCommand("tracking_lost_path");
        std::ostringstream status;
        status << "tracking_lost_path off_path_m=" << off_path
               << " limit_m=" << kMaxOffPathMeters;
        publishStatus(status.str());
        RCLCPP_WARN(get_logger(), "%s", status.str().c_str());
        return;
      }
    }

    if (groundTraversabilityEnabled() && ground_search_options_.footprint_enabled) {
      publishCurrentFootprintMarker();
      const bool footprint_hit = makeCollisionPlanningMap(
        [&](const nav3d::map::IMap& occupancy_map) {
          const nav3d::collision::Footprint3DChecker checker(ground_search_options_.footprint);
          double yaw = current_yaw_.value_or(0.0);
          double path_dz = 0.0;
          double path_ds = 0.0;
          if (active_trajectory_.has_value()) {
            const double duration = active_trajectory_->duration();
            const double t = std::clamp(tracking_path_time_.value_or(0.0), 0.0, duration);
            const double t0 = std::max(0.0, t - 0.5);
            const double t1 = std::min(duration, t + 0.5);
            const auto a = active_trajectory_->evaluate(t0);
            const auto b = active_trajectory_->evaluate(t1);
            path_dz = b.z - a.z;
            path_ds = std::hypot(b.x - a.x, b.y - a.y);
            if (path_ds > 1e-4) {
              yaw = std::atan2(b.y - a.y, b.x - a.x);
            }
          }
          return checker
            .checkAtPose(
              occupancy_map,
              *current_position_,
              yaw,
              imuAttitudeOverride(),
              path_dz,
              path_ds)
            .in_collision;
        });
      if (footprint_hit) {
        last_controller_command_.reset();
        publishZeroCommand("tracking_footprint_collision");
        publishStatus("tracking_footprint_collision");
        const auto& p = *current_position_;
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          1000,
          "tracking_footprint_collision: holding cmd_vel pose=%.3f,%.3f,%.3f path_t=%.2f",
          p.x,
          p.y,
          p.z,
          tracking_path_time_.value_or(-1.0));
        return;
      }
    }

    // Optional hard block against commanding into raw occupied voxels.
    // Off by default for ground mode: stair B-spline chords clip risers, and
    // zeroing cmd_vel there leaves the robot stuck at the stair mouth.
    if (controller_block_occupied_voxels_) {
      const double dt = trajectory_tracker_->config().control_dt;
      nav3d::common::Point3D map_velocity = command.linear;
      if (trajectory_tracker_->config().command_frame ==
          nav3d::controller::CommandFrame::Body) {
        const double yaw = current_yaw_.value_or(0.0);
        const double c = std::cos(yaw);
        const double s = std::sin(yaw);
        map_velocity = {
          command.linear.x * c - command.linear.y * s,
          command.linear.x * s + command.linear.y * c,
          command.linear.z,
        };
      }
      const auto projected = *current_position_ + map_velocity * dt;
      const bool hits_occupied = makeCollisionPlanningMap(
        [&](const nav3d::map::IMap& occupancy_map) {
          const double step = std::min(
            occupancy_map.getResolution() * 0.5,
            trajectory_tracker_->config().command_path_sample_step);
          const double length = nav3d::common::distance(*current_position_, projected);
          const int samples =
            std::max(1, static_cast<int>(std::ceil(length / std::max(step, 1e-6))));
          for (int i = 1; i <= samples; ++i) {
            const double ratio = static_cast<double>(i) / static_cast<double>(samples);
            const auto p = *current_position_ + (projected - *current_position_) * ratio;
            if (occupancy_map.isOccupied(p)) {
              return true;
            }
          }
          return false;
        });
      if (hits_occupied) {
        last_controller_command_.reset();
        publishZeroCommand("controller_blocked_occupied_voxel");
        return;
      }
    }

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = command.linear.x;
    cmd.linear.y = command.linear.y;
    cmd.linear.z = command.linear.z;
    cmd.angular.z = command.angular_z;
    cmd_vel_pub_->publish(cmd);
    last_controller_command_ = command;
  }

  nav_msgs::msg::Path sampleTrajectoryPath(
    const nav3d::planner::UniformBspline& trajectory,
    double start_time = 0.0)
  {
    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = frame_id_;

    const double duration = trajectory.duration();
    if (duration <= 0.0 || !std::isfinite(duration)) {
      return path;
    }

    start_time = std::clamp(start_time, 0.0, duration);
    // Cap published poses so rosbridge / Web UI / RViz can show the path.
    // Dense sample_dt is still used internally for controller checks.
    double sample_dt = publish_sample_dt_;
    const double span = std::max(duration - start_time, sample_dt);
    const int expected =
      static_cast<int>(std::floor(span / sample_dt)) + 2;
    if (expected > trajectory_publish_max_poses_) {
      sample_dt = span / static_cast<double>(trajectory_publish_max_poses_ - 1);
    }
    for (double t = start_time; t < duration; t += sample_dt) {
      path.poses.push_back(toPoseStamped(trajectory.evaluate(t), path.header));
    }
    path.poses.push_back(toPoseStamped(trajectory.evaluate(duration), path.header));
    return path;
  }

  visualization_msgs::msg::Marker makeTrajectoryMarker(
    const nav_msgs::msg::Path& path) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header = path.header;
    marker.ns = "nav3d_trajectory";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = path.poses.empty() ?
      visualization_msgs::msg::Marker::DELETE :
      visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.18;
    marker.color.r = 0.55F;
    marker.color.g = 0.18F;
    marker.color.b = 0.95F;
    marker.color.a = 1.0F;
    marker.points.reserve(path.poses.size());
    for (const auto& pose : path.poses) {
      marker.points.push_back(pose.pose.position);
    }
    return marker;
  }

  void publishTrajectoryMarker(const nav_msgs::msg::Path& path)
  {
    trajectory_marker_pub_->publish(makeTrajectoryMarker(path));
  }

  void publishEndpointMarker(
    const nav3d::common::Point3D& point,
    const std::string& ns,
    rclcpp::Publisher<visualization_msgs::msg::Marker>& publisher,
    float red,
    float green,
    float blue) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = now();
    marker.header.frame_id = frame_id_;
    marker.ns = ns;
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = point.x;
    marker.pose.position.y = point.y;
    marker.pose.position.z = point.z;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.45;
    marker.scale.y = 0.45;
    marker.scale.z = 0.45;
    marker.color.r = red;
    marker.color.g = green;
    marker.color.b = blue;
    marker.color.a = 1.0F;
    publisher.publish(marker);
  }

  void clearEndpointMarker(
    const std::string& ns,
    rclcpp::Publisher<visualization_msgs::msg::Marker>& publisher) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = now();
    marker.header.frame_id = frame_id_;
    marker.ns = ns;
    marker.id = 0;
    marker.action = visualization_msgs::msg::Marker::DELETE;
    publisher.publish(marker);
  }

  void publishTrajectory(const nav3d::planner::EgoPlanResult& result)
  {
    const auto path = sampleTrajectoryPath(result.trajectory);
    last_published_pose_count_ = path.poses.size();
    trajectory_pub_->publish(path);
    publishTrajectoryMarker(path);
    publishFootprintMarkerAlongTrajectory(result.trajectory);
  }

  std::optional<nav3d::collision::BodyAttitude> imuAttitudeOverride() const
  {
    if (!imu_use_for_body_attitude_ || !imu_attitude_.has_value() || !imu_attitude_->from_imu) {
      return std::nullopt;
    }
    return imu_attitude_;
  }

  void onImu(const sensor_msgs::msg::Imu& msg)
  {
    const auto& q = msg.orientation;
    const double norm_sq = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    if (norm_sq < 1e-12 || !std::isfinite(norm_sq)) {
      return;
    }
    // ZYX yaw-pitch-roll from quaternion.
    const double sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z);
    const double cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    const double sinp = 2.0 * (q.w * q.y - q.z * q.x);
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);

    nav3d::collision::BodyAttitude attitude;
    attitude.from_imu = true;
    attitude.roll = std::atan2(sinr_cosp, cosr_cosp);
    attitude.pitch = std::abs(sinp) >= 1.0 ? std::copysign(M_PI / 2.0, sinp) : std::asin(sinp);
    attitude.yaw = std::atan2(siny_cosp, cosy_cosp);
    imu_attitude_ = attitude;
  }

  nav3d::collision::TrajectoryCollisionResult checkTrajectoryFootprint(
    const nav3d::planner::UniformBspline& trajectory) const
  {
    nav3d::collision::TrajectoryCollisionResult result;
    if (!ground_search_options_.footprint_enabled) {
      return result;
    }

    const double duration = trajectory.duration();
    if (duration <= 0.0 || !std::isfinite(duration)) {
      return result;
    }

    const nav3d::collision::Footprint3DChecker checker(ground_search_options_.footprint);
    const auto imu = imuAttitudeOverride();
    return makeCollisionPlanningMap([&](const nav3d::map::IMap& occupancy_map) {
      nav3d::collision::TrajectoryCollisionResult local;
      const double dt = std::max(0.08, publish_sample_dt_ * 2.0);
      // Window ~ half wheelbase along the path for stair-parallel pitch.
      const double pitch_window = 0.5;
      auto attitudeAt = [&](double t, double yaw_hint) {
        const double t0 = std::max(0.0, t - pitch_window);
        const double t1 = std::min(duration, t + pitch_window);
        const auto a = trajectory.evaluate(t0);
        const auto b = trajectory.evaluate(t1);
        const double dx = b.x - a.x;
        const double dy = b.y - a.y;
        const double ds = std::hypot(dx, dy);
        double yaw = yaw_hint;
        if (ds > 1e-4) {
          yaw = std::atan2(dy, dx);
        }
        const auto point = trajectory.evaluate(t);
        return nav3d::collision::estimateBodyAttitude(
          occupancy_map,
          point,
          yaw,
          b.z - a.z,
          ds,
          ground_search_options_.footprint,
          imu);
      };

      auto yawAt = [&](double t) {
        const double t1 = std::min(duration, t + dt);
        const auto p0 = trajectory.evaluate(t);
        const auto p1 = trajectory.evaluate(t1);
        const double ds = std::hypot(p1.x - p0.x, p1.y - p0.y);
        if (ds < 1e-4) {
          return 0.0;
        }
        return std::atan2(p1.y - p0.y, p1.x - p0.x);
      };

      double prev_yaw = yawAt(0.0);
      for (double t = 0.0; t <= duration + 1e-9; t += dt) {
        const double tc = std::min(t, duration);
        const auto point = trajectory.evaluate(tc);
        const double yaw = yawAt(tc);
        const auto attitude = attitudeAt(tc, yaw);
        const auto hit = checker.check(occupancy_map, point, attitude);
        if (hit.in_collision) {
          local.in_collision = true;
          local.first_collision_time = tc;
          local.first_collision_point = hit.hit_point;
          return local;
        }

        // Stair landings / corners: sweep intermediate yaw while turning so the
        // long body does not cut the inside of the stairwell.
        double dyaw = yaw - prev_yaw;
        while (dyaw > M_PI) {
          dyaw -= 2.0 * M_PI;
        }
        while (dyaw < -M_PI) {
          dyaw += 2.0 * M_PI;
        }
        const int yaw_samples = static_cast<int>(std::ceil(std::abs(dyaw) / (15.0 * M_PI / 180.0)));
        for (int i = 1; i < yaw_samples; ++i) {
          const double alpha = static_cast<double>(i) / static_cast<double>(yaw_samples);
          nav3d::collision::BodyAttitude turned = attitude;
          turned.yaw = prev_yaw + alpha * dyaw;
          const auto turn_hit = checker.check(occupancy_map, point, turned);
          if (turn_hit.in_collision) {
            local.in_collision = true;
            local.first_collision_time = tc;
            local.first_collision_point = turn_hit.hit_point;
            return local;
          }
        }
        prev_yaw = yaw;
      }
      return local;
    });
  }

  void publishFootprintMarkerAlongTrajectory(
    const nav3d::planner::UniformBspline& trajectory)
  {
    if (!footprint_marker_pub_ || !ground_search_options_.footprint_enabled) {
      return;
    }

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = now();
    marker.ns = "nav3d_footprint";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.03;
    marker.color.r = 0.1F;
    marker.color.g = 0.85F;
    marker.color.b = 0.35F;
    marker.color.a = 0.95F;

    const double duration = trajectory.duration();
    if (duration <= 0.0) {
      marker.action = visualization_msgs::msg::Marker::DELETE;
      footprint_marker_pub_->publish(marker);
      return;
    }

    makeCollisionPlanningMap([&](const nav3d::map::IMap& occupancy_map) {
      const double sample_dt = std::max(0.15, duration / 16.0);
      const double pitch_window = 0.5;
      for (double t = 0.0; t <= duration + 1e-9; t += sample_dt) {
        const double tc = std::min(t, duration);
        const auto point = trajectory.evaluate(tc);
        const double t0 = std::max(0.0, tc - pitch_window);
        const double t1 = std::min(duration, tc + pitch_window);
        const auto a = trajectory.evaluate(t0);
        const auto b = trajectory.evaluate(t1);
        const double ds = std::hypot(b.x - a.x, b.y - a.y);
        double yaw = 0.0;
        if (ds > 1e-4) {
          yaw = std::atan2(b.y - a.y, b.x - a.x);
        }
        const auto attitude = nav3d::collision::estimateBodyAttitude(
          occupancy_map,
          point,
          yaw,
          b.z - a.z,
          ds,
          ground_search_options_.footprint,
          imuAttitudeOverride());
        const auto box = nav3d::collision::makeOrientedBodyBox(
          occupancy_map,
          point,
          attitude,
          ground_search_options_.footprint);
        const auto corners = nav3d::collision::bodyBoxWireframeCorners(box);
        const std::size_t n = ground_search_options_.footprint.polygon.size();
        if (corners.size() < 2 * n) {
          continue;
        }
        auto push_edge = [&](const nav3d::common::Point3D& a_pt, const nav3d::common::Point3D& b_pt) {
          geometry_msgs::msg::Point pa;
          pa.x = a_pt.x;
          pa.y = a_pt.y;
          pa.z = a_pt.z;
          geometry_msgs::msg::Point pb;
          pb.x = b_pt.x;
          pb.y = b_pt.y;
          pb.z = b_pt.z;
          marker.points.push_back(pa);
          marker.points.push_back(pb);
        };
        for (std::size_t i = 0; i < n; ++i) {
          push_edge(corners[i], corners[(i + 1) % n]);
          push_edge(corners[n + i], corners[n + ((i + 1) % n)]);
          push_edge(corners[i], corners[n + i]);
        }
      }
      return 0;
    });
    footprint_marker_pub_->publish(marker);
  }

  void publishCurrentFootprintMarker()
  {
    if (!footprint_marker_pub_ ||
        !ground_search_options_.footprint_enabled ||
        !current_position_.has_value()) {
      return;
    }

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = now();
    marker.ns = "nav3d_footprint_current";
    marker.id = 1;
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.04;
    marker.color.r = 0.95F;
    marker.color.g = 0.55F;
    marker.color.b = 0.1F;
    marker.color.a = 1.0F;

    makeCollisionPlanningMap([&](const nav3d::map::IMap& occupancy_map) {
      double yaw = current_yaw_.value_or(0.0);
      double path_dz = 0.0;
      double path_ds = 0.0;
      if (active_trajectory_.has_value()) {
        const double duration = active_trajectory_->duration();
        const double t = std::clamp(tracking_path_time_.value_or(0.0), 0.0, duration);
        const double t0 = std::max(0.0, t - 0.5);
        const double t1 = std::min(duration, t + 0.5);
        const auto a = active_trajectory_->evaluate(t0);
        const auto b = active_trajectory_->evaluate(t1);
        path_dz = b.z - a.z;
        path_ds = std::hypot(b.x - a.x, b.y - a.y);
        if (path_ds > 1e-4) {
          yaw = std::atan2(b.y - a.y, b.x - a.x);
        }
      }
      const auto attitude = nav3d::collision::estimateBodyAttitude(
        occupancy_map,
        *current_position_,
        yaw,
        path_dz,
        path_ds,
        ground_search_options_.footprint,
        imuAttitudeOverride());
      RCLCPP_DEBUG_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "footprint_attitude yaw=%.3f pitch=%.3f path_pitch=%.3f",
        attitude.yaw,
        attitude.pitch,
        path_ds > 1e-6 ? std::atan2(path_dz, path_ds) : 0.0);
      const auto box = nav3d::collision::makeOrientedBodyBox(
        occupancy_map,
        *current_position_,
        attitude,
        ground_search_options_.footprint);
      const auto corners = nav3d::collision::bodyBoxWireframeCorners(box);
      const std::size_t n = ground_search_options_.footprint.polygon.size();
      auto push_edge = [&](const nav3d::common::Point3D& a, const nav3d::common::Point3D& b) {
        geometry_msgs::msg::Point pa;
        pa.x = a.x;
        pa.y = a.y;
        pa.z = a.z;
        geometry_msgs::msg::Point pb;
        pb.x = b.x;
        pb.y = b.y;
        pb.z = b.z;
        marker.points.push_back(pa);
        marker.points.push_back(pb);
      };
      for (std::size_t i = 0; i < n && corners.size() >= 2 * n; ++i) {
        push_edge(corners[i], corners[(i + 1) % n]);
        push_edge(corners[n + i], corners[n + ((i + 1) % n)]);
        push_edge(corners[i], corners[n + i]);
      }
      return 0;
    });
    footprint_marker_pub_->publish(marker);
  }

  // W-B (v3.8): raw-A* feasibility probe. Used by replanFromCurrentPose to
  // decide *whether to attempt a local replan at all* before paying for the
  // full EGO ladder + B-spline optimization. AStar3D::search runs in tight
  // bounds (max_iterations=20000, ~6 m lookahead) and returns Success/NoPath
  // in milliseconds. Outcomes:
  //   - Success → call planWithActiveMap(current, lookahead_goal). A* having
  //     succeeded means a feasible discrete corridor exists, so the EGO
  //     fallback ladder usually converges on the first attempt instead of
  //     iterating through kinodynamic search + JPS + spline trials.
  //   - NoPath → skip the local stage entirely; fall straight through to the
  //     global replan. Avoids wasting ~50-150 ms on a doomed local attempt.
  // The probe itself does NOT touch active_trajectory_ — controller tracking
  // is uninterrupted while we decide; the existing spline keeps the cmd_vel
  // stream alive until the actual replan publishes a new trajectory.
  bool localAStarFeasible(
    const nav3d::common::Point3D& start,
    const nav3d::common::Point3D& goal,
    int& iterations_out) const
  {
    iterations_out = 0;
    if (!safety_local_use_raw_astar_) return true;  // probe disabled → always attempt local
    nav3d::planner::SearchOptions opts;
    opts.algorithm = nav3d::planner::SearchAlgorithm::AStar;
    opts.mode = ground_search_options_.mode;
    opts.allow_diagonal = true;
    opts.max_iterations = std::max(1000, safety_local_astar_max_iterations_);
    auto raw = makePlanningMap(
      [&](const nav3d::map::IMap& planning_map) {
        return local_astar_.search(planning_map, start, goal, opts);
      });
    iterations_out = raw.iterations;
    return raw.status == nav3d::planner::SearchStatus::Success && !raw.path.empty();
  }

  bool handleCurrentPoseGoalReached(const nav3d::common::Point3D& current_position)
  {
    if (!active_goal_.has_value()) {
      return false;
    }
    if (nav3d::common::distance(current_position, *active_goal_) > controller_goal_tolerance_) {
      return false;
    }

    tracking_active_ = false;
    active_trajectory_.reset();
    active_goal_.reset();
    last_controller_command_.reset();
    clearPublishedTrajectory();
    publishZeroCommand("tracking_goal_reached");
    completeNavigateGoalReached();
    return true;
  }

  void publishRemainingTrajectory(const nav3d::common::Point3D& current_position)
  {
    if (!active_trajectory_.has_value()) {
      return;
    }
    const auto now_tp = std::chrono::steady_clock::now();
    constexpr double kMinRepublishSec = 1.0;
    if (last_remaining_trajectory_publish_.has_value()) {
      const double dt = std::chrono::duration<double>(
                          now_tp - *last_remaining_trajectory_publish_)
                          .count();
      if (dt < kMinRepublishSec) {
        return;
      }
    }
    last_remaining_trajectory_publish_ = now_tp;

    const double start_time = estimateTrajectoryTime(
      current_position,
      *active_trajectory_,
      tracking_path_time_);
    const auto path = sampleTrajectoryPath(*active_trajectory_, start_time);
    if (path.poses.size() <= 2) {
      clearPublishedTrajectory();
      return;
    }
    last_published_pose_count_ = path.poses.size();
    trajectory_pub_->publish(path);
    publishTrajectoryMarker(path);
  }

  void clearPublishedTrajectory()
  {
    tracking_path_time_.reset();
    last_remaining_trajectory_publish_.reset();
    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = frame_id_;
    last_published_pose_count_ = 0;
    trajectory_pub_->publish(path);
    publishTrajectoryMarker(path);
  }

  geometry_msgs::msg::PoseStamped toPoseStamped(
    const nav3d::common::Point3D& point,
    const std_msgs::msg::Header& header) const
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = header;
    pose.pose.position.x = point.x;
    pose.pose.position.y = point.y;
    pose.pose.position.z = point.z;
    pose.pose.orientation.w = 1.0;
    return pose;
  }

  void publishStatus(const std::string& text)
  {
    std_msgs::msg::String msg;
    msg.data = text;
    status_pub_->publish(msg);
  }

  void publishZeroCommand(const std::string& reason)
  {
    geometry_msgs::msg::Twist cmd;
    cmd_vel_pub_->publish(cmd);
    last_controller_command_.reset();
    publishStatus(reason);
    RCLCPP_WARN(get_logger(), "%s: published zero cmd_vel", reason.c_str());
  }

  bool hasPendingNavigateGoal()
  {
    std::lock_guard<std::mutex> lock(pending_navigate_mutex_);
    return pending_navigate_goal_.has_value() &&
           pending_navigate_goal_->handle &&
           pending_navigate_goal_->handle->is_active();
  }

  struct PendingNavigateGoal {
    std::shared_ptr<NavigateGoalHandle> handle;
    nav3d::common::Point3D requested_goal;
    nav3d::common::Point3D planned_goal;
  };

  void setPendingNavigateGoal(
    const std::shared_ptr<NavigateGoalHandle>& goal_handle,
    const nav3d::common::Point3D& requested_goal,
    const nav3d::common::Point3D& planned_goal)
  {
    std::lock_guard<std::mutex> lock(pending_navigate_mutex_);
    pending_navigate_goal_ = PendingNavigateGoal{
      goal_handle,
      requested_goal,
      planned_goal,
    };
  }

  void updatePendingNavigatePlannedGoal(const nav3d::common::Point3D& planned_goal)
  {
    std::lock_guard<std::mutex> lock(pending_navigate_mutex_);
    if (pending_navigate_goal_.has_value()) {
      pending_navigate_goal_->planned_goal = planned_goal;
    }
  }

  void completeNavigateGoalReached()
  {
    std::optional<PendingNavigateGoal> pending_goal;
    {
      std::lock_guard<std::mutex> lock(pending_navigate_mutex_);
      pending_goal = pending_navigate_goal_;
      pending_navigate_goal_.reset();
    }
    if (!pending_goal.has_value() ||
        !pending_goal->handle ||
        !pending_goal->handle->is_active()) {
      return;
    }

    auto result = std::make_shared<NavigateToPose::Result>();
    if (nav3d::common::distance(
          pending_goal->requested_goal,
          pending_goal->planned_goal) > planningGoalTolerance()) {
      pending_goal->handle->abort(result);
      std::ostringstream status;
      status << "navigate_partial_goal_reached planned_goal="
             << pending_goal->planned_goal.x << ","
             << pending_goal->planned_goal.y << ","
             << pending_goal->planned_goal.z
             << " requested_goal="
             << pending_goal->requested_goal.x << ","
             << pending_goal->requested_goal.y << ","
             << pending_goal->requested_goal.z;
      publishStatus(status.str());
      RCLCPP_WARN(get_logger(), "%s", status.str().c_str());
      return;
    }

    pending_goal->handle->succeed(result);
    publishStatus("navigate_goal_reached");
    RCLCPP_INFO(get_logger(), "navigate_goal_reached");
  }

  void abortPendingNavigateGoal(const std::string& reason)
  {
    std::shared_ptr<NavigateGoalHandle> goal_handle;
    {
      std::lock_guard<std::mutex> lock(pending_navigate_mutex_);
      if (pending_navigate_goal_.has_value()) {
        goal_handle = pending_navigate_goal_->handle;
      }
      pending_navigate_goal_.reset();
    }
    if (!goal_handle || !goal_handle->is_active()) {
      return;
    }

    auto result = std::make_shared<NavigateToPose::Result>();
    goal_handle->abort(result);
    publishStatus(reason);
    RCLCPP_WARN(get_logger(), "%s", reason.c_str());
  }

  void cancelPendingNavigateGoal(
    const std::shared_ptr<NavigateGoalHandle>& goal_handle,
    const std::string& reason)
  {
    bool should_cancel = false;
    {
      std::lock_guard<std::mutex> lock(pending_navigate_mutex_);
      should_cancel =
        pending_navigate_goal_.has_value() &&
        pending_navigate_goal_->handle == goal_handle;
      if (should_cancel) {
        pending_navigate_goal_.reset();
      }
    }
    if (!should_cancel || !goal_handle->is_active()) {
      return;
    }

    auto result = std::make_shared<NavigateToPose::Result>();
    goal_handle->canceled(result);
    publishStatus(reason);
    RCLCPP_INFO(get_logger(), "%s", reason.c_str());
  }

  std::string frame_id_;
  double publish_sample_dt_ = 0.05;
  int trajectory_publish_max_poses_ = 600;
  bool local_grid_ray_clearing_enabled_ = false;
  bool local_grid_enabled_ = false;
  double local_grid_resolution_ = 0.5;
  double local_grid_retention_sec_ = 0.0;
  int local_grid_max_retained_frames_ = 1;
  double local_grid_max_observation_age_sec_ = 2.0;
  double local_grid_max_future_stamp_sec_ = 0.2;
  bool controller_enabled_ = false;
  bool controller_block_occupied_voxels_ = true;
  double safety_replan_cooldown_sec_ = 0.15;
  std::chrono::steady_clock::time_point last_safety_replan_at_{};
  double safety_local_replan_lookahead_distance_ = 6.0;
  bool safety_local_replan_enabled_ = true;
  // W-B (v3.8): raw A* fast path inside the local-replan window. When enabled,
  // replanFromCurrentPose first calls AStar3D::search on the active planning
  // map for a small lookahead goal and publishes the path directly as a
  // nav_msgs::Path, skipping the full EGO ladder (kinodynamic + JPS + spline
  // smoothing). Bypasses ~30-100 ms of optimization per replan, restoring the
  // promised "fast local replan" behaviour. Falls back to planWithActiveMap
  // when raw A* fails or returns < min_poses.
  bool safety_local_use_raw_astar_ = true;
  int safety_local_astar_max_iterations_ = 20000;
  nav3d::planner::AStar3D local_astar_;
  double controller_goal_tolerance_ = 0.15;
  bool tracking_active_ = false;
  std::optional<double> tracking_path_time_;
  std::optional<std::chrono::steady_clock::time_point> last_remaining_trajectory_publish_;
  std::atomic<std::uint64_t> plan_epoch_{0};
  rclcpp::CallbackGroup::SharedPtr planning_callback_group_;
  rclcpp::CallbackGroup::SharedPtr control_callback_group_;
  double controller_control_dt_ = 0.1;
  int occupancy_grid_max_cells_ = 1000000;
  double occupancy_grid_min_z_ = 0.2;
  double occupancy_grid_max_z_ = 0.9;
  int uav_endpoint_snap_radius_cells_ = 12;
  double planning_inflation_radius_ = 0.0;
  std::size_t last_published_pose_count_ = 0;
  bool next_clicked_point_is_start_ = true;
  nav3d::map::MapBuildConfig map_build_config_;
  PcdLoaderBackend pcd_loader_backend_ = PcdLoaderBackend::Builtin;
  PlanningTraversabilityMode planning_traversability_mode_ = PlanningTraversabilityMode::Uav;
  bool planning_use_octomap_map_ = false;
  GroundSearchOptions ground_search_options_;
  std::string imu_topic_ = "/imu";
  bool imu_use_for_body_attitude_ = false;
  std::optional<nav3d::collision::BodyAttitude> imu_attitude_;
  nav3d::common::Point3D start_;
  std::optional<nav3d::common::Point3D> current_position_;
  std::optional<double> current_yaw_;
  std::optional<nav3d::common::Point3D> active_goal_;
  std::optional<nav3d::planner::UniformBspline> active_trajectory_;
  std::optional<nav3d::controller::TrajectoryCommand> last_controller_command_;
  std::unique_ptr<nav3d::map::VoxelGridMap> map_;
  std::unique_ptr<nav3d::map::OctomapManager> octomap_;
  std::unique_ptr<nav3d::map::LocalGrid> local_grid_;
  std::deque<LocalPointCloudFrame> local_pointcloud_frames_;
  std::unique_ptr<nav3d::planner::EgoPlannerCore> planner_;
  std::unique_ptr<nav3d::controller::SafetyMonitor> safety_monitor_;
  std::unique_ptr<nav3d::controller::TrajectoryTracker> trajectory_tracker_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr trajectory_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr start_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr footprint_marker_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupied_grid_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr planning_occupied_markers_pub_;
#ifdef NAV3D_HAS_OCTOMAP_MSGS
  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octomap_pub_;
#endif
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_set_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr start_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_point_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr cancel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr local_pointcloud_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr map_load_sub_;
  rclcpp::Service<GetPlan>::SharedPtr plan_service_;
  rclcpp::TimerBase::SharedPtr controller_timer_;
  rclcpp::TimerBase::SharedPtr static_visualization_timer_;
  int static_visualization_republish_count_ = 0;
  rclcpp_action::Server<NavigateToPose>::SharedPtr navigate_action_server_;
  std::mutex pending_navigate_mutex_;
  std::optional<PendingNavigateGoal> pending_navigate_goal_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<Nav3DBridgeNode>();
    // Two threads: one can keep planning while cancel/current_pose/controller run.
    rclcpp::executors::MultiThreadedExecutor executor(
      rclcpp::ExecutorOptions(),
      4);
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception& error) {
    RCLCPP_FATAL(rclcpp::get_logger("nav3d_bridge_node"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}

#include "nav3d/collision/footprint_3d.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <unordered_map>
#include <utility>

namespace nav3d::collision {
namespace {

common::Point3D rotateBodyToWorld(
  double bx,
  double by,
  double bz,
  double yaw,
  double pitch,
  double roll)
{
  const double cr = std::cos(roll);
  const double sr = std::sin(roll);
  const double cp = std::cos(pitch);
  const double sp = std::sin(pitch);
  const double cy = std::cos(yaw);
  const double sy = std::sin(yaw);

  // R = Rz(yaw) * Ry_nose_up(pitch) * Rx(roll).
  // Positive pitch raises body +X (nose) toward world +Z so the belly stays
  // parallel to an uphill stair/ramp (ROS FLU / robotics convention).
  const double x1 = bx;
  const double y1 = cr * by - sr * bz;
  const double z1 = sr * by + cr * bz;

  const double x2 = cp * x1 - sp * z1;
  const double y2 = y1;
  const double z2 = sp * x1 + cp * z1;

  return {
    cy * x2 - sy * y2,
    sy * x2 + cy * y2,
    z2,
  };
}

common::Point3D worldToBody(
  const common::Point3D& world,
  const common::Point3D& origin,
  double yaw,
  double pitch,
  double roll)
{
  const double dx = world.x - origin.x;
  const double dy = world.y - origin.y;
  const double dz = world.z - origin.z;

  const double cr = std::cos(roll);
  const double sr = std::sin(roll);
  const double cp = std::cos(pitch);
  const double sp = std::sin(pitch);
  const double cy = std::cos(yaw);
  const double sy = std::sin(yaw);

  // Inverse of Rz * Ry_nose_up * Rx: Rx(-r) * Ry_nose_up(-p) * Rz(-y)
  const double x1 = cy * dx + sy * dy;
  const double y1 = -sy * dx + cy * dy;
  const double z1 = dz;

  const double x2 = cp * x1 + sp * z1;
  const double y2 = y1;
  const double z2 = -sp * x1 + cp * z1;

  return {
    x2,
    cr * y2 + sr * z2,
    -sr * y2 + cr * z2,
  };
}

bool pointInPolygon2D(double x, double y, const std::vector<FootprintPoint2D>& polygon)
{
  bool inside = false;
  const std::size_t n = polygon.size();
  for (std::size_t i = 0, j = n - 1; i < n; j = i++) {
    const double xi = polygon[i].x;
    const double yi = polygon[i].y;
    const double xj = polygon[j].x;
    const double yj = polygon[j].y;
    const bool intersect =
      ((yi > y) != (yj > y)) &&
      (x < (xj - xi) * (y - yi) / ((yj - yi) + 1e-18) + xi);
    if (intersect) {
      inside = !inside;
    }
  }
  return inside;
}

void polygonExtents(
  const std::vector<FootprintPoint2D>& polygon,
  double& x_min,
  double& x_max,
  double& y_min,
  double& y_max)
{
  x_min = polygon.front().x;
  x_max = polygon.front().x;
  y_min = polygon.front().y;
  y_max = polygon.front().y;
  for (const auto& p : polygon) {
    x_min = std::min(x_min, p.x);
    x_max = std::max(x_max, p.x);
    y_min = std::min(y_min, p.y);
    y_max = std::max(y_max, p.y);
  }
}

common::Point3D bodyPointToWorld(
  const OrientedBodyBox& box,
  double bx,
  double by,
  double bz)
{
  const auto local = rotateBodyToWorld(bx, by, bz, box.yaw, box.pitch, box.roll);
  return {
    box.origin.x + local.x,
    box.origin.y + local.y,
    box.origin.z + local.z,
  };
}

double clampPitch(double pitch, double max_pitch)
{
  return std::clamp(pitch, -max_pitch, max_pitch);
}

}  // namespace

double sampleTerrainHeight(
  const map::IMap& map,
  double x,
  double y,
  double z_hint,
  double search_up,
  double search_down)
{
  const double resolution = std::max(1e-6, map.getResolution());
  // Prefer the top of the support slab near z_hint. Do not treat tall walls as
  // terrain (that would exempt their entire column from collision).
  const double z_top = z_hint + search_up;
  const double z_bottom = z_hint - search_down;
  double best_z = std::numeric_limits<double>::quiet_NaN();

  for (double z = z_bottom; z <= z_top + 1e-9; z += resolution) {
    const common::Point3D p{x, y, z};
    if (!map.isInBounds(p)) {
      continue;
    }
    if (map.isOccupied(p)) {
      best_z = z;
    }
  }
  return best_z;
}

BodyAttitude estimatePitchFromMap(
  const map::IMap& map,
  const common::Point3D& stance,
  double yaw,
  const Footprint3DConfig& config,
  const std::optional<BodyAttitude>& imu_override)
{
  if (imu_override.has_value() && imu_override->from_imu) {
    BodyAttitude attitude = *imu_override;
    attitude.yaw = yaw;
    attitude.pitch = clampPitch(attitude.pitch, config.max_pitch_rad);
    return attitude;
  }

  validateFootprint3DConfig(config);
  double x_min = 0.0;
  double x_max = 0.0;
  double y_min = 0.0;
  double y_max = 0.0;
  polygonExtents(config.polygon, x_min, x_max, y_min, y_max);
  const double wheelbase = std::max(1e-3, x_max - x_min);

  const double cy = std::cos(yaw);
  const double sy = std::sin(yaw);
  auto worldXy = [&](double bx, double by) {
    return std::pair<double, double>{
      stance.x + cy * bx - sy * by,
      stance.y + sy * bx + cy * by,
    };
  };

  auto averageHeightAtX = [&](double bx) {
    double sum = 0.0;
    int count = 0;
    for (double by = y_min; by <= y_max + 1e-9; by += config.sample_step) {
      const auto [wx, wy] = worldXy(bx, by);
      const double h = sampleTerrainHeight(map, wx, wy, stance.z);
      if (std::isfinite(h)) {
        sum += h;
        ++count;
      }
    }
    if (count == 0) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    return sum / static_cast<double>(count);
  };

  const double h_front = averageHeightAtX(x_max);
  const double h_rear = averageHeightAtX(x_min);

  BodyAttitude attitude;
  attitude.yaw = yaw;
  attitude.roll = 0.0;
  if (std::isfinite(h_front) && std::isfinite(h_rear)) {
    attitude.pitch = clampPitch(std::atan2(h_front - h_rear, wheelbase), config.max_pitch_rad);
  } else {
    attitude.pitch = 0.0;
  }
  return attitude;
}

BodyAttitude estimatePitchFromSegment(
  double delta_z,
  double delta_s,
  double max_pitch_rad,
  double yaw,
  const std::optional<BodyAttitude>& imu_override)
{
  if (imu_override.has_value() && imu_override->from_imu) {
    BodyAttitude attitude = *imu_override;
    attitude.yaw = yaw;
    attitude.pitch = clampPitch(attitude.pitch, max_pitch_rad);
    return attitude;
  }

  BodyAttitude attitude;
  attitude.yaw = yaw;
  attitude.roll = 0.0;
  if (delta_s <= 1e-9) {
    attitude.pitch = 0.0;
  } else {
    attitude.pitch = clampPitch(std::atan2(delta_z, delta_s), max_pitch_rad);
  }
  return attitude;
}

BodyAttitude estimateBodyAttitude(
  const map::IMap& map,
  const common::Point3D& stance,
  double yaw,
  double path_delta_z,
  double path_delta_s,
  const Footprint3DConfig& config,
  const std::optional<BodyAttitude>& imu_override)
{
  if (imu_override.has_value() && imu_override->from_imu) {
    BodyAttitude attitude = *imu_override;
    attitude.yaw = yaw;
    attitude.pitch = clampPitch(attitude.pitch, config.max_pitch_rad);
    return attitude;
  }

  // Prefer local terrain slope under the footprint so the belly stays parallel
  // to the stair/ramp surface. Path Δz/Δs is only a mild fallback when the map
  // looks flat: shortcut chords invent near-vertical pitch and drive the nose
  // into the stairs.
  auto map_attitude = estimatePitchFromMap(map, stance, yaw, config, std::nullopt);

  constexpr double kMinPathBaseline = 0.25;
  if (path_delta_s >= kMinPathBaseline) {
    const double path_pitch = clampPitch(
      std::atan2(path_delta_z, path_delta_s),
      config.max_pitch_rad);
    // When map already sees a ramp, trust map (parallel to stairs). When map is
    // nearly flat (landing / sparse samples), allow a limited path contribution.
    if (std::abs(map_attitude.pitch) < 0.08) {
      const double w = std::clamp(path_delta_s / (kMinPathBaseline * 2.0), 0.0, 0.45);
      map_attitude.pitch = (1.0 - w) * map_attitude.pitch + w * path_pitch;
    }
  }
  map_attitude.yaw = yaw;
  return map_attitude;
}

OrientedBodyBox makeOrientedBodyBox(
  const map::IMap& map,
  const common::Point3D& stance,
  const BodyAttitude& attitude,
  const Footprint3DConfig& config)
{
  validateFootprint3DConfig(config);
  OrientedBodyBox box;
  box.polygon = config.polygon;
  box.body_height = config.body_height;
  box.yaw = attitude.yaw;
  box.pitch = attitude.pitch;
  box.roll = attitude.roll;

  const double terrain = sampleTerrainHeight(map, stance.x, stance.y, stance.z);
  box.local_terrain_z = std::isfinite(terrain) ? terrain : (stance.z - config.body_clearance);

  // Lift along the body +Z (plane normal) so the bottom face stays parallel to
  // the estimated stair/ground plane instead of only shifting world-Z.
  const common::Point3D terrain_point{stance.x, stance.y, box.local_terrain_z};
  const auto lift = rotateBodyToWorld(
    0.0,
    0.0,
    config.body_clearance,
    attitude.yaw,
    attitude.pitch,
    attitude.roll);
  box.origin = {
    terrain_point.x + lift.x,
    terrain_point.y + lift.y,
    terrain_point.z + lift.z,
  };
  return box;
}

std::vector<common::Point3D> bodyBoxWireframeCorners(const OrientedBodyBox& box)
{
  std::vector<common::Point3D> corners;
  corners.reserve(box.polygon.size() * 2);
  for (const auto& p : box.polygon) {
    corners.push_back(bodyPointToWorld(box, p.x, p.y, 0.0));
  }
  for (const auto& p : box.polygon) {
    corners.push_back(bodyPointToWorld(box, p.x, p.y, box.body_height));
  }
  return corners;
}

Footprint3DChecker::Footprint3DChecker(Footprint3DConfig config)
  : config_(std::move(config))
{
  validateFootprint3DConfig(config_);
}

FootprintCollisionResult Footprint3DChecker::check(
  const map::IMap& map,
  const common::Point3D& stance,
  const BodyAttitude& attitude) const
{
  const auto box = makeOrientedBodyBox(map, stance, attitude, config_);
  FootprintCollisionResult result;

  common::BoundingBox aabb;
  for (const auto& p : config_.polygon) {
    aabb.expandToInclude(bodyPointToWorld(box, p.x, p.y, 0.0));
    aabb.expandToInclude(bodyPointToWorld(box, p.x, p.y, config_.body_height));
  }
  if (!aabb.valid) {
    return result;
  }

  const double resolution = std::max(1e-6, map.getResolution());
  const double pad = resolution;
  std::unordered_map<std::uint64_t, double> terrain_cache;
  auto terrainAt = [&](double x, double y) {
    const int ix = static_cast<int>(std::floor(x / resolution));
    const int iy = static_cast<int>(std::floor(y / resolution));
    const std::uint64_t key =
      (static_cast<std::uint64_t>(static_cast<std::uint32_t>(ix)) << 32U) |
      static_cast<std::uint32_t>(iy);
    const auto it = terrain_cache.find(key);
    if (it != terrain_cache.end()) {
      return it->second;
    }
    const double h = sampleTerrainHeight(map, x, y, box.local_terrain_z, 0.35, 2.0);
    terrain_cache.emplace(key, h);
    return h;
  };

  for (double z = aabb.min.z - pad; z <= aabb.max.z + pad + 1e-9; z += resolution) {
    for (double y = aabb.min.y - pad; y <= aabb.max.y + pad + 1e-9; y += resolution) {
      for (double x = aabb.min.x - pad; x <= aabb.max.x + pad + 1e-9; x += resolution) {
        const common::Point3D candidate{x, y, z};
        if (!map.isInBounds(candidate) || !map.isOccupied(candidate)) {
          continue;
        }

        const double terrain_z = terrainAt(x, y);
        const double ignore_z =
          (std::isfinite(terrain_z) ? terrain_z : box.local_terrain_z) +
          std::max(config_.terrain_ignore_margin, config_.body_clearance * 0.95);
        if (z <= ignore_z + 1e-9) {
          continue;
        }

        const auto local = worldToBody(candidate, box.origin, box.yaw, box.pitch, box.roll);
        if (local.z < -1e-9 || local.z > config_.body_height + 1e-9) {
          continue;
        }
        if (!pointInPolygon2D(local.x, local.y, config_.polygon)) {
          continue;
        }

        result.in_collision = true;
        result.hit_point = candidate;
        return result;
      }
    }
  }

  return result;
}

FootprintCollisionResult Footprint3DChecker::checkAtPose(
  const map::IMap& map,
  const common::Point3D& stance,
  double yaw,
  const std::optional<BodyAttitude>& imu_override,
  double path_delta_z,
  double path_delta_s) const
{
  // Prefer path slope over map front/rear heights: at stair landings the map
  // samples can span two flights and invent a bogus pitch that jams the body box.
  const auto attitude = estimateBodyAttitude(
    map, stance, yaw, path_delta_z, path_delta_s, config_, imu_override);
  return check(map, stance, attitude);
}

}  // namespace nav3d::collision

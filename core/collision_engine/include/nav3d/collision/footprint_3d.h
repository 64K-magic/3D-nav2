#pragma once

#include <cmath>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

#include "nav3d/common/types.h"
#include "nav3d/map/i_map.h"

namespace nav3d::collision {

struct FootprintPoint2D {
  double x = 0.0;
  double y = 0.0;
};

struct Footprint3DConfig {
  std::vector<FootprintPoint2D> polygon;
  double body_clearance = 0.5;
  double body_height = 0.35;
  double max_pitch_rad = 0.7;
  double terrain_ignore_margin = 0.4;
  double sample_step = 0.1;
};

struct BodyAttitude {
  double yaw = 0.0;
  double pitch = 0.0;
  double roll = 0.0;
  bool from_imu = false;
};

struct FootprintCollisionResult {
  bool in_collision = false;
  std::optional<common::Point3D> hit_point;
};

struct OrientedBodyBox {
  common::Point3D origin;
  double yaw = 0.0;
  double pitch = 0.0;
  double roll = 0.0;
  std::vector<FootprintPoint2D> polygon;
  double body_height = 0.35;
  double local_terrain_z = 0.0;
};

inline Footprint3DConfig makeDefaultDogFootprintConfig()
{
  Footprint3DConfig config;
  config.polygon = {
    {-0.67, -0.4},
    {-0.67, 0.4},
    {0.87, 0.4},
    {0.87, -0.4},
  };
  return config;
}

inline void validateFootprint3DConfig(const Footprint3DConfig& config)
{
  if (config.polygon.size() < 3) {
    throw std::invalid_argument("Footprint3DConfig.polygon must have at least 3 vertices");
  }
  if (!std::isfinite(config.body_clearance) || config.body_clearance < 0.0) {
    throw std::invalid_argument("Footprint3DConfig.body_clearance must be finite and non-negative");
  }
  if (!std::isfinite(config.body_height) || config.body_height <= 0.0) {
    throw std::invalid_argument("Footprint3DConfig.body_height must be positive and finite");
  }
  if (!std::isfinite(config.max_pitch_rad) || config.max_pitch_rad < 0.0) {
    throw std::invalid_argument("Footprint3DConfig.max_pitch_rad must be finite and non-negative");
  }
  if (!std::isfinite(config.terrain_ignore_margin) || config.terrain_ignore_margin < 0.0) {
    throw std::invalid_argument(
      "Footprint3DConfig.terrain_ignore_margin must be finite and non-negative");
  }
  if (!std::isfinite(config.sample_step) || config.sample_step <= 0.0) {
    throw std::invalid_argument("Footprint3DConfig.sample_step must be positive and finite");
  }
}

double sampleTerrainHeight(
  const map::IMap& map,
  double x,
  double y,
  double z_hint,
  double search_up = 0.35,
  double search_down = 2.0);

BodyAttitude estimatePitchFromMap(
  const map::IMap& map,
  const common::Point3D& stance,
  double yaw,
  const Footprint3DConfig& config,
  const std::optional<BodyAttitude>& imu_override = std::nullopt);

BodyAttitude estimatePitchFromSegment(
  double delta_z,
  double delta_s,
  double max_pitch_rad,
  double yaw,
  const std::optional<BodyAttitude>& imu_override = std::nullopt);

// Prefer path slope (Δz/Δs) when the horizontal baseline is meaningful; otherwise
// fall back to map front/rear support heights. IMU still wins when enabled.
BodyAttitude estimateBodyAttitude(
  const map::IMap& map,
  const common::Point3D& stance,
  double yaw,
  double path_delta_z,
  double path_delta_s,
  const Footprint3DConfig& config,
  const std::optional<BodyAttitude>& imu_override = std::nullopt);

OrientedBodyBox makeOrientedBodyBox(
  const map::IMap& map,
  const common::Point3D& stance,
  const BodyAttitude& attitude,
  const Footprint3DConfig& config);

std::vector<common::Point3D> bodyBoxWireframeCorners(const OrientedBodyBox& box);

class Footprint3DChecker {
public:
  explicit Footprint3DChecker(Footprint3DConfig config);

  const Footprint3DConfig& config() const { return config_; }

  FootprintCollisionResult check(
    const map::IMap& map,
    const common::Point3D& stance,
    const BodyAttitude& attitude) const;

  FootprintCollisionResult checkAtPose(
    const map::IMap& map,
    const common::Point3D& stance,
    double yaw,
    const std::optional<BodyAttitude>& imu_override = std::nullopt,
    double path_delta_z = 0.0,
    double path_delta_s = 0.0) const;

private:
  Footprint3DConfig config_;
};

}  // namespace nav3d::collision

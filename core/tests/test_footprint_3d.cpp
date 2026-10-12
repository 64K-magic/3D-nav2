#include <cmath>
#include <gtest/gtest.h>

#include "nav3d/collision/footprint_3d.h"
#include "nav3d/map/voxel_grid_map.h"

namespace {

nav3d::map::VoxelGridMap makeFlatFloor(double resolution, double z_floor = 0.0)
{
  nav3d::map::VoxelGridMap map(resolution);
  map.setExplicitBounds({-3.0, -3.0, -1.0}, {5.0, 3.0, 3.0});
  for (double x = -2.0; x <= 4.0 + 1e-9; x += resolution) {
    for (double y = -2.0; y <= 2.0 + 1e-9; y += resolution) {
      map.insertOccupied({x, y, z_floor});
    }
  }
  return map;
}

}  // namespace

TEST(Footprint3D, PitchNearZeroOnFlatFloor)
{
  auto map = makeFlatFloor(0.2);
  auto config = nav3d::collision::makeDefaultDogFootprintConfig();
  config.sample_step = 0.2;
  const auto attitude = nav3d::collision::estimatePitchFromMap(
    map,
    {0.0, 0.0, 0.5},
    0.0,
    config);
  EXPECT_NEAR(attitude.pitch, 0.0, 0.05);
}

TEST(Footprint3D, PitchMatchesArtificialRamp)
{
  nav3d::map::VoxelGridMap map(0.2);
  map.setExplicitBounds({-3.0, -2.0, -1.0}, {4.0, 2.0, 3.0});
  // Ramp: terrain z = 0.2 * x for x in [-1, 2]
  for (double x = -1.0; x <= 2.0 + 1e-9; x += 0.2) {
    const double z = 0.2 * x;
    for (double y = -1.0; y <= 1.0 + 1e-9; y += 0.2) {
      map.insertOccupied({x, y, z});
    }
  }

  auto config = nav3d::collision::makeDefaultDogFootprintConfig();
  config.sample_step = 0.2;
  config.max_pitch_rad = 1.2;
  const auto attitude = nav3d::collision::estimatePitchFromMap(
    map,
    {0.5, 0.0, 0.5},
    0.0,
    config);
  // Slope dz/dx = 0.2 -> pitch ~= atan(0.2)
  EXPECT_NEAR(attitude.pitch, std::atan(0.2), 0.12);
}

TEST(Footprint3D, IgnoresFloorButHitsSideWall)
{
  auto map = makeFlatFloor(0.2);
  // Vertical wall beside the robot (+y), outside default footprint (|y|<=0.4)
  for (double z = 0.2; z <= 1.4 + 1e-9; z += 0.2) {
    for (double x = -0.5; x <= 0.5 + 1e-9; x += 0.2) {
      map.insertOccupied({x, 0.9, z});
    }
  }

  auto config = nav3d::collision::makeDefaultDogFootprintConfig();
  config.body_clearance = 0.5;
  config.body_height = 0.35;
  config.terrain_ignore_margin = 0.4;
  config.sample_step = 0.2;
  nav3d::collision::Footprint3DChecker checker(config);

  const auto clear = checker.checkAtPose(map, {0.0, 0.0, 0.5}, 0.0);
  EXPECT_FALSE(clear.in_collision);

  // Move sideways so footprint overlaps the wall column.
  const auto hit = checker.checkAtPose(map, {0.0, 0.55, 0.5}, 0.0);
  EXPECT_TRUE(hit.in_collision);
}

TEST(Footprint3D, BodyAttitudePrefersTerrainOverSteepPathChord)
{
  auto map = makeFlatFloor(0.2);
  auto config = nav3d::collision::makeDefaultDogFootprintConfig();
  // Flat terrain must not inherit a near-vertical path chord pitch.
  const auto attitude = nav3d::collision::estimateBodyAttitude(
    map,
    {0.0, 0.0, 0.5},
    0.0,
    0.4,
    1.0,
    config,
    std::nullopt);
  EXPECT_LT(std::abs(attitude.pitch), 0.25);
  EXPECT_LT(std::abs(attitude.pitch), std::abs(std::atan2(0.4, 1.0)));
}

TEST(Footprint3D, PositivePitchRaisesNoseParallelToRamp)
{
  auto map = makeFlatFloor(0.2);
  auto config = nav3d::collision::makeDefaultDogFootprintConfig();
  config.body_clearance = 0.5;
  nav3d::collision::BodyAttitude attitude;
  attitude.yaw = 0.0;
  attitude.pitch = std::atan(0.2);
  attitude.roll = 0.0;
  const auto box = nav3d::collision::makeOrientedBodyBox(
    map, {0.0, 0.0, 0.5}, attitude, config);
  const auto front = nav3d::collision::bodyBoxWireframeCorners(box);
  // Polygon front is +x; bottoms are first half of corners.
  double z_front = 0.0;
  double z_rear = 0.0;
  int n_front = 0;
  int n_rear = 0;
  const std::size_t n = config.polygon.size();
  for (std::size_t i = 0; i < n; ++i) {
    if (config.polygon[i].x > 0.0) {
      z_front += front[i].z;
      ++n_front;
    } else {
      z_rear += front[i].z;
      ++n_rear;
    }
  }
  ASSERT_GT(n_front, 0);
  ASSERT_GT(n_rear, 0);
  z_front /= static_cast<double>(n_front);
  z_rear /= static_cast<double>(n_rear);
  EXPECT_GT(z_front, z_rear + 0.05);
}

TEST(Footprint3D, ImuOverridePreferred)
{
  auto map = makeFlatFloor(0.2);
  auto config = nav3d::collision::makeDefaultDogFootprintConfig();
  nav3d::collision::BodyAttitude imu;
  imu.from_imu = true;
  imu.pitch = 0.25;
  imu.roll = 0.01;
  const auto attitude = nav3d::collision::estimatePitchFromMap(
    map,
    {0.0, 0.0, 0.5},
    1.2,
    config,
    imu);
  EXPECT_TRUE(attitude.from_imu);
  EXPECT_NEAR(attitude.pitch, 0.25, 1e-9);
  EXPECT_NEAR(attitude.yaw, 1.2, 1e-9);
}

TEST(Footprint3D, RejectsInvalidConfig)
{
  nav3d::collision::Footprint3DConfig bad;
  bad.polygon = {{0.0, 0.0}};
  EXPECT_THROW(
    {
      nav3d::collision::Footprint3DChecker checker(bad);
      (void)checker;
    },
    std::invalid_argument);
}

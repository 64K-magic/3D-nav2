#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "nav3d/controller/trajectory_tracker.h"
#include "nav3d/map/local_grid.h"
#include "nav3d/map/voxel_grid_map.h"
#include "nav3d/planner/bspline/uniform_bspline.h"

namespace {

constexpr double kPi = 3.14159265358979323846;

nav3d::planner::UniformBspline makeLineTrajectory()
{
  return nav3d::planner::UniformBspline(
    {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {2.0, 0.0, 0.0}, {3.0, 0.0, 0.0}},
    1,
    1.0);
}

nav3d::controller::TrajectoryTrackerConfig makeConfig()
{
  nav3d::controller::TrajectoryTrackerConfig config;
  config.sample_step_seconds = 0.05;
  config.control_dt = 0.1;
  config.lookahead_time = 0.4;
  config.max_linear_speed = 0.6;
  config.goal_tolerance = 0.15;
  config.yaw_gain = 2.0;
  config.max_yaw_rate = 0.75;
  return config;
}

}  // namespace

TEST(TrajectoryTracker, ConvertsMapVelocityIntoBodyFrame)
{
  auto config = makeConfig();
  config.command_frame = nav3d::controller::CommandFrame::Body;
  const nav3d::controller::TrajectoryTracker tracker(config);

  const auto command = tracker.computeCommand(
    makeLineTrajectory(),
    {0.0, 0.0, 0.0},
    kPi / 2.0,
    {3.0, 0.0, 0.0});

  EXPECT_FALSE(command.goal_reached);
  EXPECT_NEAR(command.linear.x, 0.0, 1e-6);
  EXPECT_NEAR(command.linear.y, -0.6, 1e-6);
  EXPECT_NEAR(command.linear.z, 0.0, 1e-6);
  EXPECT_NEAR(command.angular_z, -0.75, 1e-6);
}

TEST(TrajectoryTracker, CanPublishMapFrameVelocityForSimulatorAdapters)
{
  auto config = makeConfig();
  config.command_frame = nav3d::controller::CommandFrame::Map;
  const nav3d::controller::TrajectoryTracker tracker(config);

  const auto command = tracker.computeCommand(
    makeLineTrajectory(),
    {0.0, 0.0, 0.0},
    kPi / 2.0,
    {3.0, 0.0, 0.0});

  EXPECT_FALSE(command.goal_reached);
  EXPECT_NEAR(command.linear.x, 0.6, 1e-6);
  EXPECT_NEAR(command.linear.y, 0.0, 1e-6);
  EXPECT_NEAR(command.linear.z, 0.0, 1e-6);
  EXPECT_NEAR(command.angular_z, -0.75, 1e-6);
}

TEST(TrajectoryTracker, PublishesZeroCommandWhenGoalIsReached)
{
  const nav3d::controller::TrajectoryTracker tracker(makeConfig());

  const auto command = tracker.computeCommand(
    makeLineTrajectory(),
    {2.95, 0.0, 0.0},
    0.0,
    {3.0, 0.0, 0.0});

  EXPECT_TRUE(command.goal_reached);
  EXPECT_NEAR(command.linear.x, 0.0, 1e-9);
  EXPECT_NEAR(command.linear.y, 0.0, 1e-9);
  EXPECT_NEAR(command.linear.z, 0.0, 1e-9);
  EXPECT_NEAR(command.angular_z, 0.0, 1e-9);
}

TEST(TrajectoryTracker, AppliesDifferentialDriveMotionModel)
{
  auto config = makeConfig();
  config.command_frame = nav3d::controller::CommandFrame::Body;
  config.motion_model.type = nav3d::controller::MotionModelType::DifferentialDrive;
  const nav3d::controller::TrajectoryTracker tracker(config);

  const auto command = tracker.computeCommand(
    makeLineTrajectory(),
    {0.0, 0.0, 0.0},
    kPi / 2.0,
    {3.0, 0.0, 0.0});

  EXPECT_NEAR(command.linear.x, 0.0, 1e-6);
  EXPECT_NEAR(command.linear.y, 0.0, 1e-6);
  EXPECT_NEAR(command.linear.z, 0.0, 1e-6);
  EXPECT_NEAR(command.angular_z, -0.75, 1e-6);
}

TEST(TrajectoryTracker, AppliesMotionModelAccelerationLimitsWithPreviousCommand)
{
  auto config = makeConfig();
  config.command_frame = nav3d::controller::CommandFrame::Map;
  config.motion_model.type = nav3d::controller::MotionModelType::Omni;
  config.motion_model.max_linear_speed = 10.0;
  config.motion_model.max_linear_acceleration = 2.0;
  config.motion_model.max_yaw_rate = 10.0;
  config.motion_model.max_yaw_acceleration = 1.0;
  const nav3d::controller::TrajectoryTracker tracker(config);

  nav3d::controller::TrajectoryCommand previous;
  previous.linear = {0.0, 0.0, 0.0};
  previous.angular_z = 0.0;

  const auto command = tracker.computeCommand(
    makeLineTrajectory(),
    {0.0, 0.0, 0.0},
    kPi / 2.0,
    {3.0, 0.0, 0.0},
    previous);

  EXPECT_NEAR(command.linear.x, 0.2, 1e-6);
  EXPECT_NEAR(command.linear.y, 0.0, 1e-6);
  EXPECT_NEAR(command.angular_z, -0.1, 1e-6);
}

TEST(TrajectoryTracker, StopsWhenCurrentPoseIsNotFreeInActiveMap)
{
  auto config = makeConfig();
  config.command_frame = nav3d::controller::CommandFrame::Map;
  config.motion_model.type = nav3d::controller::MotionModelType::Omni;
  const nav3d::controller::TrajectoryTracker tracker(config);

  nav3d::map::VoxelGridMap map(0.1);
  map.setExplicitBounds({-1.0, -1.0, 0.0}, {3.0, 1.0, 0.0});
  map.insertOccupied({0.0, 0.0, 0.0});

  const auto command = tracker.computeCommand(
    makeLineTrajectory(),
    {0.0, 0.0, 0.0},
    0.0,
    {3.0, 0.0, 0.0},
    map);

  EXPECT_FALSE(command.goal_reached);
  EXPECT_NEAR(command.linear.x, 0.0, 1e-9);
  EXPECT_NEAR(command.linear.y, 0.0, 1e-9);
  EXPECT_NEAR(command.linear.z, 0.0, 1e-9);
}

TEST(TrajectoryTracker, ShortensLookaheadToReachableSafeSegment)
{
  auto config = makeConfig();
  config.command_frame = nav3d::controller::CommandFrame::Map;
  config.motion_model.type = nav3d::controller::MotionModelType::Omni;
  config.control_dt = 1.0;
  config.lookahead_time = 2.0;
  config.max_linear_speed = 2.0;
  const nav3d::controller::TrajectoryTracker tracker(config);

  nav3d::map::VoxelGridMap map(0.1);
  map.setExplicitBounds({-1.0, -1.0, 0.0}, {3.0, 1.0, 0.0});
  map.insertOccupied({1.0, 0.0, 0.0});

  const auto command = tracker.computeCommand(
    makeLineTrajectory(),
    {0.0, 0.0, 0.0},
    0.0,
    {3.0, 0.0, 0.0},
    map);

  EXPECT_FALSE(command.goal_reached);
  EXPECT_GT(command.linear.x, 0.0);
  EXPECT_LT(command.linear.x, 1.0);
  EXPECT_NEAR(command.linear.y, 0.0, 1e-9);
  EXPECT_NEAR(command.linear.z, 0.0, 1e-9);
}

TEST(TrajectoryTracker, TreatsLocalGridUnknownAsUnsafeForCommands)
{
  auto config = makeConfig();
  config.command_frame = nav3d::controller::CommandFrame::Map;
  config.motion_model.type = nav3d::controller::MotionModelType::Omni;
  config.control_dt = 1.0;
  config.lookahead_time = 1.5;
  config.max_linear_speed = 2.0;
  const nav3d::controller::TrajectoryTracker tracker(config);

  nav3d::map::LocalGrid local_grid(0.1, {-1.0, -1.0, 0.0}, {3.0, 1.0, 0.0});
  local_grid.markFree({0.0, 0.0, 0.0});
  local_grid.markFree({0.5, 0.0, 0.0});

  const auto command = tracker.computeCommand(
    makeLineTrajectory(),
    {0.0, 0.0, 0.0},
    0.0,
    {3.0, 0.0, 0.0},
    local_grid);

  EXPECT_FALSE(command.goal_reached);
  EXPECT_GT(command.linear.x, 0.0);
  EXPECT_LE(command.linear.x, 0.5);
  EXPECT_NEAR(command.linear.y, 0.0, 1e-9);
  EXPECT_NEAR(command.linear.z, 0.0, 1e-9);
}

TEST(TrajectoryTracker, ConstrainedCommandDoesNotOvershootReachableSegment)
{
  auto config = makeConfig();
  config.command_frame = nav3d::controller::CommandFrame::Map;
  config.motion_model.type = nav3d::controller::MotionModelType::Omni;
  config.motion_model.max_linear_acceleration = 1.0;
  config.control_dt = 1.0;
  config.lookahead_time = 1.5;
  config.max_linear_speed = 2.0;
  const nav3d::controller::TrajectoryTracker tracker(config);

  nav3d::map::VoxelGridMap map(0.1);
  map.setExplicitBounds({-1.0, -1.0, 0.0}, {3.0, 1.0, 0.0});
  map.insertOccupied({0.4, 0.0, 0.0});

  nav3d::controller::TrajectoryCommand previous;
  previous.linear = {2.0, 0.0, 0.0};

  const auto command = tracker.computeCommand(
    makeLineTrajectory(),
    {0.0, 0.0, 0.0},
    0.0,
    {3.0, 0.0, 0.0},
    map,
    previous);

  EXPECT_FALSE(command.goal_reached);
  EXPECT_GE(command.linear.x, 0.0);
  EXPECT_LT(command.linear.x * config.control_dt, 0.4);
  EXPECT_NEAR(command.linear.y, 0.0, 1e-9);
  EXPECT_NEAR(command.linear.z, 0.0, 1e-9);
}

TEST(TrajectoryTracker, MapAwareCommandsStayInsideReachableFreeSegmentMatrix)
{
  int cases = 0;
  for (double obstacle_x : {0.4, 0.6, 0.8, 1.0, 1.2}) {
    for (double lookahead_time : {0.6, 1.0, 1.5, 2.0}) {
      for (double current_x : {0.0, 0.1, 0.2}) {
        auto config = makeConfig();
        config.command_frame = nav3d::controller::CommandFrame::Map;
        config.motion_model.type = nav3d::controller::MotionModelType::Omni;
        config.control_dt = 1.0;
        config.lookahead_time = lookahead_time;
        config.max_linear_speed = 2.0;
        const nav3d::controller::TrajectoryTracker tracker(config);

        nav3d::map::VoxelGridMap map(0.1);
        map.setExplicitBounds({-1.0, -1.0, 0.0}, {3.0, 1.0, 0.0});
        map.insertOccupied({obstacle_x, 0.0, 0.0});

        const auto command = tracker.computeCommand(
          makeLineTrajectory(),
          {current_x, 0.0, 0.0},
          0.0,
          {3.0, 0.0, 0.0},
          map);

        SCOPED_TRACE(
          "obstacle_x=" + std::to_string(obstacle_x) +
          " lookahead_time=" + std::to_string(lookahead_time) +
          " current_x=" + std::to_string(current_x));
        EXPECT_FALSE(command.goal_reached);
        EXPECT_GE(command.linear.x, 0.0);
        EXPECT_NEAR(command.linear.y, 0.0, 1e-9);
        EXPECT_NEAR(command.linear.z, 0.0, 1e-9);
        EXPECT_LT(current_x + command.linear.x * config.control_dt, obstacle_x);
        ++cases;
      }
    }
  }
  EXPECT_GE(cases, 60);
}

TEST(TrajectoryTracker, MonotonicLateralCorrectionDoesNotPullDownWhenAboveRibbon)
{
  auto config = makeConfig();
  config.command_frame = nav3d::controller::CommandFrame::Map;
  config.motion_model.type = nav3d::controller::MotionModelType::Uav;
  config.max_linear_speed = 1.0;
  config.goal_tolerance = 0.15;
  const nav3d::controller::TrajectoryTracker tracker(config);

  // Climbing ribbon: z rises with path time.
  const auto trajectory = nav3d::planner::UniformBspline(
    {{0.0, 0.0, 0.0}, {1.0, 0.0, 1.0}, {2.0, 0.0, 2.0}, {3.0, 0.0, 3.0}},
    1,
    1.0);

  // Slightly off the ribbon laterally but still inside the stair corridor,
  // and above the tread — must not be yanked downward.
  double path_time = 0.8;
  const auto command = tracker.computeCommandMonotonic(
    trajectory,
    {1.0, 0.28, 1.35},
    0.0,
    {3.0, 0.0, 3.0},
    path_time,
    std::nullopt);

  EXPECT_FALSE(command.goal_reached);
  EXPECT_GE(command.linear.z, -1e-9);
  EXPECT_GT(std::abs(command.linear.y), 1e-3);
}

TEST(TrajectoryTracker, MonotonicPullsBackWhenOutsideStairCorridor)
{
  auto config = makeConfig();
  config.command_frame = nav3d::controller::CommandFrame::Map;
  config.motion_model.type = nav3d::controller::MotionModelType::Uav;
  config.max_linear_speed = 0.55;
  config.lookahead_time = 0.25;
  config.goal_tolerance = 0.15;
  const nav3d::controller::TrajectoryTracker tracker(config);

  const auto trajectory = nav3d::planner::UniformBspline(
    {{0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}, {0.0, 0.0, 2.0}, {0.0, 0.0, 3.0}},
    1,
    1.0);

  // Far beside the vertical climb ribbon (outside the well).
  double path_time = 0.5;
  const auto command = tracker.computeCommandMonotonic(
    trajectory,
    {1.2, 0.0, 1.0},
    0.0,
    {0.0, 0.0, 3.0},
    path_time,
    std::nullopt);

  EXPECT_FALSE(command.goal_reached);
  // Must command back toward x=0 (into the well), not further out.
  EXPECT_LT(command.linear.x, 0.0);
}

TEST(TrajectoryTracker, MonotonicForwardSnapUnsticksCornerCutting)
{
  auto config = makeConfig();
  config.command_frame = nav3d::controller::CommandFrame::Map;
  config.motion_model.type = nav3d::controller::MotionModelType::Uav;
  config.max_linear_speed = 1.0;
  config.goal_tolerance = 0.15;
  config.control_dt = 0.1;
  const nav3d::controller::TrajectoryTracker tracker(config);

  const auto trajectory = nav3d::planner::UniformBspline(
    {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {2.0, 0.0, 1.0}, {3.0, 0.0, 2.0}},
    1,
    1.0);

  // Robot has cut ahead near the climb while path_time is still frozen at 0.
  double path_time = 0.0;
  const auto command = tracker.computeCommandMonotonic(
    trajectory,
    {1.85, 0.35, 0.2},
    0.0,
    {3.0, 0.0, 2.0},
    path_time,
    std::nullopt);

  EXPECT_FALSE(command.goal_reached);
  EXPECT_GT(path_time, 0.5);
  // Must not remain glued to the t=0 origin after snapping forward.
  EXPECT_GT(std::hypot(command.linear.x, command.linear.y, command.linear.z), 1e-3);
}

TEST(TrajectoryTracker, RejectsInvalidConfiguration)
{
  auto config = makeConfig();
  config.max_linear_speed = 0.0;

  EXPECT_THROW(
    nav3d::controller::TrajectoryTracker tracker(config),
    std::invalid_argument);
}

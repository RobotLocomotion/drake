#pragma once

/// @file
/// The fixed, versioned benchmark worlds and trajectories (the benchmark
/// suite). Every world is built from the cached `drake_models` iiwa14
/// dense-sphere collision model plus programmatic anchored boxes, so a run is
/// reproducible from this file alone.

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "drake/planning/robot_diagram.h"

namespace drake {
namespace planning {
namespace certified_ccd {
namespace benchmark {

/// The dense-sphere iiwa14 collision variant: 46 collision spheres over
/// links 0-7, i.e. realistic proximity-pair counts (the benchmark suite asks
/// for the realistic model, not the 4-primitive one).
extern const char* const kIiwaUrl;

/// Nominal shelf geometry. `shelf_scale` s translates the whole bookcase by
/// +s in x *and* opens the reached-into bay by s on each side, so the swept
/// clearance of the fixed benchmark trajectory is monotone non-decreasing in
/// s over the useful range. This one scalar is what the tier bisection turns.
struct ShelfGeometry {
  static constexpr double kBayCentreZ = 0.60;
  static constexpr double kFrontX = 0.62;
  static constexpr double kDepth = 0.32;
  static constexpr double kBayHalfHeight = 0.13;
  static constexpr double kPanel = 0.03;
  static constexpr double kHalfWidth = 0.45;
  static constexpr double kBottomZ = 0.05;
  static constexpr double kTopZ = 1.25;
};

/// iiwa14 welded to the world origin, a 3 m table slab, and a seven-box
/// bookcase in reach. Model instances are named "iiwa14" and "environment".
std::shared_ptr<drake::planning::RobotDiagram<double>> MakeShelfWorld(
    double shelf_scale);

/// Two iiwa14s welded to the world `base_separation` apart along +x, the
/// second rotated 180 degrees about z so the arms face each other, over the
/// same table slab. Model instances: "iiwa14", "iiwa14_1", "environment".
std::shared_ptr<drake::planning::RobotDiagram<double>> MakeDualArmWorld(
    double base_separation);

/// The 7 x 7 joint-space waypoint matrix of the shelf-reaching trajectory:
/// home, up-and-over on the +y side, into the bay mouth, deep inside the bay,
/// out on the -y side, and home. Solved once offline with
/// drake::multibody::InverseKinematics (position + tool-axis + minimum-
/// distance constraints) against the shelf-free world, then frozen here so
/// the benchmark has no solver dependency and no run-to-run drift.
Eigen::MatrixXd ShelfTrajectoryWaypoints();

/// Times of the shelf waypoints (0, 1, ..., 6): 6 quintic Bézier segments.
std::vector<double> ShelfTrajectoryTimes();

/// The 14 x 5 waypoint matrix of the dual-arm handover: both arms home, half
/// way, at the handover poses (end-effectors passing within a few cm), a
/// slightly different half-way pose on the way back, and home.
Eigen::MatrixXd DualArmTrajectoryWaypoints();

std::vector<double> DualArmTrajectoryTimes();

}  // namespace benchmark
}  // namespace certified_ccd
}  // namespace planning
}  // namespace drake

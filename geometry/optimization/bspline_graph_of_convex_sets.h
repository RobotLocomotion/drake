#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/trajectories/bspline_trajectory.h"
#include "drake/geometry/optimization/hpolyhedron.h"

namespace drake {
namespace geometry {
namespace optimization {
class BsplineTrajectoryThroughUnionOfHPolyhedra {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(
      BsplineTrajectoryThroughUnionOfHPolyhedra);

  /** Constructs the problem. */
  BsplineTrajectoryThroughUnionOfHPolyhedra(
    const Eigen::Ref<const Eigen::VectorXd>& source,
    const Eigen::Ref<const Eigen::VectorXd>& target,
    const std::vector<HPolyhedron>& regions);

  std::optional<trajectories::BsplineTrajectory<double>> Solve(
      bool use_rounding = false) const;

  int order() const { return order_; }

  int max_repetitions() const { return max_repetitions_; }

  void set_order(int order) { order_ = order; }

  void set_max_repetitions(int max_repetitions) {
    max_repetitions_ = max_repetitions;
  }

  void set_extra_control_points_per_region(
      int extra_control_points_per_region) {
    extra_control_points_per_region_ = extra_control_points_per_region;
  }

  void set_max_velocity(const Eigen::Ref<const Eigen::VectorXd>& max_velocity) {
    max_velocity_ = max_velocity;
  }

  int ambient_dimension() const { return regions_.back().ambient_dimension(); }

  int num_regions() const { return regions_.size(); }

  int extra_control_points_per_region() const {
    return extra_control_points_per_region_;
  }

  const VectorX<double>& source() const { return source_; }

  const VectorX<double>& target() const { return target_; }

  const VectorX<double>& max_velocity() const { return max_velocity_; }

 private:
  int source_index_{};
  int target_index_{};
  VectorX<double> source_{};
  VectorX<double> target_{};
  int order_{6};
  int max_repetitions_{1};
  int extra_control_points_per_region_{0};
  VectorX<double> max_velocity_{};
  std::vector<HPolyhedron> regions_;
};
}  // namespace optimization
}  // namespace geometry
}  // namespace drake

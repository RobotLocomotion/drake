#pragma once

#include <memory>
#include <vector>

#include "drake/planning/coverage_checker_base.h"
#include "drake/planning/point_sampler_base.h"

namespace drake {
namespace planning {
using geometry::optimization::ConvexSets;

class CoverageCheckerViaBernoulliTest final : public CoverageCheckerBase {
 public:
  /**
   * @param alpha CheckCoverage returns True if an alpha fraction of
   * the sampled points are in the current sets
   * @param num_points_per_check Number of points to sample when checking
   * coverage.
   * @param point_sampler An object which samples points at random from
   * some underlying space which we will check is covered by the convex sets.
   * @param num_threads Number of threads to use when checking coverage. Numbers
   * less than 1 will be interpretted as hardware concurrency.
   * @param point_in_set_tol Membership in convex sets is frequently checked by
   * solving optimization programs. This is the tolerance used to decide whether
   * a point is in a given set. See ConvexSet::PointInSet for details.
   */
  CoverageCheckerViaBernoulliTest(
      const double alpha, const int num_points_per_check,
      std::unique_ptr<PointSamplerBase> point_sampler,
      const int num_threads = -1, const double point_in_set_tol = 1e-8);

  double get_alpha() const { return alpha_; }

  /**
   * Set the value of alpha. CheckCoverage returns True if an alpha fraction of
   * the sampled points are in the current sets. This method clamps the passed
   * alpha to the range [0,1].
   * @param alpha
   */
  void set_alpha(const double alpha) { alpha_ = std::clamp(alpha, 0.0, 1.0); }

  int get_num_points_per_check() const { return num_points_per_check_; }

  void set_num_points_per_check(const double num_points_per_check) {
    num_points_per_check_ = num_points_per_check;
  }

  int get_num_threads() const { return num_threads_; }

  void set_num_threads(const int num_threads) { num_threads_ = num_threads; }

  double get_point_in_set_tol() const { return point_in_set_tol_; }

  void set_point_in_set_tol(const double point_in_set_tol) {
    point_in_set_tol_ = point_in_set_tol;
  }

  /**
   * Samples num_points_per_check from the point_sampler and returns the
   * fraction which are contained in at least one of current_sets.
   * @param current_sets
   */
  double GetSampledCoverageFraction(const ConvexSets& current_sets) const;

 private:
  bool DoCheckCoverage(const ConvexSets& current_sets) const override;

  double alpha_;
  int num_points_per_check_;
  const std::unique_ptr<PointSamplerBase> point_sampler_;
  int num_threads_;
  double point_in_set_tol_;
};

}  // namespace planning
}  // namespace drake

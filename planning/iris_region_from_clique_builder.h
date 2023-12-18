#pragma once

#include <memory>

#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/geometry/optimization/iris.h"
#include "drake/planning/convex_set_from_clique_builder_base.h"

namespace drake {
namespace planning {

using geometry::optimization::ConvexSet;
using geometry::optimization::ConvexSets;
using geometry::optimization::HPolyhedron;
using geometry::optimization::IrisOptions;

struct DefaultIrisOptionsForIrisRegionFromCliqueBuilder : public IrisOptions {
  DefaultIrisOptionsForIrisRegionFromCliqueBuilder();
};

/**
 * Given a set of points, compute their minimum volume, circumscribed ellipsoid
 * and seed the IRIS algorithm using this as the starting ellipsoid. It is
 * recommended to only run IRIS for one iteration when constructing IRIS regions
 * in this manner. This class provides an implementation of the interface
 * ConvexSetFromCliqueBuilderBase for use in
 * TODO(Alexandre.Amice) finish this name
 * ApproximateConvexCover
 */
class IrisRegionFromCliqueBuilder final
    : public ConvexSetFromCliqueBuilderBase {
 public:
  /**
   * @param obstacles The obstacles for IRIS. See @Iris for explanation.
   * @param domain The domain for IRIS. See @Iris for explanation.
   * @param options The options for IRIS. See @Iris and @IrisOptions for
   * explanation.
   * @param rank_tol_for_lowner_john_ellipse The tolerance used to detect when
   * points in an affine subspace. See
   * @Hyperellipsoid::MinimumVolumeCircumscribedEllipsoid for explanation.
   */
  IrisRegionFromCliqueBuilder(
      const ConvexSets& obstacles, const HPolyhedron& domain,
      const IrisOptions& options =
          DefaultIrisOptionsForIrisRegionFromCliqueBuilder(),
      const double rank_tol_for_lowner_john_ellipse = 1e-6);

  [[nodiscard]] ConvexSets get_obstacles() const { return obstacles_; }

  void set_obstacles(const ConvexSets& obstacles) { obstacles_ = obstacles; }

  [[nodiscard]] HPolyhedron get_domain() const { return domain_; }

  void set_domain(const HPolyhedron& domain) { domain_ = domain; }

  [[nodiscard]] IrisOptions get_options() const { return options_; }

  void set_options(const IrisOptions& options) { options_ = options; }

  [[nodiscard]] double get_rank_tol_for_lowner_john_ellipse() {
    return rank_tol_for_lowner_john_ellipse_;
  }

  void set_rank_tol_for_lowner_john_ellipse(
      const double rank_tol_for_lowner_john_ellipse) {
    rank_tol_for_lowner_john_ellipse_ = rank_tol_for_lowner_john_ellipse;
  }

 private:
  copyable_unique_ptr<ConvexSet> DoBuildConvexSet(
      const Eigen::Ref<const Eigen::MatrixXd>& clique_points) override;

  ConvexSets obstacles_;
  HPolyhedron domain_;
  IrisOptions options_;
  double rank_tol_for_lowner_john_ellipse_;
};

}  // namespace planning
}  // namespace drake

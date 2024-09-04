#pragma once

#include <memory>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"
#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/geometry/optimization/hyperellipsoid.h"
#include "drake/planning/collision_checker.h"
#include "drake/planning/iris/iris_interface.h"
#include "drake/planning/iris/iris_interface_options.h"
#include "drake/planning/iris/iris_via_collisions_and_ellipsoid_interface_options.h"

namespace drake {
namespace planning {
namespace internal {

// SFINAE that the options subclass must derive from the
// IrisViaCollisionsAndEllipsoidInterfaceOptions.
template <typename IrisViaCollisionsAndEllipsoidInterfaceOptionsSubclass,
          typename = std::enable_if_t<std::is_base_of<
              IrisViaCollisionsAndEllipsoidInterfaceOptions,
              IrisViaCollisionsAndEllipsoidInterfaceOptionsSubclass>::value>>
// template <typename IrisViaCollisionsAndEllipsoidInterfaceOptionsSubclass>
class IrisViaCollisionsAndEllipsoidInterface
    : public IrisInterface<
          IrisViaCollisionsAndEllipsoidInterfaceOptionsSubclass> {
 public:
  virtual ~IrisViaCollisionsAndEllipsoidInterface();

 protected:
  // Do not hide IrisInterface's pure virtual functions.
  using IrisInterface<IrisViaCollisionsAndEllipsoidInterfaceOptionsSubclass>::
      DoImproveRegionHyperplanes;
  using IrisInterface<
      IrisViaCollisionsAndEllipsoidInterfaceOptionsSubclass>::DoUpdateMetric;
  using IrisInterface<
      IrisViaCollisionsAndEllipsoidInterfaceOptionsSubclass>::DoSetup;

  IrisViaCollisionsAndEllipsoidInterface(const CollisionChecker& checker)
      : IrisInterface<IrisViaCollisionsAndEllipsoidInterfaceOptionsSubclass>(
            checker){
            // TODO complete constructor. This will essentially implement
            // MakeIrisObstacles from geometry/optimization/iris.h
        };

  /** Each column of the returned matrix is a point in set which contains a
   * collision */
  virtual Eigen::MatrixXd FindCollisionPoints(
      const IrisViaCollisionsAndEllipsoidInterfaceOptions& options,
      const geometry::optimization::HPolyhedron& set) = 0;

  /** Given a set of points in collision that are inside sets, add hyperplanes
   * to set to exclude those collisions. */
  virtual void AddHyperplanesAtPoints(
      const Eigen::Ref<const Eigen::MatrixXd>& points,
      const IrisViaCollisionsAndEllipsoidInterfaceOptions& options,
      geometry::optimization::HPolyhedron* set) const = 0;

 private:
  void DoImproveRegionHyperplanes(
      const IrisViaCollisionsAndEllipsoidInterfaceOptions& options,
      geometry::optimization::HPolyhedron* set) {
    while (HPolyhedronIsCollisionFreeViaUnadaptiveTest(*set, options)) {
      const Eigen::MatrixXd collision_points =
          FindCollisionPoints(options, *set);
      AddHyperplanesAtPoints(collision_points, options, set);
    }
  };

  void DoUpdateMetric(
      const IrisViaCollisionsAndEllipsoidInterfaceOptions& options,
      const geometry::optimization::HPolyhedron& set) {
    unused(options);
    ellipsoid_metric = set.MaximumVolumeInscribedEllipsoid();
  };

  void DoSetup(const IrisViaCollisionsAndEllipsoidInterfaceOptions& options,
               geometry::optimization::HPolyhedron* set) {
    unused(set);
    DRAKE_THROW_UNLESS(options.seed.rows() ==
                       this->checker_->plant().num_positions());
    for (int i = 0; i < this->checker_->num_allocated_contexts(); ++i) {
      this->checker_->UpdatePositions(options.seed, i);
    }
    const double kEpsilonEllipsoid = 1e-2;
    ellipsoid_metric = options.starting_ellipse.value_or(
        geometry::optimization::Hyperellipsoid::MakeHypersphere(
            kEpsilonEllipsoid, options.seed));
  };

  bool HPolyhedronIsCollisionFreeViaUnadaptiveTest(
      const geometry::optimization::HPolyhedron& set,
      const IrisViaCollisionsAndEllipsoidInterfaceOptions& options) const {
    unused(set);
    unused(options);
    // TODO implement
    throw std::logic_error("Unimplemented");
    return true;
  };

  geometry::optimization::Hyperellipsoid ellipsoid_metric;
};

}  // namespace internal
}  // namespace planning
}  // namespace drake
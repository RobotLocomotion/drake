#include "planning/iris/internal/iris_via_collisions_and_ellipsoid_interface.h"

#include "planning/iris/internal/iris_via_collisions_and_ellipsoid_interface_options.h"

#include "drake/common/drake_throw.h"
#include "drake/geometry/optimization/hyperellipsoid.h"

namespace drake {
namespace planning {
namespace internal {
IrisViaCollisionsAndEllipsoidInterface::IrisViaCollisionsAndEllipsoidInterface(
    const CollisionChecker& checker)
    : IrisInterface<IrisViaCollisionsAndEllipsoidInterfaceOptions>(checker){
          // TODO complete constructor. This will essentially implement
          // MakeIrisObstacles from geometry/optimization/iris.h
      };

bool IrisViaCollisionsAndEllipsoidInterface::
    HPolyhedronIsCollisionFreeViaUnadaptiveTest(
        const geometry::optimization::HPolyhedron& set,
        const IrisViaCollisionsAndEllipsoidInterfaceOptions& options) const {
  unused(set);
  unused(options);
  // TODO implement
  throw std::logic_error("Unimplemented");
  return true;
};

void IrisViaCollisionsAndEllipsoidInterface::DoAddPlanesToSet(
    const IrisViaCollisionsAndEllipsoidInterfaceOptions& options,
    geometry::optimization::HPolyhedron* set) {
  while (HPolyhedronIsCollisionFreeViaUnadaptiveTest(*set, options)) {
    const Eigen::MatrixXd collision_points = FindCollisionPoints(options, *set);
    AddHyperplanesAtPoints(collision_points, options, set);
  }
};

void IrisViaCollisionsAndEllipsoidInterface::DoUpdateMetric(
    const IrisViaCollisionsAndEllipsoidInterfaceOptions& options,
    const geometry::optimization::HPolyhedron& set) {
  unused(options);
  ellipsoid_metric = set.MaximumVolumeInscribedEllipsoid();
}

void IrisViaCollisionsAndEllipsoidInterface::DoSetup(
    const IrisViaCollisionsAndEllipsoidInterfaceOptions& options,
    geometry::optimization::HPolyhedron* set) {
  unused(set);
  DRAKE_THROW_UNLESS(options.seed.rows() == checker_->plant().num_positions());
  for (int i = 0; i < checker_->num_allocated_contexts(); ++i) {
    checker_->UpdatePositions(options.seed, i);
  }
  const double kEpsilonEllipsoid = 1e-2;
  ellipsoid_metric = options.starting_ellipse.value_or(
      geometry::optimization::Hyperellipsoid::MakeHypersphere(kEpsilonEllipsoid,
                                                              options.seed));
}
}  // namespace internal
}  // namespace planning
}  // namespace drake
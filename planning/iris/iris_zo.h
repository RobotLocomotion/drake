#pragma once

#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/planning/collision_checker.h"
#include "drake/planning/iris/iris_via_collisions_and_ellipsoid_interface.h"
#include "drake/planning/iris/iris_via_collisions_and_ellipsoid_interface_options.h"

namespace drake {
namespace planning {

struct IrisZoOptions
    : internal::IrisViaCollisionsAndEllipsoidInterfaceOptions {
  // TODO add all the unique options.
};

class IrisZo final
    : public internal::IrisViaCollisionsAndEllipsoidInterface<IrisZoOptions> {
 public:
  explicit IrisZo(const CollisionChecker& checker);

 private:
  // Do not hide IrisViaCollisionsAndEllipsoidInterface's pure virtual
  // functions.
  using internal::IrisViaCollisionsAndEllipsoidInterface<
      IrisZoOptions>::FindCollisionPoints;
  using internal::IrisViaCollisionsAndEllipsoidInterface<
      IrisZoOptions>::AddHyperplanesAtPoints;

  Eigen::MatrixXd FindCollisionPoints(
      const IrisZoOptions& options,
      const geometry::optimization::HPolyhedron& set);

  void AddHyperplanesAtPoints(const Eigen::Ref<const Eigen::MatrixXd>& points,
                              const IrisZoOptions& options,
                              geometry::optimization::HPolyhedron* set) const;
};

}  // namespace planning
}  // namespace drake

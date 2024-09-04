#pragma once

#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/planning/collision_checker.h"
#include "drake/planning/iris/iris_via_collisions_and_ellipsoid_interface.h"
#include "drake/planning/iris/iris_via_collisions_and_ellipsoid_interface_options.h"

namespace drake {
namespace planning {

struct VanillaIrisOptions
    : internal::IrisViaCollisionsAndEllipsoidInterfaceOptions {
  // TODO add all the unique options.
};

class VanillaIris final
    : public internal::IrisViaCollisionsAndEllipsoidInterface<
          VanillaIrisOptions> {
 public:
  explicit VanillaIris(const CollisionChecker& checker);

 private:
  // Do not hide IrisViaCollisionsAndEllipsoidInterface's pure virtual
  // functions.
  using internal::IrisViaCollisionsAndEllipsoidInterface<
      VanillaIrisOptions>::FindCollisionPoints;
  using internal::IrisViaCollisionsAndEllipsoidInterface<
      VanillaIrisOptions>::AddHyperplanesAtPoints;

  // Finds the closest point on the closest obstacle to the seed point that is
  // inside set.
  Eigen::MatrixXd FindCollisionPoints(
      const VanillaIrisOptions& options,
      const geometry::optimization::HPolyhedron& set);

  // Adds a hyperplane tangent to the ellipsoid at points.
  void AddHyperplanesAtPoints(const Eigen::Ref<const Eigen::MatrixXd>& points,
                              const VanillaIrisOptions& options,
                              geometry::optimization::HPolyhedron* set) const;
};

}  // namespace planning
}  // namespace drake

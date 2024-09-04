#pragma once

#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/planning/collision_checker.h"
#include "drake/planning/iris/internal/iris_via_collisions_and_ellipsoid_interface.h"
#include "drake/planning/iris/internal/iris_via_collisions_and_ellipsoid_interface_options.h"

namespace drake {
namespace planning {

struct IrisNp2Options : IrisViaCollisionsAndEllipsoidInterfaceOptions {
  // TODO add all the unique options.
};

class IrisNp2 : IrisViaCollisionsAndEllipsoidInterface<
                    IrisViaCollisionsAndEllipsoidInterfaceOptions> {
 public:
  explicit IrisNp2(const CollisionChecker& checker);

 private:
  Eigen::VectorXd FindCollisionPoints(
      const IrisViaCollisionsAndEllipsoidInterfaceOptions& options,
      const geometry::optimization::HPolyhedron& set);

  void AddHyperplanesAtPoints(
      const Eigen::Ref<const Eigen::VectorXd>& points,
      const IrisViaCollisionsAndEllipsoidInterfaceOptions& options,
      geometry::optimization::HPolyhedron* set) const;
};

}  // namespace planning

}  // namespace drake
#pragma once

#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/planning/scene_graph_collision_checker.h"
#include "drake/planning/iris/iris_via_collisions_and_ellipsoid_interface.h"
#include "drake/planning/iris/iris_via_collisions_and_ellipsoid_interface_options.h"

namespace drake {
namespace planning {

struct IrisNp2Options
    : internal::IrisViaCollisionsAndEllipsoidInterfaceOptions {
  // TODO add all the unique options.
};

class IrisNp2 final
    : public internal::IrisViaCollisionsAndEllipsoidInterface<IrisNp2Options> {
 public:
  explicit IrisNp2(const SceneGraphCollisionChecker& checker);

 private:
  // Do not hide IrisViaCollisionsAndEllipsoidInterface's pure virtual
  // functions.
  using internal::IrisViaCollisionsAndEllipsoidInterface<
      IrisNp2Options>::FindCollisionPoints;
  using internal::IrisViaCollisionsAndEllipsoidInterface<
      IrisNp2Options>::AddHyperplanesAtPoints;

  Eigen::MatrixXd FindCollisionPoints(
      const IrisNp2Options& options,
      const geometry::optimization::HPolyhedron& set);

  void AddHyperplanesAtPoints(const Eigen::Ref<const Eigen::MatrixXd>& points,
                              const IrisNp2Options& options,
                              geometry::optimization::HPolyhedron* set) const;
};

}  // namespace planning
}  // namespace drake

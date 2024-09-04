#pragma once

#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/planning/scene_graph_collision_checker.h"
#include "drake/planning/iris/iris_via_collisions_and_ellipsoid_interface.h"
#include "drake/planning/iris/iris_via_collisions_and_ellipsoid_interface_options.h"

namespace drake {
namespace planning {

struct IrisNpOptions
    : internal::IrisViaCollisionsAndEllipsoidInterfaceOptions {
  // TODO add all the unique options.
};

class IrisNp final
    : public internal::IrisViaCollisionsAndEllipsoidInterface<IrisNpOptions> {
 public:
  explicit IrisNp(const SceneGraphCollisionChecker& checker);

 private:
  // Do not hide IrisViaCollisionsAndEllipsoidInterface's pure virtual
  // functions.
  using internal::IrisViaCollisionsAndEllipsoidInterface<
      IrisNpOptions>::FindCollisionPoints;
  using internal::IrisViaCollisionsAndEllipsoidInterface<
      IrisNpOptions>::AddHyperplanesAtPoints;

  Eigen::MatrixXd FindCollisionPoints(
      const IrisNpOptions& options,
      const geometry::optimization::HPolyhedron& set);

  void AddHyperplanesAtPoints(const Eigen::Ref<const Eigen::MatrixXd>& points,
                              const IrisNpOptions& options,
                              geometry::optimization::HPolyhedron* set) const;
};

}  // namespace planning

}  // namespace drake
#include "drake/planning/iris/vanilla_iris.h"
namespace drake {
namespace planning {

Eigen::MatrixXd VanillaIris::FindCollisionPoints(
    const VanillaIrisOptions& options,
    const geometry::optimization::HPolyhedron& set) {
  unused(options, set);
  throw std::logic_error("unimplemented");
}

void VanillaIris::AddHyperplanesAtPoints(
    const Eigen::Ref<const Eigen::MatrixXd>& points,
    const VanillaIrisOptions& options,
    geometry::optimization::HPolyhedron* set) const {
  unused(points, options, set);
  throw std::logic_error("unimplemented");
}

}  // namespace planning
}  // namespace drake
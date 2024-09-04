#include "drake/planning/iris/iris_zo.h"

namespace drake {
namespace planning {

IrisZo::IrisZo(const CollisionChecker& checker)
    : IrisViaCollisionsAndEllipsoidInterface<IrisZoOptions>(checker){};

Eigen::MatrixXd IrisZo::FindCollisionPoints(
    const IrisZoOptions& options,
    const geometry::optimization::HPolyhedron& set) {
  unused(options, set);
  throw std::logic_error("unimplemented");
  return Eigen::VectorXd::Zero(checker_->plant().num_positions());
}

void IrisZo::AddHyperplanesAtPoints(
    const Eigen::Ref<const Eigen::MatrixXd>& points,
    const IrisZoOptions& options,
    geometry::optimization::HPolyhedron* set) const {
  unused(points, options, set);
  throw std::logic_error("unimplemented");
}

}  // namespace planning
}  // namespace drake

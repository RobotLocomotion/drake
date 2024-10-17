#include "drake/planning/iris/iris_np2.h"

namespace drake {
namespace planning {

IrisNp2::IrisNp2(const SceneGraphCollisionChecker& checker)
    : IrisViaCollisionsAndEllipsoidInterface<IrisNp2Options>(checker){};

Eigen::MatrixXd IrisNp2::FindCollisionPoints(
    const IrisNp2Options& options,
    const geometry::optimization::HPolyhedron& set) {
  unused(options, set);
  throw std::logic_error("unimplemented");
  return Eigen::VectorXd::Zero(checker_->plant().num_positions());
}

void IrisNp2::AddHyperplanesAtPoints(
    const Eigen::Ref<const Eigen::MatrixXd>& points,
    const IrisNp2Options& options,
    geometry::optimization::HPolyhedron* set) const {
  unused(points, options, set);
  throw std::logic_error("unimplemented");
}

}  // namespace planning
}  // namespace drake

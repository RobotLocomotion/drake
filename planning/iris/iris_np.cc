#include "drake/planning/iris/iris_np.h"

namespace drake {
namespace planning {

IrisNp::IrisNp(const SceneGraphCollisionChecker& checker)
    : IrisViaCollisionsAndEllipsoidInterface<IrisNpOptions>(checker){};

Eigen::MatrixXd IrisNp::FindCollisionPoints(
    const IrisNpOptions& options,
    const geometry::optimization::HPolyhedron& set) {
  unused(options, set);
  throw std::logic_error("unimplemented");
  return Eigen::VectorXd::Zero(checker_->plant().num_positions());
}

void IrisNp::AddHyperplanesAtPoints(
    const Eigen::Ref<const Eigen::MatrixXd>& points,
    const IrisNpOptions& options,
    geometry::optimization::HPolyhedron* set) const {
  unused(points, options, set);
  throw std::logic_error("unimplemented");
}

}  // namespace planning
}  // namespace drake

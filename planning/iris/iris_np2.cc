#include "drake/planning/iris/iris_np2.h"

namespace drake {
namespace planning {

IrisNp2::IrisNp2(const CollisionChecker& checker)
    : IrisViaCollisionsAndEllipsoidInterface<IrisNp2Options>(checker);

IrisNp2::FindCollisionPoints(
    const IrisViaCollisionsAndEllipsoidInterfaceOptions& options,
    const geometry::optimization::HPolyhedron& set) {
  unused(options);
  unused(set);
  throw std::logic_error("unimplemented");
  return Eigen::VectorXd::Zeros(checker_->plant().num_positions());
}

IrisNp2::AddHyperplanesAtPoints(
    const Eigen::Ref<const Eigen::VectorXd>& points,
    const IrisViaCollisionsAndEllipsoidInterfaceOptions& options,
    geometry::optimization::HPolyhedron* set) const {
  unused(points, options, set);
  throw std::logic_error("unimplemented");
}

}  // namespace planning
}  // namespace drake

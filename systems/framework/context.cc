#include "drake/systems/framework/context.h"

namespace drake {
namespace systems {

// The Vector2/3 instantiations here are for the benefit of some
// older unit tests but are not otherwise advertised.
template class Context<Eigen::AutoDiffScalar<Eigen::Vector2d>>;
template class Context<Eigen::AutoDiffScalar<Eigen::Vector3d>>;

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    struct ::drake::systems::StepInfo)

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::Context)

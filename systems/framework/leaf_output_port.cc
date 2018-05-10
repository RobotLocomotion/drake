#include "drake/systems/framework/leaf_output_port.h"

#include "drake/common/default_scalars.h"

namespace drake {
namespace systems {

// The Vector2/3 instantiations here are for the benefit of some
// older unit tests but are not otherwise advertised.
template class LeafOutputPort<Eigen::AutoDiffScalar<Eigen::Vector2d>>;
template class LeafOutputPort<Eigen::AutoDiffScalar<Eigen::Vector3d>>;

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::LeafOutputPort)

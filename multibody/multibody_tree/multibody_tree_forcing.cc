#include "drake/multibody/multibody_tree/multibody_tree_forcing.h"

#include "drake/common/default_scalars.h"
#include "drake/multibody/multibody_tree/multibody_tree.h"

namespace drake {
namespace multibody {

template <typename T>
MultibodyTreeForcing<T>::MultibodyTreeForcing(const MultibodyTree<T>& model) :
    F_B_W_(model.get_num_bodies(), SpatialForce<T>::Zero()),
    tau_(VectorX<T>::Zero(model.get_num_velocities())) {}

}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::MultibodyTreeForcing)

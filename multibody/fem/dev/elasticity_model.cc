#include "drake/multibody/fem/dev/elasticity_model.h"

#include "drake/multibody/fem/dev/linear_simplex_element.h"

namespace drake {
namespace multibody {
namespace fem {
template <typename T>
T ElasticityModel<T>::CalcElasticEnergy(const FemState<T>& state) const {
  T energy(0);
  for (ElementIndex element_index(0); element_index < this->num_elements();
       ++element_index) {
    const ElasticityElementBase<T>& e =
        this->get_elasticity_element_base(element_index);
    energy += e.CalcElasticEnergy(state);
  }
  return energy;
}

template <typename T>
std::unique_ptr<FemState<T>> ElasticityModel<T>::DoMakeFemState() const {
  const int num_nodes = this->num_nodes();
  VectorX<T> q(this->num_dofs());
  for (int i = 0; i < num_nodes; ++i) {
    const Vector3<T>& q_WP = reference_positions_.at(NodeIndex(i));
    q.template segment<3>(3 * i) = q_WP;
  }
  /* Velocity of the nodes are set to zero. If nonzero initial velocity is
   desired, set it with SetInitialVelocity(). */
  // TODO(xuchenhan-tri): Add SetInitialVelocity() to support configuring
  // initial velocities.
  const VectorX<T> qdot = VectorX<T>::Zero(this->num_dofs());
  return std::make_unique<FemState<T>>(q, qdot);
}
}  // namespace fem
}  // namespace multibody
}  // namespace drake
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::fem::ElasticityModel);

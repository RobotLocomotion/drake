#include "drake/multibody/fem/dev/elasticity_model.h"

#include "drake/multibody/fem/dev/linear_simplex_element.h"

namespace drake {
namespace multibody {
namespace fem {
template <typename T>
T ElasticityModel<T>::CalcElasticEnergy(const FemState<T>& state) const {
  T energy(0);
  for (int i = 0; i < this->num_elements(); ++i) {
    const FemElement<T>& e = this->element(ElementIndex(i));
    energy += static_cast<const ElasticityElementBase<T>&>(e).CalcElasticEnergy(
        state);
  }
  return energy;
}

template <typename T>
std::unique_ptr<FemState<T>> ElasticityModel<T>::DoMakeFemState() const {
  this->ThrowIfNodeIndicesAreInvalid();
  const int num_nodes = this->num_nodes();
  VectorX<T> q(num_nodes * solution_dimension());
  for (int i = 0; i < num_nodes; ++i) {
      const Vector3<T>& q_WP = reference_positions_.at(NodeIndex(i));
      q.segment(3 * i, 3) = q_WP;
  }
  const VectorX<T> qdot = VectorX<T>::Zero(num_nodes * solution_dimension());
  return std::make_unique<FemState<T>>(q, qdot);
}
}  // namespace fem
}  // namespace multibody
}  // namespace drake
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::fem::ElasticityModel);

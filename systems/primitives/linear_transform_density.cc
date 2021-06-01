#include "drake/systems/primitives/linear_transform_density.h"

#include "drake/common/default_scalars.h"
#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace systems {

template <typename T>
LinearTransformDensity<T>::LinearTransformDensity(
    RandomDistribution distribution, int input_size, int output_size)
    : LeafSystem<T>(SystemTypeTag<LinearTransformDensity>{}),
      distribution_{distribution},
      input_size_{input_size},
      output_size_{output_size} {
  w_in_port_id_ =
      this->DeclareInputPort("w_in", kVectorValued, input_size_).get_index();
  A_port_id_ =
      this->DeclareInputPort("A", kVectorValued, input_size_ * output_size_)
          .get_index();
  b_port_id_ =
      this->DeclareInputPort("b", kVectorValued, output_size_).get_index();

  this->DeclareVectorOutputPort("w_out", BasicVector<T>(output_size_),
                                &LinearTransformDensity<T>::CalcOutput);
}

template <typename T>
template <typename U>
LinearTransformDensity<T>::LinearTransformDensity(
    const LinearTransformDensity<U>& other)
    : LinearTransformDensity<T>(other.get_distribution(),
                                other.get_input_port_w_in().size(),
                                other.get_input_port_b().size()) {}

template <typename T>
void LinearTransformDensity<T>::CalcOutput(const Context<T>& context,
                                           BasicVector<T>* w_out) const {
  const auto w_in = this->get_input_port_w_in().Eval(context);
  const Eigen::VectorBlock<const VectorX<T>> A_flat =
      this->get_input_port_A().Eval(context);
  const auto b = this->get_input_port_b().Eval(context);
  const Eigen::Map<const MatrixX<T>> A(A_flat.data(), output_size_,
                                       input_size_);
  w_out->get_mutable_value() = A * w_in + b;
}
}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::systems::LinearTransformDensity)

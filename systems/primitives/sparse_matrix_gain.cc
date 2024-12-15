#include "drake/systems/primitives/sparse_matrix_gain.h"

#include "drake/common/default_scalars.h"

namespace drake {
namespace systems {

template <typename T>
SparseMatrixGain<T>::SparseMatrixGain(const Eigen::SparseMatrix<double>& D)
    : LeafSystem<T>(SystemTypeTag<SparseMatrixGain>{}), D_(D) {
  InputPortIndex u_index =
      this->DeclareVectorInputPort("u", D.cols()).get_index();
  this->DeclareVectorOutputPort("y", D.rows(), &SparseMatrixGain<T>::CalcOutput,
                                {this->input_port_ticket(u_index)});
}

template <typename T>
SparseMatrixGain<T>::~SparseMatrixGain() = default;

template <typename T>
template <typename U>
SparseMatrixGain<T>::SparseMatrixGain(const SparseMatrixGain<U>& other)
    : SparseMatrixGain<T>(other.D()) {}

template <typename T>
void SparseMatrixGain<T>::CalcOutput(const Context<T>& context,
                                     BasicVector<T>* output_vector) const {
  DRAKE_THROW_UNLESS(this->get_input_port().HasValue(context));
  const BasicVector<T>* u = this->EvalVectorInput(context, 0);
  DRAKE_DEMAND(u != nullptr);
  output_vector->SetFromVector(D_ * u->value());
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::SparseMatrixGain);

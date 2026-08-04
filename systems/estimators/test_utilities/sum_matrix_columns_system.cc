#include "drake/systems/estimators/test_utilities/sum_matrix_columns_system.h"

namespace drake {
namespace systems {
namespace estimators_test {

template <typename T>
SumMatrixColumnsSystem<T>::SumMatrixColumnsSystem(int rows, int cols)
    : LeafSystem<T>(SystemTypeTag<SumMatrixColumnsSystem>{}),
      rows_(rows),
      cols_(cols) {
  DRAKE_THROW_UNLESS(rows > 0);
  DRAKE_THROW_UNLESS(cols > 0);
  Eigen::MatrixX<T> model_value = Eigen::MatrixX<T>::Zero(rows, cols);
  this->DeclareAbstractInputPort(kUseDefaultName, Value(model_value));
  this->DeclareVectorOutputPort(
      kUseDefaultName, rows,
      [this](const Context<T>& context, BasicVector<T>* out) {
        out->set_value(
            Eigen::VectorX<T>(this->get_input_port()
                                  .template Eval<Eigen::MatrixX<T>>(context)
                                  .rowwise()
                                  .sum()));
      });
}

template <typename T>
template <typename U>
SumMatrixColumnsSystem<T>::SumMatrixColumnsSystem(
    const SumMatrixColumnsSystem<U>& other)
    : SumMatrixColumnsSystem(other.rows_, other.cols_) {}

template <typename T>
SumMatrixColumnsSystem<T>::~SumMatrixColumnsSystem() = default;

}  // namespace estimators_test
}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::estimators_test::SumMatrixColumnsSystem);

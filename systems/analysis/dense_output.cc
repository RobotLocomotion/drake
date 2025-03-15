#include "drake/systems/analysis/dense_output.h"

namespace drake {
namespace systems {

template <typename T>
DenseOutput<T>::~DenseOutput() = default;

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::systems::DenseOutput);

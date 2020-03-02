#include "drake/systems/primitives/barycentric_system.h"

#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/common/unused.h"

namespace drake {
namespace systems {

template <typename T>
BarycentricMeshSystem<T>::BarycentricMeshSystem(
    math::BarycentricMesh<T> mesh,
    const Eigen::Ref<const MatrixX<T>>& output_values)
    : VectorSystem<T>(mesh.get_input_size(), output_values.rows()),
      mesh_(std::move(mesh)),
      output_values_(output_values) {
  DRAKE_DEMAND(output_values_.rows() > 0);
  DRAKE_DEMAND(output_values_.cols() == mesh_.get_num_mesh_points());
}

template <typename T>
void BarycentricMeshSystem<T>::DoCalcVectorOutput(
      const Context<T>& context,
      const Eigen::VectorBlock<const VectorX<T>>& input,
      const Eigen::VectorBlock<const VectorX<T>>& state,
      Eigen::VectorBlock<VectorX<T>>* output) const {
    unused(context, state);
    mesh_.Eval(output_values_, input, output);
  }

}  // namespace systems
}  // namespace drake

template class ::drake::systems::BarycentricMeshSystem<double>;

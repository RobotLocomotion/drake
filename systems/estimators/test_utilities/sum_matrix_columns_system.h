#pragma once

#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {
namespace estimators_test {

/**
 * A system that outputs the sum of the columns of the matrix on the input.
 *
 * @system
 * name: SumMatrixColumnsSystem
 * input_ports:
 * - u0
 * output_ports:
 * - y0
 * @endsystem
 *
 * @tparam_default_scalar
 */
template <typename T>
class SumMatrixColumnsSystem final : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SumMatrixColumnsSystem);

  /**
   * Constructs a system with an abstract input port expecting an
   * Eigen::MatrixX<T> of size `rows` × `cols`. The system outputs the sum of
   * the columns (column 1 + column 2 + column 3 + ...) as a single vector.
   */
  SumMatrixColumnsSystem(int rows, int cols);

  /** Scalar-converting copy constructor.  See @ref system_scalar_conversion. */
  template <typename U>
  SumMatrixColumnsSystem(const SumMatrixColumnsSystem<U>& other);

  ~SumMatrixColumnsSystem() override;

 private:
  template <typename U>
  friend class SumMatrixColumnsSystem;

  const int rows_;
  const int cols_;
};

}  // namespace estimators_test
}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::estimators_test::SumMatrixColumnsSystem);

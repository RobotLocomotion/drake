#pragma once

#include <Eigen/SparseCore>

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {

// TODO(russt): Consider deriving from LinearSystem, but see the draft and
// discussion at https://github.com/RobotLocomotion/drake/pull/21591 which
// emphasized that we need to avoid performance degredation for dense matrices
// in AffineSystem.

/// A variant of MatrixGain which supports multiplication by SparseMatrix, `D`.
/// Specifically, given an input signal `u` and a state `x`, the output of this
/// system, `y`, is:
///
/// @f[
///   y = D u
/// @f]
///
/// Note that, unlike MatrixGain, this system is not derived from LinearSystem
/// (which does not yet support sparse matrices). However, we use `D` as the
/// name for the gain matrix here to be consistent.
///
/// @system
/// name: SparseMatrixGain
/// input_ports:
/// - u
/// output_ports:
/// - y
/// @endsystem
///
/// @tparam_default_scalar
/// @ingroup primitive_systems
///
/// @see MatrixGain
template <typename T>
class SparseMatrixGain final : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SparseMatrixGain);

  /// A constructor where the gain matrix `D` is @p D.
  explicit SparseMatrixGain(const Eigen::SparseMatrix<double>& D);

  /// Scalar-converting copy constructor.  See @ref system_scalar_conversion.
  template <typename U>
  explicit SparseMatrixGain(const SparseMatrixGain<U>&);

  ~SparseMatrixGain() final;

  /// Getter for the gain matrix `D`.
  const Eigen::SparseMatrix<double>& D() const { return D_; }

  // Note: we use a setter instead of returning a mutable D to simplify the
  // python bindings.

  /// Setter for the gain matrix `D`.
  void set_D(const Eigen::SparseMatrix<double>& D) { D_ = D; }

 private:
  void CalcOutput(const Context<T>& context,
                  BasicVector<T>* output_vector) const;

  Eigen::SparseMatrix<double> D_{};
};

}  // namespace systems
}  // namespace drake

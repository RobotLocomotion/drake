#pragma once

#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {

/// A continuous affine system. Given an input vector `u`, an output
/// vector 'y', a state vector `x`, its derivative @f['\dot{x}'@f] and
/// state space coefficient matrices `A`, `B`, `C`, and `D`, this system
/// implements the following equations:
/// @f[
///   \dot{x} = Ax + Bu + \dot{x}_0 \newline
///   y = Cx + Du + y_0
/// @f]
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - AutoDiffXd
///
/// They are already available to link against in libdrakeSystemFramework.
/// No other values for T are currently supported.
/// @ingroup primitive_systems
template <typename T>
class AffineSystem : public LeafSystem<T> {
 public:
  /// Constructs an Affine system with a fixed set of coefficient matrices `A`,
  /// `B`,`C`, and `D` as well as fixed initial velocity offset `xDot0` and
  /// output offset `y0`.
  /// The coefficient matrices must obey the following dimensions :
  /// | Matrix  | Num Rows    | Num Columns |
  /// |:-------:|:-----------:|:-----------:|
  /// | A       | num states  | num states  |
  /// | B       | num states  | num inputs  |
  /// | C       | num outputs | num states  |
  /// | D       | num outputs | num inputs  |
  AffineSystem(const Eigen::Ref<const MatrixX<T>>& A,
               const Eigen::Ref<const MatrixX<T>>& B,
               const Eigen::Ref<const VectorX<T>>& xDot0,
               const Eigen::Ref<const MatrixX<T>>& C,
               const Eigen::Ref<const MatrixX<T>>& D,
               const Eigen::Ref<const VectorX<T>>& y0);

  /// The input to this system is direct feedthrough only if the coefficient
  /// matrix `D` is zero.
  bool has_any_direct_feedthrough() const override { return !D_.isZero(); }

  /// Returns the input port containing the externally applied input.
  const SystemPortDescriptor<T>& get_input_port() const;

  /// Returns the port containing the output state.
  const SystemPortDescriptor<T>& get_output_port() const;

  void EvalOutput(const Context<T>& context,
                  SystemOutput<T>* output) const override;

  void EvalTimeDerivatives(const Context<T>& context,
                           ContinuousState<T>* derivatives) const override;

  // Helper getter methods.
  const Eigen::Ref<const MatrixX<T>> GetA(void) const { return A_; }
  const Eigen::Ref<const MatrixX<T>> GetB(void) const { return B_; }
  const Eigen::Ref<const MatrixX<T>> GetC(void) const { return C_; }
  const Eigen::Ref<const MatrixX<T>> GetD(void) const { return D_; }
  const Eigen::Ref<const VectorX<T>> GetXDot0(void) const { return XDot0_; }
  const Eigen::Ref<const VectorX<T>> GetY0(void) const { return Y0_; }

 private:
  const MatrixX<T> A_;
  const MatrixX<T> B_;
  const VectorX<T> XDot0_;
  const MatrixX<T> C_;
  const MatrixX<T> D_;
  const VectorX<T> Y0_;
  const int kNumInputs;
  const int kNumOutputs;
  const int kNumStates;
};

}  // namespace systems
}  // namespace drake

#pragma once

#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {

/// An affine system. An Affine system. Given an input vector u, a state vector
/// x, and state space coefficient matrices A, B, C, and D, this system
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
class AffineSystemPlant : public LeafSystem<T> {
 public:
  /// Constructs an Affine system with a fixed set of coefficient matrices `A`,
  /// `B`,`C`, and `D` as well as fixed initial velocity offset `xDot0` and
  /// output offset `y0`.
  /// Note that A must be a square matrix where number of Rows / Columns must
  /// match that on B and on xDot0. The number of rows on C must equal the
  /// number of Rows on D and Y0.
  AffineSystemPlant(const Eigen::Ref<const MatrixX<T>>& A,
                    const Eigen::Ref<const MatrixX<T>>& B,
                    const Eigen::Ref<const VectorX<T>>& xDot0,
                    const Eigen::Ref<const MatrixX<T>>& C,
                    const Eigen::Ref<const MatrixX<T>>& D,
                    const Eigen::Ref<const VectorX<T>>& y0);

  /// The input to this system is not direct feedthrough.
  bool has_any_direct_feedthrough() const override { return !D_.isZero(); }

  /// LeafSystem override.
  std::unique_ptr<ContinuousState<T>>
  AllocateContinuousState() const override;

  /// Returns the input port to the externally applied input.
  const SystemPortDescriptor<T>& get_input_port() const;

  /// Returns the port to output state.
  const SystemPortDescriptor<T>& get_output_port() const;

  void EvalOutput(const Context<T>& context,
                  SystemOutput<T>* output) const override;

  void EvalTimeDerivatives(const Context<T>& context,
                           ContinuousState<T>* derivatives) const override;

  // Helper getter methods.
  const MatrixX<T> GetA(void) const { return A_; }
  const MatrixX<T> GetB(void) const { return B_; }
  const MatrixX<T> GetC(void) const { return C_; }
  const MatrixX<T> GetD(void) const { return D_; }
  const VectorX<T> GetXDot0(void) const { return XDot0_; }
  const VectorX<T> GetY0(void) const { return Y0_; }

 private:
  const MatrixX<T> A_;
  const MatrixX<T> B_;
  const MatrixX<T> C_;
  const MatrixX<T> D_;
  const VectorX<T> XDot0_;
  const VectorX<T> Y0_;
  const int kNumInputs;
  const int kNumOutputs;
  const int kNumStates;
};

}  // namespace systems
}  // namespace drake

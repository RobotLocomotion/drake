#pragma once

#include "drake/common/drake_export.h"
#include "drake/common/eigen_types.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/system_output.h"

namespace drake {
namespace systems {

/// An affine system. Given an input signal `u` and a state `x`
/// the output of this sytem is
/// <pre>
///   xDot = Ax + Bu + xDot0
///   y = Cx + Du + y0
/// </pre>
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - AutoDiffXd
///
/// They are already available to link against in libdrakeSystemFramework.
/// No other values for T are currently supported.
/// @ingroup systems

template <typename T>
class DRAKE_EXPORT AffineSystemPlant : public LeafSystem<T> {
 public:
  using MyContext = Context<T>;
  using MyContinuousState = ContinuousState<T>;
  using MyOutput = SystemOutput<T>;

  /// Construct a Affine system with a fixed set of matrices `A`, `B`,`C`, and
  /// `D` as well as fixed initial velocity offset xd0
  AffineSystemPlant(const Eigen::Ref<const VectorX<T>>& xdot0,
                    const Eigen::Ref<const MatrixX<T>>& A,
                    const Eigen::Ref<const MatrixX<T>>& B,
                    const Eigen::Ref<const MatrixX<T>>& C,
                    const Eigen::Ref<const MatrixX<T>>& D,
                    const Eigen::Ref<const VectorX<T>>& y0);

  /// The input force to this system is not direct feedthrough.
  virtual bool has_any_direct_feedthrough() const override { return false; }

  /// LeafSystem override.
  virtual std::unique_ptr<ContinuousState<T>>
  AllocateContinuousState() const override;

  /// Returns the input port to the externally applied input.
  virtual const SystemPortDescriptor<T>& get_input_port() const;

  /// Returns the port to output state.
  virtual const SystemPortDescriptor<T>& get_output_port() const;

  /// @returns the external driving force to the system.
  virtual T get_input_force(const MyContext& context) const {
    T external_force = 0;
    external_force = this->EvalVectorInput(context, 0)->GetAtIndex(0);
    return external_force;
  }

  /// Sets the continuous state vector of the system to be @p x.
  virtual void set_state_vector(Context<T>* context,
                        const Eigen::Ref<const VectorX<T>> x) const;
  /// System<T> overrides.
  /// Allocates a single output port of type SystemOutput<T>.
  virtual std::unique_ptr<MyOutput> AllocateOutput(
      const MyContext& context) const override;

  void EvalOutput(const MyContext& context, MyOutput* output) const override;

  void EvalTimeDerivatives(const MyContext& context,
                           MyContinuousState* derivatives) const override;

  // Helper getter methods.
  const MatrixX<T> GetA(void) const { return kA; }
  const MatrixX<T> GetB(void) const { return kB; }
  const MatrixX<T> GetC(void) const { return kC; }
  const MatrixX<T> GetD(void) const { return kD; }
  const VectorX<T> GetXDot0(void) const { return kXDot0; }
  const VectorX<T> GetY0(void) const { return kY0; }

 private:
  const MatrixX<T> kA;
  const MatrixX<T> kB;
  const MatrixX<T> kC;
  const MatrixX<T> kD;
  const VectorX<T> kXDot0;
  const VectorX<T> kY0;
  const int kNumInputs;
  const int kNumOutputs;
  const int kNumStates;
};

}  // namespace systems
}  // namespace drake

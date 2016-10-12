#pragma once

#include <memory>

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
///   xDot = Ax + Bu + x0
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
  /// Construct a Affine system with a fixed spring constant and given
  /// mass.
  /// @param[in] name The name of the system.
  /// @param[in] spring_constant_N_per_m The spring constant in N/m.
  /// @param[in] mass_Kg The actual value in Kg of the mass attached to the
  /// spring.
  /// @param[in] system_is_forced If `true`, the system has an input port for an
  /// external force. If `false`, the system has no inputs.

  using MyContext = Context<T>;
  using MyContinuousState = ContinuousState<T>;
  using MyOutput = SystemOutput<T>;

  AffineSystemPlant(const Eigen::Ref<const VectorX<T>>& x0,
                    const Eigen::Ref<const MatrixX<T>>& A,
                    const Eigen::Ref<const MatrixX<T>>& B,
                    const Eigen::Ref<const MatrixX<T>>& C,
                    const Eigen::Ref<const MatrixX<T>>& D,
                    const Eigen::Ref<const VectorX<T>>& y0);

  /// The input force to this system is not direct feedthrough.
  bool has_any_direct_feedthrough() const override { return false; }

  /// Returns the input port to the externally applied force.
   const SystemPortDescriptor<T>& get_force_port() const;

  /// Returns the port to output state.
   const SystemPortDescriptor<T>& get_output_port() const;

  /// @returns the external driving force to the system.
  T get_input_force(const MyContext& context) const {
    T external_force = 0;
    external_force = this->EvalVectorInput(context, 0)->GetAtIndex(0);
    return external_force;
  }

  /// Sets the continuous state vector of the system to be @p x.
  void set_state_vector(Context<T>* context,
                        const Eigen::Ref<const VectorX<T>> x) const;
  //// System<T> overrides.
  ///// Allocates a single output port of type SpringMassStateVector<T>.
  std::unique_ptr<MyOutput> AllocateOutput(
      const MyContext& context) const override;

  void EvalOutput(const MyContext& context, MyOutput* output) const override;

  void EvalTimeDerivatives(const MyContext& context,
                           MyContinuousState* derivatives) const override;

 protected:
  std::unique_ptr<ContinuousState<T>> AllocateContinuousState() const override;

 private:
  const MatrixX<T> kA;
  const MatrixX<T> kB;
  const MatrixX<T> kC;
  const MatrixX<T> kD;
  const VectorX<T> kX0;
  const VectorX<T> kY0;
  const int kNumInputs;
  const int kNumOutputs;
  const int kNumStates;
};

}  // namespace systems
}  // namespace drake

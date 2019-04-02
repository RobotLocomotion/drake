#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_optional.h"
#include "drake/common/eigen_types.h"
#include "drake/common/symbolic.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {

/// A LeafSystem that is defined by vectors of symbolic::Expression
/// representing the dynamics and output.  The resulting system has only zero
/// or one vector input ports, zero or one vector of continuous or discrete
/// state (depending on the specified time_period), and only zero or one vector
/// output ports.

///
/// See SymbolicVectorSystemBuilder to make the construction a little nicer.
///
/// For example, to define the system: ẋ = -x + x³, y = x, we could write
/// @code
///   symbolic::Variable x("x");
///   auto system = SymbolicVectorBuilder().state(x).dynamics(-x + pow(x,3))
///                                        .output(x).Build();
/// @endcode
///
/// Note: This will not be as performant as writing your own LeafSystem.
/// It is meant primarily for rapid prototyping.
///
/// Instantiated templates for the following scalar types @p T are provided:
///
/// - double
/// - symbolic::Expression
template <typename T>
class SymbolicVectorSystem final : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SymbolicVectorSystem)

  /// Construct the SymbolicVectorSystem.
  ///
  /// @param time an (optional) Variable used to represent time in the dynamics.
  ///
  /// @param state an (optional) vector of Variables representing the state. The
  /// order in this vector will determine the order of the elements in the state
  /// vector.  Each element must be unique.
  ///
  /// @param input an (optional) vector of Variables representing the input. The
  /// order in this vector will determine the order of the elements in the
  /// vector-valued input port.  Each element must be unique.
  ///
  /// @param dynamics a vector of Expressions representing the dynamics of the
  /// system.  If @p time_period == 0, then this describes the continuous time
  /// derivatives.  If @p time_period > 0, then it defines the updates of the
  /// single discrete-valued state vector.  The size of this vector must match
  /// the number of state variables.
  ///
  /// @param output a vector of Expressions representing the output of the
  /// system.  If empty, then no output port will be allocated.
  ///
  /// @param time_period a scalar representing the period of a periodic update.
  /// time_period == 0.0 implies that the state variables will be declared as
  /// continuous state and the dynamics will be implemented as time
  /// derivatives.  time_period > 0.0 implies the state variables will be
  /// declared as discrete state and the dynamics will be implemented as a
  /// dicraete variable update.
  SymbolicVectorSystem(
      const optional<symbolic::Variable>& time,
      const Eigen::Ref<const VectorX<symbolic::Variable>>& state,
      const Eigen::Ref<const VectorX<symbolic::Variable>>& input,
      const Eigen::Ref<const VectorX<symbolic::Expression>>& dynamics,
      const Eigen::Ref<const VectorX<symbolic::Expression>>& output =
          Vector0<symbolic::Expression>{},
      double time_period = 0.0);

  // TODO(russt): Add support for parameters.

  /// Scalar-converting copy constructor.  See @ref system_scalar_conversion.
  template <typename U>
  explicit SymbolicVectorSystem(const SymbolicVectorSystem<U>& other)
      : SymbolicVectorSystem<T>(other.time_var_, other.state_vars_,
                                other.input_vars_, other.dynamics_,
                                other.output_, other.time_period_) {}

  ~SymbolicVectorSystem() override = default;

  /// Returns the sole input port.
  const InputPort<T>& get_input_port() const {
    DRAKE_DEMAND(this->num_input_ports() == 1);
    return LeafSystem<T>::get_input_port(0);
  }

  /// Returns the sole output port.
  const OutputPort<T>& get_output_port() const {
    DRAKE_DEMAND(this->num_output_ports() == 1);
    return LeafSystem<T>::get_output_port(0);
  }

 private:
  // Override feedthrough detection to avoid the need for `DoToSymbolic()`.
  optional<bool> DoHasDirectFeedthrough(int input_port,
                                        int output_port) const final;

  template <typename Container>
  void PopulateFromContext(const Context<T>& context, Container* penv) const;

  // Evaluate context to a vector.
  void EvaluateWithContext(const Context<T>& context,
                           const VectorX<symbolic::Expression>& expr,
                           const MatrixX<symbolic::Expression>& jacobian,
                           VectorBase<T>* out) const;

  void CalcOutput(const Context<T>& context,
                  BasicVector<T>* output_vector) const;

  void DoCalcTimeDerivatives(const Context<T>& context,
                             ContinuousState<T>* derivatives) const final;

  void DoCalcDiscreteVariableUpdates(
      const drake::systems::Context<T>& context,
      const std::vector<const drake::systems::DiscreteUpdateEvent<T>*>& events,
      drake::systems::DiscreteValues<T>* updates) const final;

  const optional<symbolic::Variable> time_var_{nullopt};
  const VectorX<symbolic::Variable> state_vars_{};
  const VectorX<symbolic::Variable> input_vars_{};
  const VectorX<symbolic::Expression> dynamics_{};
  const VectorX<symbolic::Expression> output_{};

  symbolic::Environment env_{};
  const double time_period_{0.0};

  // Storage for Jacobians (empty unless T == AutoDiffXd).
  MatrixX<symbolic::Expression> dynamics_jacobian_{};
  MatrixX<symbolic::Expression> output_jacobian_{};

  template <typename U>
  friend class SymbolicVectorSystem;
};

/// Builder design pattern to help with all of the optional arguments in the
/// constructor of SymbolicVectorSystem.
///
/// For example, to define the system: ẋ = -x + x³, y = x, we could write
/// @code
///   symbolic::Variable x("x");
///   auto system = SymbolicVectorBuilder().state(x)
///                                        .dynamics(-x + pow(x,3))
///                                        .output(x)
///                                        .Build();
/// @endcode
///
/// @see SymbolicVectorSystem
class SymbolicVectorSystemBuilder {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SymbolicVectorSystemBuilder)

  SymbolicVectorSystemBuilder() {}

  /// Sets the time variable.
  SymbolicVectorSystemBuilder time(const symbolic::Variable& t) {
    time_var_ = t;
    SymbolicVectorSystemBuilder result = *this;
    return result;
  }
  /// Sets the state variable (scalar version).
  SymbolicVectorSystemBuilder state(const symbolic::Variable& v) {
    state_vars_ = Vector1<symbolic::Variable>{v};
    SymbolicVectorSystemBuilder result = *this;
    return result;
  }
  /// Sets the state variables (vector version).
  SymbolicVectorSystemBuilder state(
      const Eigen::Ref<const VectorX<symbolic::Variable>>& vars) {
    state_vars_ = vars;
    SymbolicVectorSystemBuilder result = *this;
    return result;
  }
  /// Sets the input varaible (scalar version).
  SymbolicVectorSystemBuilder input(const symbolic::Variable& v) {
    input_vars_ = Vector1<symbolic::Variable>{v};
    SymbolicVectorSystemBuilder result = *this;
    return result;
  }
  /// Sets the input variables (vector version).
  SymbolicVectorSystemBuilder input(
      const Eigen::Ref<const VectorX<symbolic::Variable>>& vars) {
    input_vars_ = vars;
    SymbolicVectorSystemBuilder result = *this;
    return result;
  }
  /// Sets the dynamics method (scalar version).
  SymbolicVectorSystemBuilder dynamics(const symbolic::Expression& e) {
    dynamics_ = Vector1<symbolic::Expression>{e};
    SymbolicVectorSystemBuilder result = *this;
    return result;
  }
  /// Sets the dynamics method (vector version).
  SymbolicVectorSystemBuilder dynamics(
      const Eigen::Ref<const VectorX<symbolic::Expression>>& e) {
    dynamics_ = e;
    SymbolicVectorSystemBuilder result = *this;
    return result;
  }
  /// Sets the output method (scalar version).
  SymbolicVectorSystemBuilder output(const symbolic::Expression& e) {
    output_ = Vector1<symbolic::Expression>{e};
    SymbolicVectorSystemBuilder result = *this;
    return result;
  }
  /// Sets the output method (vector version).
  SymbolicVectorSystemBuilder output(
      const Eigen::Ref<const VectorX<symbolic::Expression>>& e) {
    output_ = e;
    SymbolicVectorSystemBuilder result = *this;
    return result;
  }
  /// Sets the time period (0 is continuous time).
  SymbolicVectorSystemBuilder time_period(double p) {
    time_period_ = p;
    SymbolicVectorSystemBuilder result = *this;
    return result;
  }

  /// Dispatches to the SymbolicVectorSystem constructor with our accumulated
  /// arguments.
  template <typename T = double>
  std::unique_ptr<SymbolicVectorSystem<T>> Build() {
    return std::make_unique<SymbolicVectorSystem<T>>(
        time_var_, state_vars_, input_vars_, dynamics_, output_, time_period_);
  }

  /// @name Accessor methods.
  /// @{
  const optional<symbolic::Variable>& time() const { return time_var_; }
  const VectorX<symbolic::Variable>& state() const { return state_vars_; }
  const VectorX<symbolic::Variable>& input() const { return input_vars_; }
  const VectorX<symbolic::Expression>& dynamics() const { return dynamics_; }
  const VectorX<symbolic::Expression>& output() const { return output_; }
  double time_period() const { return time_period_; }
  /// @}

 private:
  optional<symbolic::Variable> time_var_{nullopt};
  VectorX<symbolic::Variable> state_vars_{};
  VectorX<symbolic::Variable> input_vars_{};
  VectorX<symbolic::Expression> dynamics_{};
  VectorX<symbolic::Expression> output_{};
  double time_period_{0.0};
};

#ifndef DRAKE_DOXYGEN_CXX
// Forward-declare specializations, prior to DRAKE_DECLARE... below.
template <>
void SymbolicVectorSystem<double>::EvaluateWithContext(
    const Context<double>& context, const VectorX<symbolic::Expression>& expr,
    const MatrixX<symbolic::Expression>& jacobian,
    VectorBase<double>* out) const;

template <>
void SymbolicVectorSystem<AutoDiffXd>::EvaluateWithContext(
    const Context<AutoDiffXd>& context,
    const VectorX<symbolic::Expression>& expr,
    const MatrixX<symbolic::Expression>& jacobian,
    VectorBase<AutoDiffXd>* out) const;

template <>
void SymbolicVectorSystem<symbolic::Expression>::EvaluateWithContext(
    const Context<symbolic::Expression>& context,
    const VectorX<symbolic::Expression>& expr,
    const MatrixX<symbolic::Expression>& jacobian,
    VectorBase<symbolic::Expression>* out) const;
#endif

}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::SymbolicVectorSystem);

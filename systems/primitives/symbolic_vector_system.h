#pragma once

#include <memory>
#include <optional>
#include <stdexcept>
#include <unordered_map>
#include <vector>

#include <fmt/format.h>

#include "drake/common/eigen_types.h"
#include "drake/common/symbolic.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {

/// A LeafSystem that is defined by vectors of symbolic::Expression
/// representing the dynamics and output.  The resulting system has only zero
/// or one vector input ports, zero or one vector of continuous or discrete
/// state (depending on the specified time_period), zero or one vector of
/// numeric parameters, and only zero or one vector output ports.
///
/// See SymbolicVectorSystemBuilder to make the construction a little nicer.
///
/// For example, to define the system: ẋ = -x + x³, y = x, we could write
/// @code
///   symbolic::Variable x("x");
///   auto system = SymbolicVectorSystemBuilder().state(x)
///                                              .dynamics(-x + pow(x,3))
///                                              .output(x)
///                                              .Build();
/// @endcode
///
/// Note: This will not be as performant as writing your own LeafSystem.
/// It is meant primarily for rapid prototyping.
///
/// @tparam_default_scalar
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
  /// @param parameter an (optional) vector of Variables representing the
  /// numeric parameter. The order in this vector will determine the order of
  /// the elements in the vector-valued parameter.  Each element must be unique.
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
      const std::optional<symbolic::Variable>& time,
      const Eigen::Ref<const VectorX<symbolic::Variable>>& state,
      const Eigen::Ref<const VectorX<symbolic::Variable>>& input,
      const Eigen::Ref<const VectorX<symbolic::Variable>>& parameter,
      const Eigen::Ref<const VectorX<symbolic::Expression>>& dynamics,
      const Eigen::Ref<const VectorX<symbolic::Expression>>& output =
          Vector0<symbolic::Expression>{},
      double time_period = 0.0);

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
      const std::optional<symbolic::Variable>& time,
      const Eigen::Ref<const VectorX<symbolic::Variable>>& state,
      const Eigen::Ref<const VectorX<symbolic::Variable>>& input,
      const Eigen::Ref<const VectorX<symbolic::Expression>>& dynamics,
      const Eigen::Ref<const VectorX<symbolic::Expression>>& output =
          Vector0<symbolic::Expression>{},
      double time_period = 0.0)
      : SymbolicVectorSystem<T>(time, state, input,
                                Vector0<symbolic::Variable>{}, dynamics, output,
                                time_period) {}

  /// Scalar-converting copy constructor.  See @ref system_scalar_conversion.
  template <typename U>
  explicit SymbolicVectorSystem(const SymbolicVectorSystem<U>& other)
      : SymbolicVectorSystem<T>(other.time_var_, other.state_vars_,
                                other.input_vars_, other.parameter_vars_,
                                other.dynamics_, other.output_,
                                other.time_period_) {}

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

  /// @name Accessor methods.
  /// @{
  const std::optional<symbolic::Variable>& time_var() const {
    return time_var_;
  }
  const VectorX<symbolic::Variable>& state_vars() const { return state_vars_; }
  const VectorX<symbolic::Variable>& input_vars() const { return input_vars_; }
  const VectorX<symbolic::Variable>& parameter_vars() const {
    return parameter_vars_;
  }
  const VectorX<symbolic::Expression>& dynamics() const { return dynamics_; }

  /// Returns the dynamics for the variable @p var. That is, it returns the
  /// scalar expression corresponding to either `\dot{var}` (continuous case) or
  /// `var[n+1]` (discrete case).
  ///
  /// @throw std::out_of_range if this system has no corresponding dynamics for
  /// the variable @p var.
  const symbolic::Expression& dynamics_for_variable(
      const symbolic::Variable& var) const {
    auto it = state_var_to_index_.find(var.get_id());
    if (it != state_var_to_index_.end()) {
      return dynamics_[it->second];
    } else {
      throw std::out_of_range{
          fmt::format("This SymbolicVectorSystem does not have a dynamics for "
                      "the given variable {}",
                      var)};
    }
  }

  const VectorX<symbolic::Expression>& output() const { return output_; }
  /// @}

 private:
  // Reports if the given expression contains an input variable.
  bool DependsOnInputs(const VectorX<symbolic::Expression>& expr) const;

  template <typename Container>
  void PopulateFromContext(const Context<T>& context, bool needs_inputs,
                           Container* penv) const;

  // Evaluate context to a vector.
  void EvaluateWithContext(const Context<T>& context,
                           const VectorX<symbolic::Expression>& expr,
                           const MatrixX<symbolic::Expression>& jacobian,
                           bool needs_inputs, VectorBase<T>* out) const;

  void CalcOutput(const Context<T>& context,
                  BasicVector<T>* output_vector) const;

  void DoCalcTimeDerivatives(const Context<T>& context,
                             ContinuousState<T>* derivatives) const final;

  void DoCalcDiscreteVariableUpdates(
      const drake::systems::Context<T>& context,
      const std::vector<const drake::systems::DiscreteUpdateEvent<T>*>& events,
      drake::systems::DiscreteValues<T>* updates) const final;

  const std::optional<symbolic::Variable> time_var_{std::nullopt};
  const VectorX<symbolic::Variable> state_vars_{};
  const VectorX<symbolic::Variable> input_vars_{};
  const VectorX<symbolic::Variable> parameter_vars_{};
  const VectorX<symbolic::Expression> dynamics_{};
  const VectorX<symbolic::Expression> output_{};
  const bool dynamics_needs_inputs_;
  const bool output_needs_inputs_;

  symbolic::Environment env_{};
  const double time_period_{0.0};

  std::unordered_map<symbolic::Variable::Id, int> state_var_to_index_;

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
///   auto system = SymbolicVectorSystemBuilder().state(x)
///                                              .dynamics(-x + pow(x,3))
///                                              .output(x)
///                                              .Build();
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
    return state(Vector1<symbolic::Variable>{v});
  }
  /// Sets the state variables (Eigen::Vector version).
  SymbolicVectorSystemBuilder state(
      const Eigen::Ref<const VectorX<symbolic::Variable>>& vars) {
    state_vars_ = vars;
    for (int i = 0; i < vars.size(); ++i) {
      state_var_to_index_.emplace(state_vars_[i].get_id(), i);
    }
    SymbolicVectorSystemBuilder result = *this;
    return result;
  }
  /// Sets the state variables (std::vector version).
  SymbolicVectorSystemBuilder state(
      const std::vector<symbolic::Variable>& vars) {
    return state(Eigen::Map<const VectorX<symbolic::Variable>>(vars.data(),
                                                               vars.size()));
  }
  /// Sets the input variable (scalar version).
  SymbolicVectorSystemBuilder input(const symbolic::Variable& v) {
    input_vars_ = Vector1<symbolic::Variable>{v};
    SymbolicVectorSystemBuilder result = *this;
    return result;
  }
  /// Sets the input variables (Eigen::Vector version).
  SymbolicVectorSystemBuilder input(
      const Eigen::Ref<const VectorX<symbolic::Variable>>& vars) {
    input_vars_ = vars;
    SymbolicVectorSystemBuilder result = *this;
    return result;
  }
  /// Sets the input variables (std::vector version).
  SymbolicVectorSystemBuilder input(
      const std::vector<symbolic::Variable>& vars) {
    return input(Eigen::Map<const VectorX<symbolic::Variable>>(vars.data(),
                                                               vars.size()));
  }
  /// Sets the parameter variable (scalar version).
  SymbolicVectorSystemBuilder parameter(const symbolic::Variable& v) {
    parameter_vars_ = Vector1<symbolic::Variable>{v};
    SymbolicVectorSystemBuilder result = *this;
    return result;
  }
  /// Sets the parameter variables (Eigen::Vector version).
  SymbolicVectorSystemBuilder parameter(
      const Eigen::Ref<const VectorX<symbolic::Variable>>& vars) {
    parameter_vars_ = vars;
    SymbolicVectorSystemBuilder result = *this;
    return result;
  }
  /// Sets the parameter variables (std::vector version).
  SymbolicVectorSystemBuilder parameter(
      const std::vector<symbolic::Variable>& vars) {
    return parameter(Eigen::Map<const VectorX<symbolic::Variable>>(
        vars.data(), vars.size()));
  }
  /// Sets the dynamics method (scalar version).
  SymbolicVectorSystemBuilder dynamics(const symbolic::Expression& e) {
    dynamics_ = Vector1<symbolic::Expression>{e};
    SymbolicVectorSystemBuilder result = *this;
    return result;
  }
  /// Sets the dynamics method (Eigen::Vector version).
  SymbolicVectorSystemBuilder dynamics(
      const Eigen::Ref<const VectorX<symbolic::Expression>>& e) {
    dynamics_ = e;
    SymbolicVectorSystemBuilder result = *this;
    return result;
  }
  /// Sets the dynamics variables (std::vector version).
  SymbolicVectorSystemBuilder dynamics(
      const std::vector<symbolic::Expression>& e) {
    return dynamics(
        Eigen::Map<const VectorX<symbolic::Expression>>(e.data(), e.size()));
  }
  /// Sets the output method (scalar version).
  SymbolicVectorSystemBuilder output(const symbolic::Expression& e) {
    output_ = Vector1<symbolic::Expression>{e};
    SymbolicVectorSystemBuilder result = *this;
    return result;
  }
  /// Sets the output method (Eigen::Vector version).
  SymbolicVectorSystemBuilder output(
      const Eigen::Ref<const VectorX<symbolic::Expression>>& e) {
    output_ = e;
    SymbolicVectorSystemBuilder result = *this;
    return result;
  }
  /// Sets the output variables (std::vector version).
  SymbolicVectorSystemBuilder output(
      const std::vector<symbolic::Expression>& e) {
    return output(
        Eigen::Map<const VectorX<symbolic::Expression>>(e.data(), e.size()));
  }

  /// Linearizes the system dynamics around `(x0, u0)` using the first-order
  /// Taylor Series expansion.
  ///
  /// @pre The length of @p x0 should be the length of `state()`.
  /// @pre The length of @p u0 should be the length of `input()`.
  /// @pre @p x0 and @p u0 should not include a state variable or an input
  /// variable.
  ///
  /// @note If @p x0 or @p u0 includes a variable new to this system builder, it
  /// will be added to this system builder as a parameter.
  SymbolicVectorSystemBuilder LinearizeDynamics(
      const Eigen::Ref<const VectorX<symbolic::Expression>>& x0,
      const Eigen::Ref<const VectorX<symbolic::Expression>>& u0);

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
        time_var_, state_vars_, input_vars_, parameter_vars_, dynamics_,
        output_, time_period_);
  }

  /// @name Accessor methods.
  /// @{
  /// Returns the time variable if exists.
  const std::optional<symbolic::Variable>& time() const { return time_var_; }
  /// Returns the state variables.
  const VectorX<symbolic::Variable>& state() const { return state_vars_; }
  /// Returns the input variables.
  const VectorX<symbolic::Variable>& input() const { return input_vars_; }
  /// Returns the parameter variables.
  const VectorX<symbolic::Variable>& parameter() const {
    return parameter_vars_;
  }
  /// Returns the dynamics.
  const VectorX<symbolic::Expression>& dynamics() const { return dynamics_; }
  /// Returns the dynamics for the variable @p var. That is, it returns the
  /// scalar expression corresponding to either `\dot{var}` (continuous case) or
  /// `var[n+1]` (discrete case).
  ///
  /// @throw std::out_of_range if this builder has no corresponding dynamics for
  /// the variable @p var.
  const symbolic::Expression& dynamics_for_variable(
      const symbolic::Variable& var) const {
    auto it = state_var_to_index_.find(var.get_id());
    if (it != state_var_to_index_.end()) {
      return dynamics_[it->second];
    } else {
      throw std::out_of_range{fmt::format(
          "This SymbolicVectorSystemBuilder does not have a dynamics for "
          "the given variable {}",
          var)};
    }
  }
  /// Returns the output.
  const VectorX<symbolic::Expression>& output() const { return output_; }
  /// Returns the time period.
  double time_period() const { return time_period_; }
  /// @}
 private:
  std::optional<symbolic::Variable> time_var_{std::nullopt};
  VectorX<symbolic::Variable> state_vars_{};
  VectorX<symbolic::Variable> input_vars_{};
  VectorX<symbolic::Variable> parameter_vars_{};
  VectorX<symbolic::Expression> dynamics_{};
  VectorX<symbolic::Expression> output_{};
  double time_period_{0.0};
  std::unordered_map<symbolic::Variable::Id, int> state_var_to_index_;
};

#ifndef DRAKE_DOXYGEN_CXX
// Forward-declare specializations, prior to DRAKE_DECLARE... below.
template <>
void SymbolicVectorSystem<double>::EvaluateWithContext(
    const Context<double>& context, const VectorX<symbolic::Expression>& expr,
    const MatrixX<symbolic::Expression>& jacobian, bool needs_inputs,
    VectorBase<double>* out) const;

template <>
void SymbolicVectorSystem<AutoDiffXd>::EvaluateWithContext(
    const Context<AutoDiffXd>& context,
    const VectorX<symbolic::Expression>& expr,
    const MatrixX<symbolic::Expression>& jacobian, bool needs_inputs,
    VectorBase<AutoDiffXd>* out) const;

template <>
void SymbolicVectorSystem<symbolic::Expression>::EvaluateWithContext(
    const Context<symbolic::Expression>& context,
    const VectorX<symbolic::Expression>& expr,
    const MatrixX<symbolic::Expression>& jacobian, bool needs_inputs,
    VectorBase<symbolic::Expression>* out) const;
#endif

}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::SymbolicVectorSystem);

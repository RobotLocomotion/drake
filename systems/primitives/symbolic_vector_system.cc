#include "drake/systems/primitives/symbolic_vector_system.h"

#include <algorithm>
#include <optional>

#include "drake/math/autodiff_gradient.h"

namespace drake {
namespace systems {

using Eigen::Ref;
using symbolic::Environment;
using symbolic::Expression;
using symbolic::Jacobian;
using symbolic::Substitution;
using symbolic::Variable;
using symbolic::Variables;

namespace {

// Returns the first-order Taylor-series of @p f at @p a. The substitution a
// provides the linearization point.
Expression FirstOrderTaylorExpand(const Expression& f, const Substitution& a) {
  // f(a) + ∑ᵢ (xᵢ - aᵢ)∂f/∂xᵢ(a)
  Expression ret = f.Substitute(a);  // f(a)
  for (const auto& p : a) {
    const Variable& x_i = p.first;
    const Expression& a_i = p.second;
    // Add (xᵢ - aᵢ)∂f/∂xᵢ(a)
    ret += (x_i - a_i) * f.Differentiate(x_i).Substitute(a);
  }
  return ret.Expand();
}

// Checks if @p v is in @p variables.
bool Includes(const Ref<const VectorX<Variable>>& variables,
                     const Variable& v) {
  for (int i = 0; i < variables.size(); ++i) {
    if (variables[i].equal_to(v)) {
      return true;
    }
  }
  return false;
}

}  // namespace

SymbolicVectorSystemBuilder SymbolicVectorSystemBuilder::LinearizeDynamics(
    const Ref<const VectorX<Expression>>& x0,
    const Ref<const VectorX<Expression>>& u0) {
  DRAKE_DEMAND(state_vars_.size() == x0.size());
  DRAKE_DEMAND(input_vars_.size() == u0.size());

  // Build a substitution and collect all variables in x0 and u0.
  Substitution subst;
  Variables variables;
  for (int i = 0; i < state_vars_.size(); ++i) {
    subst.emplace(state_vars_[i], x0[i]);
    variables += x0[i].GetVariables();
  }
  for (int i = 0; i < input_vars_.size(); ++i) {
    subst.emplace(input_vars_[i], u0[i]);
    variables += u0[i].GetVariables();
  }

  // Linearize dynamics.
  for (int i = 0; i < dynamics_.size(); ++i) {
    dynamics_[i] = FirstOrderTaylorExpand(dynamics_[i], subst);
  }

  // Update parameters.
  for (const auto& var : variables) {
    DRAKE_DEMAND(!Includes(state_vars_, var));
    DRAKE_DEMAND(!Includes(input_vars_, var));
    if (!Includes(parameter_vars_, var)) {
      const auto prev_size = parameter_vars_.size();
      parameter_vars_.conservativeResize(prev_size + 1);
      parameter_vars_[prev_size] = var;
    }
  }

  SymbolicVectorSystemBuilder result = *this;
  return result;
}

template <typename T>
SymbolicVectorSystem<T>::SymbolicVectorSystem(
    const std::optional<Variable>& time,
    const Ref<const VectorX<Variable>>& state,
    const Ref<const VectorX<Variable>>& input,
    const Ref<const VectorX<Variable>>& parameter,
    const Ref<const VectorX<Expression>>& dynamics,
    const Ref<const VectorX<Expression>>& output, double time_period)
    : LeafSystem<T>(SystemTypeTag<SymbolicVectorSystem>{}),
      time_var_(time),
      state_vars_(state),
      input_vars_(input),
      parameter_vars_(parameter),
      dynamics_(dynamics),
      output_(output),
      dynamics_needs_inputs_(DependsOnInputs(dynamics)),
      output_needs_inputs_(DependsOnInputs(output_)),
      time_period_(time_period) {  // Must have dynamics and/or output.
  DRAKE_DEMAND(dynamics_.rows() > 0 || output_.rows() > 0);
  DRAKE_DEMAND(time_period_ >= 0.0);

  // Construct set of all variables to check validity of dynamics and output.
  VectorX<Variable> vars_vec(state_vars_.size() + input_vars_.size() +
                             parameter_vars_.size() + (time_var_ ? 1 : 0));
  if (time_var_) {
    vars_vec << state_vars_, input_vars_, parameter_vars_, *time_var_;
  } else {
    vars_vec << state_vars_, input_vars_, parameter_vars_;
  }
  Variables all_vars(vars_vec);
  // Check that state and input variables were all unique.
  DRAKE_DEMAND(static_cast<int>(all_vars.size()) == vars_vec.size());

  if (input_vars_.size() > 0) {
    this->DeclareInputPort(kUseDefaultName, kVectorValued, input_vars_.size());
  }
  for (int i = 0; i < state_vars_.size(); i++) {
    state_var_to_index_.emplace(state_vars_[i].get_id(), i);
  }
  if (state_vars_.size() > 0) {
    for (int i = 0; i < dynamics_.size(); i++) {
      DRAKE_ASSERT(dynamics_[i].GetVariables().IsSubsetOf(all_vars));
    }
    if (time_period_ == 0.0) {
      this->DeclareContinuousState(state_vars_.size());
    } else {
      this->DeclareDiscreteState(state_vars_.size());
      this->DeclarePeriodicDiscreteUpdate(time_period_, 0.0);
    }
  }
  if (parameter_vars_.size() > 0) {
    this->DeclareNumericParameter(BasicVector<T>(parameter_vars_.size()));
  }
  if (output_.size() > 0) {
    for (int i = 0; i < output_.size(); i++) {
      DRAKE_ASSERT(output_[i].GetVariables().IsSubsetOf(all_vars));
    }
    this->DeclareVectorOutputPort(kUseDefaultName, output_.size(),
                                  &SymbolicVectorSystem<T>::CalcOutput);
  }

  // Initialize Jacobian matrices iff T == AutoDiffXd.
  if (std::is_same_v<T, AutoDiffXd>) {
    if (dynamics_.size() > 0) {
      dynamics_jacobian_ = Jacobian(dynamics_, vars_vec);
    }
    if (output_.size() > 0) {
      output_jacobian_ = Jacobian(output_, vars_vec);
    }
  }

  // Allocate a symbolic::Environment once to be cloned/reused below.
  for (const auto& v : all_vars) {
    env_.insert(v, 0.0);
  }
}

template <typename T>
bool SymbolicVectorSystem<T>::DependsOnInputs(
    const VectorX<Expression>& expr) const {
  Variables needed_variables;
  for (int j = 0; j < expr.size(); ++j) {
    needed_variables.insert(expr(j).GetVariables());
  }

  for (int i = 0; i < input_vars_.size(); i++) {
    if (needed_variables.include(input_vars_[i])) {
      return true;
    }
  }
  return false;
}

template <typename T>
template <typename Container>
void SymbolicVectorSystem<T>::PopulateFromContext(const Context<T>& context,
                                                  bool needs_inputs,
                                                  Container* penv) const {
  Container& env = *penv;
  if (time_var_) {
    env[*time_var_] = context.get_time();
  }
  if (state_vars_.size() > 0) {
    const VectorBase<T>& state = (time_period_ > 0.0)
                                     ? context.get_discrete_state_vector()
                                     : context.get_continuous_state_vector();
    for (int i = 0; i < state_vars_.size(); i++) {
      env[state_vars_[i]] = state[i];
    }
  }
  // Note: Invocations only require pre-analysis on the *input* dependency (as
  // opposed to state, time, etc) because all other values come directly from
  // the Context (and not from input ports whose *unnecessary* evaluation can
  // lead to spurious algebraic loops).
  if (input_vars_.size() > 0 && needs_inputs) {
    const auto& input = this->get_input_port().Eval(context);
    for (int i = 0; i < input_vars_.size(); i++) {
      env[input_vars_[i]] = input[i];
    }
  }
  if (parameter_vars_.size() > 0) {
    const auto& parameter = context.get_numeric_parameter(0);
    for (int i = 0; i < parameter_vars_.size(); i++) {
      env[parameter_vars_[i]] = parameter[i];
    }
  }
}

// TODO(eric.cousineau): Consider decoupling output from `VectorBase` and use
// `EigenPtr` or something.

template <>
void SymbolicVectorSystem<double>::EvaluateWithContext(
    const Context<double>& context, const VectorX<Expression>& expr,
    const MatrixX<symbolic::Expression>& jacobian,
    bool needs_inputs, VectorBase<double>* out) const {
  unused(jacobian);
  Environment env = env_;
  PopulateFromContext(context, needs_inputs, &env);
  for (int i = 0; i < out->size(); i++) {
    out->SetAtIndex(i, expr[i].Evaluate(env));
  }
}

template <>
void SymbolicVectorSystem<AutoDiffXd>::EvaluateWithContext(
    const Context<AutoDiffXd>& context, const VectorX<Expression>& expr,
    const MatrixX<symbolic::Expression>& jacobian,
    bool needs_inputs, VectorBase<AutoDiffXd>* pout) const {
  VectorBase<AutoDiffXd>& out = *pout;

  const BasicVector<AutoDiffXd> empty(0);
  const AutoDiffXd& time = context.get_time();
  const VectorBase<AutoDiffXd>& state =
      (state_vars_.size() > 0)
          ? ((time_period_ > 0.0) ? context.get_discrete_state_vector()
                                  : context.get_continuous_state_vector())
          : empty;
  // It is very important we don't evaluate the inputs if the expression doesn't
  // actually depend on it (as declared by the needs_inputs parameter). This
  // avoids introducing spurious algebraic loops. In this case, `needs_inputs`
  // is sufficient to know that there are input variables; we don't need to
  // test the size of input_vars_.
  const BasicVector<AutoDiffXd>& input =
      needs_inputs
          ? get_input_port().Eval<BasicVector<AutoDiffXd>>(context)
          : empty;

  const BasicVector<AutoDiffXd>& parameter =
      (parameter_vars_.size() > 0) ? context.get_numeric_parameter(0) : empty;

  // Figure out the length of the derivative vector.  The derivatives must all
  // have size zero or the same non-zero size.
  int num_gradients = time.derivatives().size();
  auto set_num_gradients = [&num_gradients](
                               const VectorBase<AutoDiffXd>& vars) {
    for (int i = 0; i < vars.size(); i++) {
      if (vars[i].derivatives().size()) {
        if (num_gradients == 0) {
          num_gradients = static_cast<int>(vars[i].derivatives().size());
        } else {
          DRAKE_DEMAND(static_cast<int>(vars[i].derivatives().size()) ==
                       num_gradients);
        }
      }
    }
  };
  set_num_gradients(state);
  if (needs_inputs) {
    set_num_gradients(input);
  }
  set_num_gradients(parameter);

  Eigen::MatrixXd dvars = Eigen::MatrixXd::Zero(jacobian.cols(), num_gradients);
  Environment env = env_;
  if (time_var_) {
    env[*time_var_] = time.value();
    if (time.derivatives().size()) {
      dvars.bottomRows<1>() = time.derivatives();
    }
  }
  size_t dvars_row_idx = 0;
  for (int i = 0; i < state_vars_.size(); i++, dvars_row_idx++) {
    env[state_vars_[i]] = state[i].value();
    if (state[i].derivatives().size()) {
      dvars.row(dvars_row_idx) = state[i].derivatives();
    }
  }
  if (needs_inputs) {
    for (int i = 0; i < input_vars_.size(); i++, dvars_row_idx++) {
      env[input_vars_[i]] = input[i].value();
      if (input[i].derivatives().size()) {
        dvars.row(dvars_row_idx) = input[i].derivatives();
      }
    }
  } else {
    dvars_row_idx += input_vars_.size();
  }
  for (int i = 0; i < parameter_vars_.size(); i++, dvars_row_idx++) {
    env[parameter_vars_[i]] = parameter[i].value();
    if (parameter[i].derivatives().size()) {
      dvars.row(dvars_row_idx) = parameter[i].derivatives();
    }
  }

  // Now actually compute the output values and derivatives.
  Eigen::RowVectorXd dout_dvars(jacobian.cols());
  for (int i = 0; i < out.size(); i++) {
    out[i].value() = expr[i].Evaluate(env);

    for (int j = 0; j < jacobian.cols(); j++) {
      dout_dvars(j) = jacobian(i, j).Evaluate(env);
    }
    out[i].derivatives() = dout_dvars * dvars;
  }
}

template <>
void SymbolicVectorSystem<Expression>::EvaluateWithContext(
    const Context<Expression>& context, const VectorX<Expression>& expr,
    const MatrixX<symbolic::Expression>& jacobian,
    bool needs_inputs, VectorBase<Expression>* out) const {
  unused(jacobian);
  Substitution s;
  PopulateFromContext(context, needs_inputs, &s);
  for (int i = 0; i < out->size(); i++) {
    out->SetAtIndex(i, expr[i].Substitute(s));
  }
}

template <typename T>
void SymbolicVectorSystem<T>::CalcOutput(const Context<T>& context,
                                         BasicVector<T>* output_vector) const {
  DRAKE_DEMAND(output_.size() > 0);
  EvaluateWithContext(context, output_, output_jacobian_, output_needs_inputs_,
                      output_vector);
}

template <typename T>
void SymbolicVectorSystem<T>::DoCalcTimeDerivatives(
    const Context<T>& context, ContinuousState<T>* derivatives) const {
  DRAKE_DEMAND(time_period_ == 0.0);
  DRAKE_DEMAND(dynamics_.size() > 0);
  EvaluateWithContext(context, dynamics_, dynamics_jacobian_,
                      dynamics_needs_inputs_,
                      &derivatives->get_mutable_vector());
}

template <typename T>
void SymbolicVectorSystem<T>::DoCalcDiscreteVariableUpdates(
    const drake::systems::Context<T>& context,
    const std::vector<const drake::systems::DiscreteUpdateEvent<T>*>& events,
    drake::systems::DiscreteValues<T>* updates) const {
  unused(events);
  DRAKE_DEMAND(time_period_ > 0.0);
  DRAKE_DEMAND(dynamics_.size() > 0);
  EvaluateWithContext(context, dynamics_, dynamics_jacobian_,
                      dynamics_needs_inputs_, &updates->get_mutable_vector());
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::SymbolicVectorSystem);

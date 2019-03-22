#include "drake/systems/primitives/symbolic_vector_system.h"

#include "drake/common/drake_optional.h"

namespace drake {
namespace systems {

using Eigen::Ref;
using symbolic::Environment;
using symbolic::Expression;
using symbolic::Variable;
using symbolic::Variables;

template <typename T>
SymbolicVectorSystem<T>::SymbolicVectorSystem(
    const optional<Variable>& time_var,
    const Ref<const VectorX<Variable>>& state_vars,
    const Ref<const VectorX<Variable>>& input_vars,
    const Ref<const VectorX<Expression>>& dynamics,
    const Ref<const VectorX<Expression>>& output, double time_period)
    : LeafSystem<T>(SystemTypeTag<systems::SymbolicVectorSystem>{}),
      time_var_(time_var),
      state_vars_(state_vars),
      input_vars_(input_vars),
      dynamics_(dynamics),
      output_(output),
      time_period_(time_period) {  // Must have dynamics and/or output.
  DRAKE_DEMAND(dynamics_.rows() > 0 || output_.rows() > 0);
  DRAKE_DEMAND(time_period_ >= 0.0);

  // Construct set of all variables to check validity of dynamics and output.
  VectorX<Variable> vars_vec(state_vars_.size() + input_vars_.size());
  vars_vec << state_vars_, input_vars_;
  Variables all_vars(vars_vec);
  // Check that state_vars and input_vars were all unique.
  DRAKE_DEMAND(static_cast<int>(all_vars.size()) == vars_vec.size());
  if (time_var_) {
    all_vars.insert(*time_var_);
  }

  if (input_vars_.rows() > 0) {
    this->DeclareInputPort(kVectorValued, input_vars_.size());
  }
  if (state_vars_.rows() > 0) {
    for (int i = 0; i < dynamics_.rows(); i++) {
      DRAKE_ASSERT(dynamics_[i].GetVariables().IsSubsetOf(all_vars));
    }
    if (time_period_ == 0.0) {
      this->DeclareContinuousState(state_vars_.rows());
    } else {
      this->DeclareDiscreteState(state_vars_.rows());
      this->DeclarePeriodicDiscreteUpdate(time_period_, 0.0);
    }
  }
  if (output_.rows() > 0) {
    for (int i = 0; i < output_.rows(); i++) {
      DRAKE_ASSERT(output_[i].GetVariables().IsSubsetOf(all_vars));
    }
    this->DeclareVectorOutputPort(BasicVector<T>(output_.rows()),
                                  &SymbolicVectorSystem<T>::CalcOutput);
  }

  // Allocate a symbolic::Environment once to be cloned/reused below.
  for (const auto& v : all_vars) {
    env_.insert(v, 0.0);
  }
}

template <typename T>
optional<bool> SymbolicVectorSystem<T>::DoHasDirectFeedthrough(
    int input_port, int output_port) const {
  DRAKE_DEMAND(input_port == 0);
  DRAKE_DEMAND(output_port == 0);
  for (int i = 0; i <= output_.size(); i++) {
    for (int j = 0; j <= input_vars_.size(); j++) {
      if (output_[i].GetVariables().include(input_vars_[j])) return true;
    }
  }
  return false;
}

template <typename T>
template <typename Container>
void SymbolicVectorSystem<T>::PopulateFromContext(
    const Context<T>& context, Container* penv) const {
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
  if (input_vars_.size() > 0) {
    const auto& input = get_input_port().Eval(context);
    for (int i = 0; i < input_vars_.size(); i++) {
      env[input_vars_[i]] = input[i];
    }
  }
}

// TODO(eric.cousineau): Consider decoupling output from `VectorBase` and use
// `EigenPtr` or something.

template <>
void SymbolicVectorSystem<double>::EvaluateWithContext(
    const Context<double>& context, const VectorX<Expression>& expr,
    VectorBase<double>* out) const {
  Environment env = env_;
  PopulateFromContext(context, &env);
  for (int i = 0; i < out->size(); i++) {
    out->SetAtIndex(i, expr[i].Evaluate(env));
  }
}

template <>
void SymbolicVectorSystem<Expression>::EvaluateWithContext(
    const Context<Expression>& context, const VectorX<Expression>& expr,
    VectorBase<Expression>* out) const {
  symbolic::Substitution s;
  PopulateFromContext(context, &s);
  for (int i = 0; i < out->size(); i++) {
    out->SetAtIndex(i, expr[i].Substitute(s));
  }
}

template <typename T>
void SymbolicVectorSystem<T>::CalcOutput(
    const Context<T>& context, BasicVector<T>* output_vector) const {
  DRAKE_DEMAND(output_.size() > 0);
  EvaluateWithContext(context, output_, output_vector);
}

template <typename T>
void SymbolicVectorSystem<T>::DoCalcTimeDerivatives(
    const Context<T>& context,
    ContinuousState<T>* derivatives) const {
  DRAKE_DEMAND(time_period_ == 0.0);
  DRAKE_DEMAND(dynamics_.size() > 0);
  EvaluateWithContext(context, dynamics_, &derivatives->get_mutable_vector());
}

template <typename T>
void SymbolicVectorSystem<T>::DoCalcDiscreteVariableUpdates(
    const drake::systems::Context<T>& context,
    const std::vector<const drake::systems::DiscreteUpdateEvent<T>*>&
        events,
    drake::systems::DiscreteValues<T>* updates) const {
  unused(events);
  DRAKE_DEMAND(time_period_ > 0.0);
  DRAKE_DEMAND(dynamics_.size() > 0);
  EvaluateWithContext(context, dynamics_, &updates->get_mutable_vector());
}

}  // namespace systems
}  // namespace drake

template class ::drake::systems::SymbolicVectorSystem<double>;
template class ::drake::systems::SymbolicVectorSystem<
    drake::symbolic ::Expression>;

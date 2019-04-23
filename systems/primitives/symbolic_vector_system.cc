#include "drake/systems/primitives/symbolic_vector_system.h"

#include <algorithm>

#include "drake/common/drake_optional.h"
#include "drake/math/autodiff_gradient.h"

namespace drake {
namespace systems {

using Eigen::Ref;
using symbolic::Environment;
using symbolic::Expression;
using symbolic::Jacobian;
using symbolic::Variable;
using symbolic::Variables;

template <typename T>
SymbolicVectorSystem<T>::SymbolicVectorSystem(
    const optional<Variable>& time, const Ref<const VectorX<Variable>>& state,
    const Ref<const VectorX<Variable>>& input,
    const Ref<const VectorX<Expression>>& dynamics,
    const Ref<const VectorX<Expression>>& output, double time_period)
    : LeafSystem<T>(SystemTypeTag<systems::SymbolicVectorSystem>{}),
      time_var_(time),
      state_vars_(state),
      input_vars_(input),
      dynamics_(dynamics),
      output_(output),
      time_period_(time_period) {  // Must have dynamics and/or output.
  DRAKE_DEMAND(dynamics_.rows() > 0 || output_.rows() > 0);
  DRAKE_DEMAND(time_period_ >= 0.0);

  // Construct set of all variables to check validity of dynamics and output.
  VectorX<Variable> vars_vec(state_vars_.size() + input_vars_.size() +
                             (time_var_ ? 1 : 0));
  if (time_var_) {
    vars_vec << state_vars_, input_vars_, *time_var_;
  } else {
    vars_vec << state_vars_, input_vars_;
  }
  Variables all_vars(vars_vec);
  // Check that state and input variables were all unique.
  DRAKE_DEMAND(static_cast<int>(all_vars.size()) == vars_vec.size());

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

  // Initialize Jacobian matrices iff T == AutoDiffXd.
  if (std::is_same<T, AutoDiffXd>::value) {
    if (dynamics_.rows() > 0) {
      dynamics_jacobian_ = Jacobian(dynamics_, vars_vec);
    }
    if (output_.rows() > 0) {
      output_jacobian_ = Jacobian(output_, vars_vec);
    }
  }

  // Allocate a symbolic::Environment once to be cloned/reused below.
  for (const auto& v : all_vars) {
    env_.insert(v, 0.0);
  }
}

template <typename T>
template <typename Container>
void SymbolicVectorSystem<T>::PopulateFromContext(const Context<T>& context,
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
    const MatrixX<symbolic::Expression>& jacobian,
    VectorBase<double>* out) const {
  unused(jacobian);
  Environment env = env_;
  PopulateFromContext(context, &env);
  for (int i = 0; i < out->size(); i++) {
    out->SetAtIndex(i, expr[i].Evaluate(env));
  }
}

template <>
void SymbolicVectorSystem<AutoDiffXd>::EvaluateWithContext(
    const Context<AutoDiffXd>& context, const VectorX<Expression>& expr,
    const MatrixX<symbolic::Expression>& jacobian,
    VectorBase<AutoDiffXd>* pout) const {
  VectorBase<AutoDiffXd>& out = *pout;

  const BasicVector<AutoDiffXd> empty(0);
  const AutoDiffXd& time = context.get_time();
  const VectorBase<AutoDiffXd>& state =
      (state_vars_.rows() > 0)
          ? ((time_period_ > 0.0) ? context.get_discrete_state_vector()
                                  : context.get_continuous_state_vector())
          : empty;
  const BasicVector<AutoDiffXd>& input =
      (input_vars_.rows() > 0)
          ? get_input_port().Eval<BasicVector<AutoDiffXd>>(context)
          : empty;

  // Figure out the length of the derivative vector.  Some of the
  // derivatives may have length zero, but we assume (as always) that any
  // properly initialized derivatives in the Context will have a consistent
  // size.
  int num_gradients = time.derivatives().size();
  if (state_vars_.rows() > 0)
    num_gradients = std::max(num_gradients,
                             static_cast<int>(state[0].derivatives().size()));
  if (input_vars_.rows() > 0)
    num_gradients = std::max(num_gradients,
                             static_cast<int>(input[0].derivatives().size()));

  Eigen::MatrixXd dvars(jacobian.cols(), num_gradients);
  Environment env = env_;
  if (time_var_) {
    env[*time_var_] = time.value();
    dvars.bottomRows<1>() = time.derivatives();
  }
  if (state_vars_.size() > 0) {
    for (int i = 0; i < state_vars_.size(); i++) {
      env[state_vars_[i]] = state[i].value();
      dvars.row(i) = state[i].derivatives();
    }
  }
  if (input_vars_.size() > 0) {
    for (int i = 0; i < input_vars_.size(); i++) {
      env[input_vars_[i]] = input[i].value();
      dvars.row(state_vars_.size() + i) = input[i].derivatives();
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
    VectorBase<Expression>* out) const {
  unused(jacobian);
  symbolic::Substitution s;
  PopulateFromContext(context, &s);
  for (int i = 0; i < out->size(); i++) {
    out->SetAtIndex(i, expr[i].Substitute(s));
  }
}

template <typename T>
void SymbolicVectorSystem<T>::CalcOutput(const Context<T>& context,
                                         BasicVector<T>* output_vector) const {
  DRAKE_DEMAND(output_.size() > 0);
  EvaluateWithContext(context, output_, output_jacobian_, output_vector);
}

template <typename T>
void SymbolicVectorSystem<T>::DoCalcTimeDerivatives(
    const Context<T>& context, ContinuousState<T>* derivatives) const {
  DRAKE_DEMAND(time_period_ == 0.0);
  DRAKE_DEMAND(dynamics_.size() > 0);
  EvaluateWithContext(context, dynamics_, dynamics_jacobian_,
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
                      &updates->get_mutable_vector());
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::SymbolicVectorSystem);

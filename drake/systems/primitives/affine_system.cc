#include "drake/systems/primitives/affine_system.h"

#include <stdexcept>
#include <string>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/eigen_types.h"
#include "drake/common/symbolic.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/leaf_context.h"

namespace drake {
namespace systems {

using std::make_unique;
using std::runtime_error;
using std::string;
using std::unique_ptr;

template <typename T>
TimeVaryingAffineSystem<T>::TimeVaryingAffineSystem(int num_states,
                                                    int num_inputs,
                                                    int num_outputs,
                                                    double time_period)
    : num_states_(num_states),
      num_inputs_(num_inputs),
      num_outputs_(num_outputs),
      time_period_(time_period) {
  DRAKE_DEMAND(num_states_ >= 0);
  DRAKE_DEMAND(num_inputs_ >= 0);
  DRAKE_DEMAND(num_outputs_ >= 0);
  DRAKE_DEMAND(time_period_ >= 0.0);

  // Declare state and input/output ports.
  // Declares the state variables and (potentially) the discrete-time update.
  if (time_period_ == 0.0) {
    this->DeclareContinuousState(num_states_);
  } else {
    this->DeclareContinuousState(0);
    this->DeclareDiscreteState(num_states_);
    this->DeclarePeriodicDiscreteUpdate(time_period_, 0.0);
  }
  if (num_inputs_ > 0)
    this->DeclareInputPort(kVectorValued, num_inputs_);
  if (num_outputs_ > 0) {
    this->DeclareVectorOutputPort(BasicVector<T>(num_outputs_),
                                  &TimeVaryingAffineSystem::CalcOutputY);
  }
}

template <typename T>
const InputPortDescriptor<T>& TimeVaryingAffineSystem<T>::get_input_port()
    const {
  DRAKE_DEMAND(num_inputs_ > 0);
  return System<T>::get_input_port(0);
}

template <typename T>
const OutputPort<T>& TimeVaryingAffineSystem<T>::get_output_port()
    const {
  DRAKE_DEMAND(num_outputs_ > 0);
  return System<T>::get_output_port(0);
}

// This is the default implementation for this virtual method.
template <typename T>
void TimeVaryingAffineSystem<T>::CalcOutputY(
    const Context<T>& context, BasicVector<T>* output_vector) const {
  const T t = context.get_time();

  VectorX<T> y = y0(t);
  DRAKE_DEMAND(y.rows() == num_outputs_);

  if (num_states_ > 0) {
    const MatrixX<T> Ct = C(t);
    DRAKE_DEMAND(Ct.rows() == num_outputs_ && Ct.cols() == num_states_);
    const auto& x = dynamic_cast<const BasicVector<T>&>(
                        context.get_continuous_state_vector())
                        .get_value();
    y += Ct * x;
  }

  if (num_inputs_ > 0) {
    const BasicVector<T>* input = this->EvalVectorInput(context, 0);
    DRAKE_DEMAND(input);
    const auto& u = input->get_value();
    const MatrixX<T> Dt = D(t);
    DRAKE_DEMAND(Dt.rows() == num_outputs_ && Dt.cols() == num_inputs_);
    y += Dt * u;
  }

  output_vector->SetFromVector(y);
}

template <typename T>
void TimeVaryingAffineSystem<T>::DoCalcTimeDerivatives(
    const Context<T>& context, ContinuousState<T>* derivatives) const {
  if (num_states_ == 0) return;

  const T t = context.get_time();

  VectorX<T> xdot = f0(t);
  DRAKE_DEMAND(xdot.rows() == num_states_);

  const auto& x =
      dynamic_cast<const BasicVector<T>&>(context.get_continuous_state_vector())
          .get_value();
  const MatrixX<T> At = A(t);
  DRAKE_DEMAND(At.rows() == num_states_ && At.cols() == num_states_);
  xdot += At * x;

  if (num_inputs_ > 0) {
    const BasicVector<T>* input = this->EvalVectorInput(context, 0);
    DRAKE_DEMAND(input);
    const auto& u = input->get_value();

    const MatrixX<T> Bt = B(t);
    DRAKE_DEMAND(Bt.rows() == num_states_ && Bt.cols() == num_inputs_);
    xdot += Bt * u;
  }
  derivatives->SetFromVector(xdot);
}

template <typename T>
void TimeVaryingAffineSystem<T>::DoCalcDiscreteVariableUpdates(
    const drake::systems::Context<T>& context,
    const std::vector<const drake::systems::DiscreteUpdateEvent<T>*>&,
    drake::systems::DiscreteValues<T>* updates) const {
  if (num_states_ == 0 || time_period_ == 0.0) return;

  const T t = context.get_time();

  // TODO(russt): consider demanding that t is a multiple of time_period_.
  // But this could be non-trivial for non-double T.

  VectorX<T> xn = f0(t);
  DRAKE_DEMAND(xn.rows() == num_states_);

  const auto& x = context.get_discrete_state(0)->get_value();

  const MatrixX<T> At = A(t);
  DRAKE_DEMAND(At.rows() == num_states_ && At.cols() == num_states_);
  xn += At * x;

  if (num_inputs_ > 0) {
    const BasicVector<T>* input = this->EvalVectorInput(context, 0);
    DRAKE_DEMAND(input);
    const auto& u = input->get_value();

    const MatrixX<T> Bt = B(t);
    DRAKE_DEMAND(Bt.rows() == num_states_ && Bt.cols() == num_inputs_);
    xn += Bt * u;
  }
  updates->get_mutable_vector()->SetFromVector(xn);
}

template class TimeVaryingAffineSystem<double>;
template class TimeVaryingAffineSystem<AutoDiffXd>;

template <typename T>
AffineSystem<T>::AffineSystem(const Eigen::Ref<const Eigen::MatrixXd>& A,
                              const Eigen::Ref<const Eigen::MatrixXd>& B,
                              const Eigen::Ref<const Eigen::VectorXd>& f0,
                              const Eigen::Ref<const Eigen::MatrixXd>& C,
                              const Eigen::Ref<const Eigen::MatrixXd>& D,
                              const Eigen::Ref<const Eigen::VectorXd>& y0,
                              double time_period)
    : TimeVaryingAffineSystem<T>(f0.size(), D.cols(), D.rows(), time_period),
      A_(A),
      B_(B),
      f0_(f0),
      C_(C),
      D_(D),
      y0_(y0) {
  DRAKE_DEMAND(this->num_states() == A.rows());
  DRAKE_DEMAND(this->num_states() == A.cols());
  DRAKE_DEMAND(this->num_states() == B.rows());
  DRAKE_DEMAND(this->num_states() == C.cols());
  DRAKE_DEMAND(this->num_inputs() == B.cols());
  DRAKE_DEMAND(this->num_inputs() == D.cols());
  DRAKE_DEMAND(this->num_outputs() == C.rows());
  DRAKE_DEMAND(this->num_outputs() == D.rows());
}

namespace {
using symbolic::Expression;
using symbolic::Monomial;
using symbolic::Polynomial;
using symbolic::Variable;
using symbolic::Variables;

void ThrowNonAffineError(const string& msg) {
  throw runtime_error(
      "DecomposeAffineSystem detects that a non-affine expression: " + msg +
      ".");
}

// A helper function for DecomposeAffineSystem. It finds the coefficient of the
// monomial `m` in the `map` and fills `M(i)` with the value. If the monomial
// `m` does not appear in `map`, it uses `0.0` instead. If the coefficient is
// not a constant, it throws a runtime_error.
template <typename Derived>
void DecomposeAffineSystemFindAndFill(const Polynomial::MapType& map,
                                      const Monomial& m, const int i,
                                      Eigen::MatrixBase<Derived> const& M) {
  const auto it = map.find(m);
  // Here, we use const_cast hack. See
  // https://eigen.tuxfamily.org/dox/TopicFunctionTakingEigenTypes.html for
  // details.
  Eigen::MatrixBase<Derived>& M_dummy{
      const_cast<Eigen::MatrixBase<Derived>&>(M)};
  if (it != map.end()) {
    // m should have a constant coefficient.
    if (!is_constant(it->second)) {
      ThrowNonAffineError(it->second.to_string());
    }
    M_dummy(i) = get_constant_value(it->second);
  } else {
    M_dummy(i) = 0.0;
  }
}

// Decomposes `expressions` into `M1 * vars1 + M2 * vars2 + v`. This is a helper
// function used to implement AffineSystem<T>::MakeAffineSystem() method.
//
// Note that it throws runtime_error if `expressions` is not affine in `vars1`
// and `vars2`.
void DecomposeAffineSystem(
    const Eigen::Ref<const VectorX<Expression>>& expressions,
    const Eigen::Ref<const VectorX<symbolic::Variable>> vars1,
    const Eigen::Ref<const VectorX<symbolic::Variable>> vars2,
    Eigen::MatrixXd* const M1, Eigen::MatrixXd* const M2,
    Eigen::VectorXd* const v) {
  for (int i = 0; i < expressions.size(); ++i) {
    const Expression& e{expressions(i)};
    if (!e.is_polynomial()) {
      ThrowNonAffineError(e.to_string());  // e should be a polynomial.
    }
    const Polynomial p{e, Variables{vars1} + Variables{vars2}};
    if (p.TotalDegree() > 1) {
      ThrowNonAffineError(e.to_string());  // e should be affine.
    }
    const Polynomial::MapType& map{p.monomial_to_coefficient_map()};
    // Fill M1(i, j).
    for (int j = 0; j < vars1.size(); ++j) {
      DecomposeAffineSystemFindAndFill(map, Monomial{vars1.coeff(j)}, j,
                                       M1->row(i));
    }
    // Fill M2(i, j).
    for (int j = 0; j < vars2.size(); ++j) {
      DecomposeAffineSystemFindAndFill(map, Monomial{vars2.coeff(j)}, j,
                                       M2->row(i));
    }
    // Fill v(i).
    DecomposeAffineSystemFindAndFill(map, Monomial{}, i, *v);
  }
}
}  // namespace

template <typename T>
unique_ptr<AffineSystem<T>> AffineSystem<T>::MakeAffineSystem(
    const Eigen::Ref<const VectorX<symbolic::Expression>>& dynamics,
    const Eigen::Ref<const VectorX<symbolic::Expression>>& output,
    const Eigen::Ref<const VectorX<symbolic::Variable>>& state_vars,
    const Eigen::Ref<const VectorX<symbolic::Variable>>& input_vars,
    const double time_period) {
  // Need to extract, A, B, f₀, C, D, y₀ such that,
  //
  //     dynamics = Ax + Bu + f₀
  //     output   = Cx + Du + y₀
  //
  // where x = state_vars and u = input_vars.
  const int num_states = state_vars.size();
  DRAKE_ASSERT(num_states == dynamics.size());
  const int num_inputs = input_vars.size();
  const int num_outputs = output.size();
  Eigen::MatrixXd A(num_states, num_states);
  Eigen::MatrixXd B(num_states, num_inputs);
  Eigen::VectorXd f0(num_states);
  Eigen::MatrixXd C(num_outputs, num_states);
  Eigen::MatrixXd D(num_outputs, num_inputs);
  Eigen::VectorXd y0(num_outputs);
  DecomposeAffineSystem(dynamics, state_vars, input_vars, &A, &B, &f0);
  DecomposeAffineSystem(output, state_vars, input_vars, &C, &D, &y0);
  return make_unique<AffineSystem<T>>(A, B, f0, C, D, y0, time_period);
}

// Setup equivalent system with a different scalar type.
template <typename T>
AffineSystem<AutoDiffXd>* AffineSystem<T>::DoToAutoDiffXd() const {
  return new AffineSystem<AutoDiffXd>(A_, B_, f0_, C_, D_, y0_,
                                      this->time_period());
}

template <typename T>
AffineSystem<symbolic::Expression>* AffineSystem<T>::DoToSymbolic() const {
  return new AffineSystem<symbolic::Expression>(A_, B_, f0_, C_, D_, y0_,
                                                this->time_period());
}

template <typename T>
void AffineSystem<T>::CalcOutputY(const Context<T>& context,
                                  BasicVector<T>* output_vector) const {
  const auto& x =
      dynamic_cast<const BasicVector<T>&>(context.get_continuous_state_vector())
          .get_value();

  auto y = output_vector->get_mutable_value();
  y = C_ * x + y0_;

  if (this->num_inputs()) {
    const BasicVector<T>* input = this->EvalVectorInput(context, 0);
    DRAKE_DEMAND(input);
    const auto& u = input->get_value();
    y += D_ * u;
  }
}

template <typename T>
void AffineSystem<T>::DoCalcTimeDerivatives(
    const Context<T>& context, ContinuousState<T>* derivatives) const {
  if (this->num_states() == 0 || this->time_period() > 0.0) return;

  const auto& x =
      dynamic_cast<const BasicVector<T>&>(context.get_continuous_state_vector())
          .get_value();

  VectorX<T> xdot = A_ * x + f0_;

  if (this->num_inputs() > 0) {
    const BasicVector<T>* input = this->EvalVectorInput(context, 0);
    DRAKE_DEMAND(input);
    const auto& u = input->get_value();

    xdot += B_ * u;
  }
  derivatives->SetFromVector(xdot);
}

template <typename T>
void AffineSystem<T>::DoCalcDiscreteVariableUpdates(
    const drake::systems::Context<T>& context,
    const std::vector<const drake::systems::DiscreteUpdateEvent<T>*>&,
    drake::systems::DiscreteValues<T>* updates) const {
  if (this->num_states() == 0 || this->time_period() == 0.0) return;

  const auto& x = context.get_discrete_state(0)->get_value();

  VectorX<T> xnext = A_ * x + f0_;

  if (this->num_inputs() > 0) {
    const BasicVector<T>* input = this->EvalVectorInput(context, 0);
    DRAKE_DEMAND(input);
    const auto& u = input->get_value();

    xnext += B_ * u;
  }
  updates->get_mutable_vector()->SetFromVector(xnext);
}

template class AffineSystem<double>;
template class AffineSystem<AutoDiffXd>;

}  // namespace systems
}  // namespace drake

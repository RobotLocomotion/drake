#include "drake/systems/primitives/affine_system.h"

#include <utility>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/common/symbolic.h"
#include "drake/common/symbolic_decompose.h"
#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace systems {

using std::make_unique;
using std::unique_ptr;

template <typename T>
TimeVaryingAffineSystem<T>::TimeVaryingAffineSystem(
    SystemScalarConverter converter,
    int num_states, int num_inputs, int num_outputs, double time_period)
    : LeafSystem<T>(std::move(converter)),
      num_states_(num_states),
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
    const VectorX<T>& x = (this->time_period() == 0.)
        ? dynamic_cast<const BasicVector<T>&>(
            context.get_continuous_state_vector()).get_value()
        : context.get_discrete_state().get_vector().get_value();
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

  const auto& x = context.get_discrete_state(0).get_value();

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
  updates->get_mutable_vector().SetFromVector(xn);
}

// Our public constructor declares that our most specific subclass is
// AffineSystem, and then delegates to our protected constructor.
template <typename T>
AffineSystem<T>::AffineSystem(const Eigen::Ref<const Eigen::MatrixXd>& A,
                              const Eigen::Ref<const Eigen::MatrixXd>& B,
                              const Eigen::Ref<const Eigen::VectorXd>& f0,
                              const Eigen::Ref<const Eigen::MatrixXd>& C,
                              const Eigen::Ref<const Eigen::MatrixXd>& D,
                              const Eigen::Ref<const Eigen::VectorXd>& y0,
                              double time_period)
    : AffineSystem<T>(
          SystemTypeTag<systems::AffineSystem>{},
          A, B, f0, C, D, y0, time_period) {}

// Our protected constructor does all of the real work -- everything else
// delegates to here.
template <typename T>
AffineSystem<T>::AffineSystem(SystemScalarConverter converter,
                              const Eigen::Ref<const Eigen::MatrixXd>& A,
                              const Eigen::Ref<const Eigen::MatrixXd>& B,
                              const Eigen::Ref<const Eigen::VectorXd>& f0,
                              const Eigen::Ref<const Eigen::MatrixXd>& C,
                              const Eigen::Ref<const Eigen::MatrixXd>& D,
                              const Eigen::Ref<const Eigen::VectorXd>& y0,
                              double time_period)
    : TimeVaryingAffineSystem<T>(
          std::move(converter), f0.size(), D.cols(), D.rows(), time_period),
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

// Our copy constructor delegates to the public constructor; this used only by
// SystemScalarConverter as known to our public constructor, not by subclasses.
template <typename T>
template <typename U>
AffineSystem<T>::AffineSystem(const AffineSystem<U>& other)
    : AffineSystem(other.A(), other.B(), other.f0(), other.C(), other.D(),
                   other.y0(), other.time_period()) {}

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

  Eigen::MatrixXd AB(num_states, num_states + num_inputs);
  Eigen::VectorXd f0(num_states);
  VectorX<symbolic::Variable> vars(num_states + num_inputs);
  vars << state_vars, input_vars;
  DecomposeAffineExpressions(dynamics, vars, &AB, &f0);
  const auto A = AB.leftCols(num_states);
  const auto B = AB.rightCols(num_inputs);

  Eigen::MatrixXd CD(num_outputs, num_states + num_inputs);
  Eigen::VectorXd y0(num_outputs);
  DecomposeAffineExpressions(output, vars, &CD, &y0);
  const auto C = CD.leftCols(num_states);
  const auto D = CD.rightCols(num_inputs);

  return make_unique<AffineSystem<T>>(A, B, f0, C, D, y0, time_period);
}

template <typename T>
void AffineSystem<T>::CalcOutputY(const Context<T>& context,
                                  BasicVector<T>* output_vector) const {
  const VectorX<T>& x = (this->time_period() == 0.)
      ? dynamic_cast<const BasicVector<T>&>(
          context.get_continuous_state_vector()).get_value()
      : context.get_discrete_state().get_vector().get_value();

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

  const auto& x = context.get_discrete_state(0).get_value();

  VectorX<T> xnext = A_ * x + f0_;

  if (this->num_inputs() > 0) {
    const BasicVector<T>* input = this->EvalVectorInput(context, 0);
    DRAKE_DEMAND(input);
    const auto& u = input->get_value();

    xnext += B_ * u;
  }
  updates->get_mutable_vector().SetFromVector(xnext);
}


}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::TimeVaryingAffineSystem)

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::AffineSystem)

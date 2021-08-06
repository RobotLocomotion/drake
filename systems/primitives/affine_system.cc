#include "drake/systems/primitives/affine_system.h"

#include <set>
#include <utility>

#include <Eigen/Eigenvalues>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_throw.h"
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
    SystemScalarConverter converter, int num_states, int num_inputs,
    int num_outputs, double time_period)
    : LeafSystem<T>(std::move(converter)),
      num_states_(num_states),
      num_inputs_(num_inputs),
      num_outputs_(num_outputs),
      time_period_(time_period),
      x0_(VectorX<T>::Zero(num_states)),
      Sqrt_Sigma_x0_(Eigen::MatrixXd::Zero(num_states, num_states)) {
  DRAKE_DEMAND(num_states_ >= 0);
  DRAKE_DEMAND(num_inputs_ >= 0);
  DRAKE_DEMAND(num_outputs_ >= 0);
  DRAKE_DEMAND(time_period_ >= 0.0);

  // Declare state and input/output ports.
  // Declares the state variables and (potentially) the discrete-time update.
  if (num_states > 0) {
    if (time_period_ == 0.0) {
      this->DeclareContinuousState(num_states_);
    } else {
      this->DeclareDiscreteState(num_states_);
      this->DeclarePeriodicDiscreteUpdate(time_period_, 0.0);
    }
  }

  if (num_inputs_ > 0)
    this->DeclareInputPort(kUseDefaultName, kVectorValued, num_inputs_);
  if (num_outputs_ > 0) {
    // N.B. Subclasses that override CalcOutputY may want to fine-tune the
    // output port's prerequisites; see AffineSystem's ctor for an example.
    // By default, the output port will depend on everything (time, input,
    // state, accuracy, etc.).
    this->DeclareVectorOutputPort(kUseDefaultName, num_outputs_,
                                  &TimeVaryingAffineSystem::CalcOutputY);
  }
}

template <typename T>
const InputPort<T>& TimeVaryingAffineSystem<T>::get_input_port()
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

template <typename T>
void TimeVaryingAffineSystem<T>::configure_default_state(
    const Eigen::Ref<const VectorX<T>>& x0) {
  DRAKE_DEMAND(x0.rows() == num_states_);
  if (num_states_ == 0) return;
  x0_ = x0;
}

template <typename T>
void TimeVaryingAffineSystem<T>::configure_random_state(
    const Eigen::Ref<const Eigen::MatrixXd>& covariance) {
  DRAKE_DEMAND(covariance.rows() == num_states_);
  DRAKE_DEMAND(covariance.cols() == num_states_);
  if (num_states_ == 0) return;
  Sqrt_Sigma_x0_ = Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd>(covariance)
                        .operatorSqrt();
}

template <typename T>
template <typename U>
void TimeVaryingAffineSystem<T>::ConfigureDefaultAndRandomStateFrom(
    const TimeVaryingAffineSystem<U>& other) {
  // Convert default state from U -> double -> T.
  VectorX<T> x0(other.num_states());
  const VectorX<U>& other_x0 = other.get_default_state();
  for (int i = 0; i < other.num_states(); i++) {
    x0[i] = ExtractDoubleOrThrow(other_x0[i]);
  }
  this->configure_default_state(x0);
  this->configure_random_state(other.get_random_state_covariance());
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
    const auto& u = get_input_port().Eval(context);
    const MatrixX<T> Dt = D(t);
    DRAKE_DEMAND(Dt.rows() == num_outputs_ && Dt.cols() == num_inputs_);
    y += Dt * u;
  }

  output_vector->SetFromVector(y);
}

template <typename T>
void TimeVaryingAffineSystem<T>::DoCalcTimeDerivatives(
    const Context<T>& context, ContinuousState<T>* derivatives) const {
  if (num_states_ == 0 || time_period_ > 0) return;

  const T t = context.get_time();

  VectorX<T> xdot = f0(t);
  DRAKE_THROW_UNLESS(xdot.rows() == num_states_);

  const auto& x =
      dynamic_cast<const BasicVector<T>&>(context.get_continuous_state_vector())
          .get_value();
  const MatrixX<T> At = A(t);
  DRAKE_THROW_UNLESS(At.rows() == num_states_ && At.cols() == num_states_);
  xdot += At * x;

  if (num_inputs_ > 0) {
    const auto& u = get_input_port().Eval(context);

    const MatrixX<T> Bt = B(t);
    DRAKE_THROW_UNLESS(Bt.rows() == num_states_ && Bt.cols() == num_inputs_);
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
    const auto& u = get_input_port().Eval(context);

    const MatrixX<T> Bt = B(t);
    DRAKE_DEMAND(Bt.rows() == num_states_ && Bt.cols() == num_inputs_);
    xn += Bt * u;
  }
  updates->set_value(xn);
}

template <typename T>
void TimeVaryingAffineSystem<T>::SetDefaultState(const Context<T>& context,
                                                 State<T>* state) const {
  unused(context);
  if (num_states_ == 0) return;

  if (time_period_ == 0.0) {
    state->get_mutable_continuous_state().SetFromVector(x0_);
  } else {
    state->get_mutable_discrete_state(0).SetFromVector(x0_);
  }
}

template <typename T>
void TimeVaryingAffineSystem<T>::SetRandomState(
    const Context<T>& context, State<T>* state,
    RandomGenerator* generator) const {
  unused(context);
  if (num_states_ == 0) return;

  Eigen::VectorXd w(num_states_);
  std::normal_distribution<double> normal;
  for (int i = 0; i < num_states_; i++) {
    w[i] = normal(*generator);
  }
  const auto x0 = x0_ + Sqrt_Sigma_x0_ * w;
  if (time_period_ == 0.0) {
    state->get_mutable_continuous_state().SetFromVector(x0);
  } else {
    state->get_mutable_discrete_state(0).SetFromVector(x0);
  }
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
          SystemTypeTag<AffineSystem>{},
          A, B, f0, C, D, y0, time_period) {}

namespace {

// Returns whether a matrix is "meaningful" when pre-multiplying a vector.
bool IsMeaningful(const Eigen::MatrixXd& m) {
  return m.size() > 0 && (m.array() != 0).any();
}

}  // namespace

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
      y0_(y0),
      has_meaningful_C_(IsMeaningful(C)),
      has_meaningful_D_(IsMeaningful(D)) {
  DRAKE_DEMAND(this->num_states() == A.rows());
  DRAKE_DEMAND(this->num_states() == A.cols());
  DRAKE_DEMAND(this->num_states() == B.rows());
  DRAKE_DEMAND(this->num_states() == C.cols());
  DRAKE_DEMAND(this->num_inputs() == B.cols());
  DRAKE_DEMAND(this->num_inputs() == D.cols());
  DRAKE_DEMAND(this->num_outputs() == C.rows());
  DRAKE_DEMAND(this->num_outputs() == D.rows());

  // Specify our output port's dependencies more precisely than our base class
  // is able to.  We know that output never depends on time nor parameters,
  // only on state (iff C if non-zero) and input (iff D is non-zero).
  if (this->num_outputs() > 0) {
    const OutputPort<T>& output_port = this->get_output_port();
    const auto& leaf_port = dynamic_cast<const LeafOutputPort<T>&>(output_port);
    const CacheIndex cache_index = leaf_port.cache_entry().cache_index();
    CacheEntry& cache_entry = this->get_mutable_cache_entry(cache_index);
    std::set<DependencyTicket>& prereqs = cache_entry.mutable_prerequisites();
    prereqs.clear();
    if (has_meaningful_C_) {
      prereqs.insert(this->all_state_ticket());
    }
    if (has_meaningful_D_) {
      prereqs.insert(this->all_input_ports_ticket());
    }
  }
}

// Our copy constructor delegates to the public constructor; this used only by
// SystemScalarConverter as known to our public constructor, not by subclasses.
template <typename T>
template <typename U>
AffineSystem<T>::AffineSystem(const AffineSystem<U>& other)
    : AffineSystem(other.A(), other.B(), other.f0(), other.C(), other.D(),
                   other.y0(), other.time_period()) {
  this->ConfigureDefaultAndRandomStateFrom(other);
}

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
  DRAKE_DEMAND(num_states == dynamics.size());
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
  auto y = output_vector->get_mutable_value();
  y = y0_;

  if (has_meaningful_C_) {
    const VectorX<T>& x = (this->time_period() == 0.)
        ? dynamic_cast<const BasicVector<T>&>(
            context.get_continuous_state_vector()).get_value()
        : context.get_discrete_state().get_vector().get_value();
    y += C_ * x;
  }

  if (has_meaningful_D_) {
    const auto& u = this->get_input_port().Eval(context);
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
    const auto& u = this->get_input_port().Eval(context);

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
    const auto& u = this->get_input_port().Eval(context);

    xnext += B_ * u;
  }
  updates->set_value(xnext);
}

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS((
    &TimeVaryingAffineSystem<T>::template ConfigureDefaultAndRandomStateFrom<U>
))

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::TimeVaryingAffineSystem)

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::AffineSystem)

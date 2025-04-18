#include "drake/systems/estimators/test_utilities/stochastic_linear_system.h"

namespace drake {
namespace systems {
namespace estimators_test {

template <typename T>
StochasticLinearSystem<T>::StochasticLinearSystem(
    const Eigen::Ref<const Eigen::MatrixXd>& A,
    const Eigen::Ref<const Eigen::MatrixXd>& B,
    const Eigen::Ref<const Eigen::MatrixXd>& G,
    const Eigen::Ref<const Eigen::MatrixXd>& C,
    const Eigen::Ref<const Eigen::MatrixXd>& D,
    const Eigen::Ref<const Eigen::MatrixXd>& H, double time_period)
    : LeafSystem<T>(SystemTypeTag<StochasticLinearSystem>{}),
      A_(A),
      B_(B),
      G_(G),
      C_(C),
      D_(D),
      H_(H),
      time_period_(time_period) {
  const int num_states = A.rows();
  const int num_inputs = B.cols();
  const int num_outputs = C.rows();
  DRAKE_THROW_UNLESS(A.rows() == num_states && A.cols() == num_states);
  DRAKE_THROW_UNLESS(B.rows() == num_states && B.cols() == num_inputs);
  DRAKE_THROW_UNLESS(G.rows() == num_states && G.cols() > 0);
  DRAKE_THROW_UNLESS(C.rows() == num_outputs && C.cols() == num_states);
  DRAKE_THROW_UNLESS(D.rows() == num_outputs && D.cols() == num_inputs);
  DRAKE_THROW_UNLESS(H.rows() == num_outputs && H.cols() > 0);
  DRAKE_THROW_UNLESS(time_period >= 0.0);

  this->DeclareVectorInputPort("u", num_inputs);
  this->DeclareVectorInputPort("w", G.cols(), RandomDistribution::kGaussian);
  this->DeclareVectorInputPort("v", H.cols(), RandomDistribution::kGaussian);
  this->DeclareVectorOutputPort(
      "y", num_outputs, &StochasticLinearSystem::CalculateOutput,
      {this->input_port_ticket(InputPortIndex(0)),
       this->input_port_ticket(InputPortIndex(2)), this->all_state_ticket()});

  if (time_period == 0) {
    this->DeclareContinuousState(num_states);
  } else {
    this->DeclareDiscreteState(num_states);
    this->DeclarePeriodicDiscreteUpdateEvent(
        time_period, 0.0, &StochasticLinearSystem::DiscreteUpdate);
  }
}

template <typename T>
StochasticLinearSystem<T>::StochasticLinearSystem(
    const LinearSystem<T>& sys, const Eigen::Ref<const Eigen::MatrixXd>& G,
    const Eigen::Ref<const Eigen::MatrixXd>& H)
    : StochasticLinearSystem(
          sys.A(), sys.B(), G, sys.C(), sys.D(), H,
          sys.GetUniquePeriodicDiscreteUpdateAttribute().has_value()
              ? sys.GetUniquePeriodicDiscreteUpdateAttribute()
                    .value()
                    .period_sec()
              : 0.0) {}

template <typename T>
template <typename U>
StochasticLinearSystem<T>::StochasticLinearSystem(
    const StochasticLinearSystem<U>& other)
    : StochasticLinearSystem(other.A_, other.B_, other.G_, other.C_, other.D_,
                             other.H_, other.time_period_) {}

template <typename T>
StochasticLinearSystem<T>::~StochasticLinearSystem() = default;

template <typename T>
const InputPort<T>& StochasticLinearSystem<T>::get_u_input_port() const {
  return this->get_input_port(0);
}

template <typename T>
const InputPort<T>& StochasticLinearSystem<T>::get_w_input_port() const {
  return this->get_input_port(1);
}

template <typename T>
const InputPort<T>& StochasticLinearSystem<T>::get_v_input_port() const {
  return this->get_input_port(2);
}

template <typename T>
const OutputPort<T>& StochasticLinearSystem<T>::get_y_output_port() const {
  return this->get_output_port(0);
}

template <typename T>
void StochasticLinearSystem<T>::DoCalcTimeDerivatives(
    const Context<T>& context, ContinuousState<T>* derivatives) const {
  Eigen::VectorX<T> x =
      context.has_only_continuous_state()
          ? context.get_continuous_state_vector().CopyToVector()
          : context.get_discrete_state_vector().CopyToVector();
  const Eigen::VectorX<T>& u = get_u_input_port().Eval(context);
  const Eigen::VectorX<T>& w = get_w_input_port().Eval(context);
  Eigen::VectorX<T> xdot = A_ * x;
  xdot += B_ * u;
  xdot += G_ * w;
  derivatives->SetFromVector(xdot);
}

template <typename T>
void StochasticLinearSystem<T>::DiscreteUpdate(
    const Context<T>& context, DiscreteValues<T>* update) const {
  Eigen::VectorX<T> x =
      context.has_only_continuous_state()
          ? context.get_continuous_state_vector().CopyToVector()
          : context.get_discrete_state_vector().CopyToVector();
  const Eigen::VectorX<T>& u = get_u_input_port().Eval(context);
  const Eigen::VectorX<T>& w = get_w_input_port().Eval(context);
  auto x_next = update->get_mutable_value();
  x_next.setZero();
  x_next += A_ * x;
  x_next += B_ * u;
  x_next += G_ * w;
}

template <typename T>
void StochasticLinearSystem<T>::CalculateOutput(const Context<T>& context,
                                                BasicVector<T>* out) const {
  Eigen::VectorX<T> x =
      context.has_only_continuous_state()
          ? context.get_continuous_state_vector().CopyToVector()
          : context.get_discrete_state_vector().CopyToVector();
  Eigen::VectorX<T> u = get_u_input_port().Eval(context);
  Eigen::VectorX<T> v = get_v_input_port().Eval(context);
  auto y = out->get_mutable_value();
  y.setZero();
  y += C_ * x;
  y += D_ * u;
  y += H_ * v;
}

}  // namespace estimators_test
}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::estimators_test::StochasticLinearSystem);

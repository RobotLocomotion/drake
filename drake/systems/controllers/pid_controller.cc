#include "drake/systems/controllers/pid_controller.h"

#include <string>

#include "drake/common/autodiff_overloads.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/symbolic.h"

namespace drake {
namespace systems {
namespace controllers {

template <typename T>
PidController<T>::PidController(const Eigen::VectorXd& kp,
                                const Eigen::VectorXd& ki,
                                const Eigen::VectorXd& kd)
    : PidController(MatrixX<double>::Identity(2 * kp.size(), 2 * kp.size()), kp,
                    ki, kd) {}

template <typename T>
PidController<T>::PidController(const MatrixX<double>& state_selector,
                                const Eigen::VectorXd& kp,
                                const Eigen::VectorXd& ki,
                                const Eigen::VectorXd& kd)
<<<<<<< HEAD
    : PidController(MatrixX<double>::Identity(kp.size(), kp.size()),
                    state_selector, kp, ki, kd) {}

template <typename T>
PidController<T>::PidController(const MatrixX<double>& Binv,
                                const MatrixX<double>& state_selector,
                                const Eigen::VectorXd& kp,
                                const Eigen::VectorXd& ki,
                                const Eigen::VectorXd& kd)
    : kp_(kp),
      kd_(kd),
=======
    : LeafSystem<T>(SystemTypeTag<controllers::PidController>{}),
      kp_(kp),
>>>>>>> d62ba8084102d4f65a3685e58edec9415863c3bd
      ki_(ki),
      kd_(kd),
      num_controlled_q_(kp.size()),
      num_full_state_(state_selector.cols()),
      Binv_(Binv),
      state_selector_(state_selector) {
  DRAKE_DEMAND(kp_.size() == kd_.size());
  DRAKE_DEMAND(kd_.size() == ki_.size());
  DRAKE_DEMAND(state_selector_.rows() == 2 * num_controlled_q_);

  this->DeclareContinuousState(num_controlled_q_);

  output_index_control_ =
      this->DeclareVectorOutputPort(BasicVector<T>(num_controlled_q_),
                                    &PidController<T>::CalcControl)
          .get_index();

  input_index_state_ =
      this->DeclareInputPort(kVectorValued, num_full_state_).get_index();

  input_index_desired_state_ =
      this->DeclareInputPort(kVectorValued, 2 * num_controlled_q_).get_index();
}

template <typename T>
template <typename U>
PidController<T>::PidController(const PidController<U>& other)
    : PidController(
          other.state_selector_,
          other.kp_,
          other.ki_,
          other.kd_) {}

template <typename T>
void PidController<T>::DoCalcTimeDerivatives(
    const Context<T>& context, ContinuousState<T>* derivatives) const {
  const Eigen::VectorBlock<const VectorX<T>> state =
      this->EvalEigenVectorInput(context, input_index_state_);
  const Eigen::VectorBlock<const VectorX<T>> state_d =
      this->EvalEigenVectorInput(context, input_index_desired_state_);

  // The derivative of the continuous state is the instantaneous position error.
  VectorBase<T>* const derivatives_vector = derivatives->get_mutable_vector();
  const VectorX<T> controlled_state_diff =
      state_d - (state_selector_.cast<T>() * state);
  derivatives_vector->SetFromVector(
      controlled_state_diff.head(num_controlled_q_));
}

template <typename T>
void PidController<T>::CalcControl(const Context<T>& context,
                                   BasicVector<T>* control) const {
  const Eigen::VectorBlock<const VectorX<T>> state =
      this->EvalEigenVectorInput(context, input_index_state_);
  const Eigen::VectorBlock<const VectorX<T>> state_d =
      this->EvalEigenVectorInput(context, input_index_desired_state_);

  // State error.
  const VectorX<T> controlled_state_diff =
      state_d - (state_selector_.cast<T>() * state);

  // Intergral error, which is stored in the continuous state.
  const VectorBase<T>& state_vector = context.get_continuous_state_vector();
  const Eigen::VectorBlock<const VectorX<T>> state_block =
      dynamic_cast<const BasicVector<T>&>(state_vector).get_value();

  // Sets output to the sum of all three terms.
  control->SetFromVector(
      Binv_ *
      ((kp_.array() * controlled_state_diff.head(num_controlled_q_).array())
           .matrix() +
       (kd_.array() * controlled_state_diff.tail(num_controlled_q_).array())
           .matrix() +
       (ki_.array() * state_block.array()).matrix()));
}

// Adds a simple record-based representation of the PID controller to @p dot.
template <typename T>
void PidController<T>::GetGraphvizFragment(std::stringstream* dot) const {
  std::string name = this->get_name();
  if (name.empty()) {
    name = "PID Controller";
  }
  *dot << this->GetGraphvizId() << " [shape=record, label=\"" << name;
  *dot << " | { {<u0> x |<u1> x_d} |<y0> y}";
  *dot << "\"];" << std::endl;
}

template class PidController<double>;
template class PidController<AutoDiffXd>;
template class PidController<symbolic::Expression>;

}  // namespace controllers
}  // namespace systems
}  // namespace drake

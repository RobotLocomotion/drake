#include "drake/systems/controllers/pid_controller.h"

#include <string>

#include "drake/common/default_scalars.h"

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
PidController<T>::PidController(const MatrixX<double>& state_projection,
                                const Eigen::VectorXd& kp,
                                const Eigen::VectorXd& ki,
                                const Eigen::VectorXd& kd)
    : PidController(state_projection,
                    MatrixX<double>::Identity(kp.size(), kp.size()), kp, ki,
                    kd) {}

template <typename T>
PidController<T>::PidController(const MatrixX<double>& state_projection,
                                const MatrixX<double>& output_projection,
                                const Eigen::VectorXd& kp,
                                const Eigen::VectorXd& ki,
                                const Eigen::VectorXd& kd)
    : LeafSystem<T>(SystemTypeTag<PidController>{}),
      kp_(kp),
      ki_(ki),
      kd_(kd),
      num_controlled_q_(kp.size()),
      num_full_state_(state_projection.cols()),
      state_projection_(state_projection),
      output_projection_(output_projection) {
  if (kp_.size() != kd_.size() || kd_.size() != ki_.size()) {
    throw std::logic_error(
        "Gains must have equal length: |Kp| = " + std::to_string(kp_.size()) +
        ", |Ki| = " + std::to_string(ki_.size()) +
        ", |Kd| = " + std::to_string(kd_.size()));
  }
  if (state_projection_.rows() != 2 * num_controlled_q_) {
    throw std::logic_error(
        "State projection row dimension mismatch, expecting " +
        std::to_string(2 * num_controlled_q_) + ", is " +
        std::to_string(state_projection_.rows()));
  }
  if (output_projection_.cols() != kp_.size()) {
    throw std::logic_error(
        "Output projection column dimension mismatch, expecting " +
        std::to_string(kp_.size()) + ", is " +
        std::to_string(output_projection_.cols()));
  }

  this->DeclareContinuousState(num_controlled_q_);

  output_index_control_ =
      this->DeclareVectorOutputPort("control", output_projection_.rows(),
                                    &PidController<T>::CalcControl)
          .get_index();

  input_index_state_ =
      this->DeclareVectorInputPort("estimated_state", num_full_state_)
          .get_index();

  input_index_desired_state_ =
      this->DeclareInputPort("desired_state", kVectorValued,
                             2 * num_controlled_q_)
          .get_index();
}

template <typename T>
template <typename U>
PidController<T>::PidController(const PidController<U>& other)
    : PidController(other.state_projection_, other.output_projection_,
                    other.kp_, other.ki_, other.kd_) {}

template <typename T>
void PidController<T>::DoCalcTimeDerivatives(
    const Context<T>& context, ContinuousState<T>* derivatives) const {
  const VectorX<T>& state = get_input_port_estimated_state().Eval(context);
  const VectorX<T>& state_d = get_input_port_desired_state().Eval(context);

  // The derivative of the continuous state is the instantaneous position error.
  VectorBase<T>& derivatives_vector = derivatives->get_mutable_vector();
  const VectorX<T> controlled_state_diff =
      state_d - (state_projection_.cast<T>() * state);
  derivatives_vector.SetFromVector(
      controlled_state_diff.head(num_controlled_q_));
}

template <typename T>
void PidController<T>::CalcControl(const Context<T>& context,
                                   BasicVector<T>* control) const {
  const VectorX<T>& state = get_input_port_estimated_state().Eval(context);
  const VectorX<T>& state_d = get_input_port_desired_state().Eval(context);

  // State error.
  const VectorX<T> controlled_state_diff =
      state_d - (state_projection_.cast<T>() * state);

  // Integral error, which is stored in the continuous state.
  const VectorX<T>& state_vector =
      dynamic_cast<const BasicVector<T>&>(context.get_continuous_state_vector())
          .value();

  // Sets output to the sum of all three terms.
  control->SetFromVector(
      output_projection_.cast<T>() *
      ((kp_.array() * controlled_state_diff.head(num_controlled_q_).array())
           .matrix() +
       (kd_.array() * controlled_state_diff.tail(num_controlled_q_).array())
           .matrix() +
       (ki_.array() * state_vector.array()).matrix()));
}

// Adds a simple record-based representation of the PID controller to @p dot.
template <typename T>
void PidController<T>::GetGraphvizFragment(int max_depth,
                                           std::stringstream* dot) const {
  unused(max_depth);
  std::string name = this->get_name();
  if (name.empty()) {
    name = "PID Controller";
  }
  *dot << this->GetGraphvizId() << " [shape=record, label=\"" << name;
  *dot << " | { {<u0> x |<u1> x_d} |<y0> y}";
  *dot << "\"];" << std::endl;
}

}  // namespace controllers
}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::controllers::PidController)

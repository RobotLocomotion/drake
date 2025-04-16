#pragma once

#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/primitives/linear_system.h"

namespace drake {
namespace systems {
namespace estimators_test {

/**
 * A discrete OR continuous stochastic linear system.
 *
 * @system
 * name: StochasticLinearSystem
 * input_ports:
 * - u
 * - w
 * - v
 * output_ports:
 * - y
 * @endsystem
 *
 * If time_period>0.0, then the stochastic linear system will have the following
 * discrete-time state update:
 *   @f[ x[n+1] = A x[n] + B u[n] + G w[n], @f]
 *
 * or if time_period==0.0, then the stochastic linear system will have the
 * following continuous-time state update:
 *   @f[\dot{x} = A x + B u + G w. @f]
 *
 * In both cases, the system will have the output:
 *   @f[ y = C x + D u + H v, @f]
 * where `u` denotes the actuation input vector, `w` denotes the process noise
 * input vector, `v` denotes the measurement noise input vector, `x` denotes the
 * state vector, and `y` denotes the output vector.
 *
 * @tparam_default_scalar
 */
template <typename T>
class StochasticLinearSystem : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(StochasticLinearSystem);

  StochasticLinearSystem(const Eigen::Ref<const Eigen::MatrixXd>& A,
                         const Eigen::Ref<const Eigen::MatrixXd>& B,
                         const Eigen::Ref<const Eigen::MatrixXd>& G,
                         const Eigen::Ref<const Eigen::MatrixXd>& C,
                         const Eigen::Ref<const Eigen::MatrixXd>& D,
                         const Eigen::Ref<const Eigen::MatrixXd>& H,
                         double time_period = 0.0);

  StochasticLinearSystem(const LinearSystem<T>& sys,
                         const Eigen::Ref<const Eigen::MatrixXd>& G,
                         const Eigen::Ref<const Eigen::MatrixXd>& H);

  // Scalar-converting copy constructor.  See @ref system_scalar_conversion.
  template <typename U>
  StochasticLinearSystem(const StochasticLinearSystem<U>& other);

  ~StochasticLinearSystem() override;

  const InputPort<T>& get_u_input_port() const;
  const InputPort<T>& get_w_input_port() const;
  const InputPort<T>& get_v_input_port() const;
  const OutputPort<T>& get_y_output_port() const;

  const Eigen::MatrixXd& A() const { return A_; }
  const Eigen::MatrixXd& B() const { return B_; }
  const Eigen::MatrixXd& G() const { return G_; }
  const Eigen::MatrixXd& C() const { return C_; }
  const Eigen::MatrixXd& D() const { return D_; }
  const Eigen::MatrixXd& H() const { return H_; }

 private:
  template <typename U>
  friend class StochasticLinearSystem;

  void DoCalcTimeDerivatives(const Context<T>& context,
                             ContinuousState<T>* derivatives) const override;

  void DiscreteUpdate(const Context<T>& context,
                      DiscreteValues<T>* update) const;

  void CalculateOutput(const Context<T>& context, BasicVector<T>* out) const;

  const Eigen::MatrixXd A_;
  const Eigen::MatrixXd B_;
  const Eigen::MatrixXd G_;
  const Eigen::MatrixXd C_;
  const Eigen::MatrixXd D_;
  const Eigen::MatrixXd H_;
  const double time_period_;
};

}  // namespace estimators_test
}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::estimators_test::StochasticLinearSystem);

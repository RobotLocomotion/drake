#pragma once

#include <memory>

#include "drake/examples/acrobot/acrobot_plant.h"
#include "drake/examples/acrobot/gen/acrobot_input.h"
#include "drake/examples/acrobot/gen/acrobot_params.h"
#include "drake/examples/acrobot/gen/acrobot_state.h"
#include "drake/math/wrap_to.h"
#include "drake/systems/controllers/linear_quadratic_regulator.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/primitives/linear_system.h"

/// @file The Spong acrobot swing-up controller implemented as a LeafSystem.
///   Spong, Mark W. "Swing up control of the acrobot." Robotics and Automation,
///   1994. Proceedings., 1994 IEEE International Conference on. IEEE, 1994.
///
/// Note that the Spong controller works well on the default set of parameters,
/// which Spong used in his paper. In contrast, it is difficult to find a
/// functional set of gains to stabilize the robot about its upright fixed
/// point using the parameters of the physical robot we have in lab.
///

namespace drake {
namespace examples {
namespace acrobot {

using std::cout;
using std::endl;

template <typename T>
class AcrobotSpongController : public systems::LeafSystem<T> {
 public:
  AcrobotSpongController()
      : acrobot_{}, acrobot_context_(acrobot_.CreateDefaultContext()) {
    this->DeclareVectorInputPort(AcrobotState<T>());
    this->DeclareVectorOutputPort(AcrobotInput<T>(),
                                  &AcrobotSpongController::CalcControlTorque);

    // Setup context for linearization.
    acrobot_context_->FixInputPort(0, Vector1d(0));

    // Set nominal state to the upright fixed point.
    AcrobotState<T>& state =
        AcrobotPlant<T>::get_mutable_state(acrobot_context_.get());
    state.set_theta1(M_PI);
    state.set_theta2(0.0);
    state.set_theta1dot(0.0);
    state.set_theta2dot(0.0);

    auto linear_system = Linearize(acrobot_, *acrobot_context_);

    Eigen::Matrix4d Q = Eigen::Matrix4d::Identity();
    Q(0, 0) = 10;
    Q(1, 1) = 10;
    Vector1d R(1);

    systems::controllers::LinearQuadraticRegulatorResult lqr_result =
        systems::controllers::LinearQuadraticRegulator(
            linear_system->A(), linear_system->B(), Q, R);
    S_ = lqr_result.S;
    K_ = lqr_result.K;
  }

  void CalcControlTorque(const systems::Context<T>& context,
                         AcrobotInput<T>* output) const {
    acrobot_context_->get_mutable_continuous_state_vector().SetFromVector(
        this->EvalVectorInput(context, 0)->CopyToVector());
    const AcrobotState<T>& state =
        AcrobotPlant<T>::get_state(*acrobot_context_);
    const AcrobotParams<T>& p = acrobot_.get_parameters(*acrobot_context_);

    const Vector4<T> x0(M_PI, 0, 0, 0);
    Vector4<T> x = state.CopyToVector();
    // Wrap theta1 and theta2.
    x(0) = math::wrap_to(x(0), 0., 2. * M_PI);
    x(1) = math::wrap_to(x(1), -M_PI, M_PI);

    const T cost = (x - x0).dot(S_ * (x - x0));
    T u;
    if (cost < 1e3) {
      /*
       * Balancing control law
       * When the robot is close enough to the upright fixed point, i.e.
       * (x-x0)'*S*(x-x0) < some_threshold, an LQR controller linearized about
       * the upright fixed point takes over and balances the robot about its
       * upright position.
       */

      const Vector1<T> u_v = K_ * (x0 - x);
      u = u_v(0);
    } else {
      /*
       * Swing-up control law: u = u_p + u_e;
       * u_e is the energy shaping controller.
       * u_e = k_e * (E_desired - E) * q̇₂, so that
       * Edot = k_e * (E_desired - E) * q̇₂^2.
       *
       * u_p is the partial feedback linearization controller which stabilizes
       * q₂.
       * We want 
       *   q̈ = y = - k_p * q₂ - k_d * q̇₂
       * Given Acrobot's manipulator equation:
       *   q̈ =M⁻¹ * (Bu - bias),
       * where
       *   M⁻¹ = [a1,a2; a2,a3], 
       *   B = [0;1],
       *   bias=[C0;C1]
       * we have:
       *   q̈₂ = -C0*a2 + a3*(u-C1)
       * Equating the above equation to y gives
       *   u_p = (a2 * C0 + y) / a3 + C1.
       *
       * See http://underactuated.mit.edu/underactuated.html?chapter=acrobot
       */
      const Matrix2<T> M = acrobot_.MassMatrix(*acrobot_context_);
      const Vector2<T> bias = acrobot_.DynamicsBiasTerm(*acrobot_context_);
      const Matrix2<T> M_inverse = M.inverse();

      // controller gains
      const double k_e = 5;
      const double k_p = 50;
      const double k_d = 5;

      const T PE = acrobot_.EvalPotentialEnergy(*acrobot_context_);
      const T KE = acrobot_.EvalKineticEnergy(*acrobot_context_);
      const T E = PE + KE;
      const T E_desired =
          (p.m1() * p.lc1() + p.m2() * (p.l1() + p.lc2())) * p.gravity();
      const T E_tilde = E - E_desired;
      const T u_e = -k_e * E_tilde * state.theta2dot();

      const T y = -k_p * state.theta2() - k_d * state.theta2dot();
      T a3 = M_inverse(1, 1), a2 = M_inverse(0, 1);
      T u_p = (a2 * bias(0) + y) / a3 + bias(1);

      u = u_e + u_p;
    }

    // Saturation.
    const T ku_upper_bound = 20;
    const T ku_lower_bound = -20;
    if (u >= ku_upper_bound) u = ku_upper_bound;
    if (u <= ku_lower_bound) u = ku_lower_bound;

    output->set_tau(u);
  }

 private:
  AcrobotPlant<T> acrobot_;
  // The implementation above is (and must remain) careful to not store hidden
  // state in here.  This is only used to avoid runtime allocations.
  const std::unique_ptr<systems::Context<T>> acrobot_context_;
  Eigen::Matrix4d S_;
  Eigen::RowVector4d K_;
};

}  // namespace acrobot
}  // namespace examples
}  // namespace drake

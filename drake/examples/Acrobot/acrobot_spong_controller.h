// The Spong acrobot swing-up controller implemented as a LeafSystem.
// Spong, Mark W. "Swing up control of the acrobot." Robotics and Automation,
// 1994. Proceedings., 1994 IEEE International Conference on. IEEE, 1994.
// The structure of this controller follows
// PendulumEnergyShapingController in pendulum_run_energy_shaping.cc

#pragma once

#include <memory>

#include "drake/examples/Acrobot/acrobot_plant.h"
#include "drake/systems/controllers/linear_quadratic_regulator.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/primitives/linear_system.h"

namespace drake {
namespace examples {
namespace acrobot {

using std::cout;
using std::endl;

template <typename T>
class AcrobotSpongController : public systems::LeafSystem<T> {
 public:
  AcrobotSpongController()
      : acrobot_{},
        m1_{acrobot_.m1()},
        m2_{acrobot_.m2()},
        l1_{acrobot_.l1()},
        lc1_{acrobot_.lc1()},
        lc2_{acrobot_.lc2()},
        g_{acrobot_.g()} {
    this->DeclareInputPort(systems::kVectorValued,
                           acrobot_.get_output_port(0).size());
    this->DeclareVectorOutputPort(
        systems::BasicVector<T>(acrobot_.get_input_port(0).size()),
        &AcrobotSpongController::CalcControlTorque);

    // Create context for linearization.
    auto context0 = acrobot_.CreateDefaultContext();
    context0->FixInputPort(0, Vector1d(0));

    // Set nominal state to the upright fixed point.
    AcrobotStateVector<double>* x = dynamic_cast<AcrobotStateVector<double>*>(
        context0->get_mutable_continuous_state_vector());
    DRAKE_ASSERT(x != nullptr);
    x->set_theta1(M_PI);
    x->set_theta2(0.0);
    x->set_theta1dot(0.0);
    x->set_theta2dot(0.0);

    auto linear_system = Linearize(acrobot_, *context0);

    Eigen::Matrix4d Q = Eigen::Matrix4d::Identity();
    Q(0, 0) = 10;
    Q(1, 1) = 10;
    Vector1d R(1);

    systems::LinearQuadraticRegulatorResult lqr_result =
        systems::LinearQuadraticRegulator(linear_system->A(),
                                          linear_system->B(), Q, R);
    S_ = lqr_result.S;
    K_ = lqr_result.K;
  }

  void CalcControlTorque(const systems::Context<T>& context,
                         systems::BasicVector<T>* output) const {
    const AcrobotStateVector<T>* x = dynamic_cast<const AcrobotStateVector<T>*>(
        this->EvalVectorInput(context, 0));
    DRAKE_ASSERT(x != nullptr);

    const Matrix2<T> H = acrobot_.MatrixH(*x);
    const Vector2<T> C = acrobot_.VectorC(*x);
    const Matrix2<T> H_inverse = H.inverse();
    const Vector4<T> x0(M_PI, 0, 0, 0);
    Vector4<T> x_c(x->theta1(), x->theta2(), x->theta1dot(), x->theta2dot());

    /*
     * Swing-up control law: u = u_p + u_e;
     * u_e is the energy shaping controller.
     * u_e = k_e * (E_desired - E) * theta2dot, so that
     * Edot = k_e * (E_desired - E) * theta2dot^2.
     *
     * u_p is the partial feedback linearization controller which stabalizes q2.
     * We want qdotdot = y = - k_p * theta2 - k_d * theta2dot
     * Given acrobot's manipulator equation:
     * qdot = H^-1 * (Bu - C), where H = [a1,a2; a2,a3], B = [0;1], C=[C0;C1]
     * we have:
     * qdotdot = -C0*a2 + a3*(u-C1)
     * Equating the above equation to y gives
     * u_p = (a2 * C0 + y) / a3 + C1;
     *
     */

    // controller gains
    const double k_e = 5;
    const double k_p = 50;
    const double k_d = 5;

    auto context_acrobot = acrobot_.CreateDefaultContext();
    AcrobotStateVector<double>* x_acrobot =
        dynamic_cast<AcrobotStateVector<double>*>(
            context_acrobot->get_mutable_continuous_state_vector());

    x_acrobot->set_theta1(x->theta1());
    x_acrobot->set_theta2(x->theta2());
    x_acrobot->set_theta1dot(x->theta1dot());
    x_acrobot->set_theta2dot(x->theta2dot());

    const T PE = acrobot_.CalcPotentialEnergy(*context_acrobot);
    const T KE = acrobot_.CalcKineticEnergy(*context_acrobot);
    const T E = PE + KE;
    const T E_desired = (m1_ * lc1_ + m2_ * (l1_ + lc2_)) * g_;
    const T E_tilde = E - E_desired;
    const T u_e = -k_e * E_tilde * x->theta2dot();

    const T y = -k_p * x->theta2() - k_d * x->theta2dot();
    T a3 = H_inverse(1, 1), a2 = H_inverse(0, 1);
    T u_p = (a2 * C(0) + y) / a3 + C(1);

    // Wrapping of theta1 and theta2.
    while (x_c(0) > 2 * M_PI) {
      x_c(0) -= 2 * M_PI;
    }
    while (x_c(0) < 0) {
      x_c(0) += 2 * M_PI;
    }

    while (x_c(1) > M_PI) {
      x_c(1) -= 2 * M_PI;
    }
    while (x_c(1) < -M_PI) {
      x_c(1) += 2 * M_PI;
    }

    /*
     * Balancing control law
     * When the robot is close enough to the upright fixed point, i.e.
     * (x-x0)'*S*(x-x0) < some_threshold, an LQR controller linearized about
     * the upright fixed point takes over and balances the robot about its
     * upright position.
     */
    T cost = (x_c.transpose() - x0.transpose()) * S_ * (x_c - x0);
    T u;
    if (cost < 1e3) {
      Eigen::Matrix<T, 1, 1> u_v = K_ * (x0 - x_c);
      u = u_v(0, 0);
    } else {
      u = u_e + u_p;
    }

    // Saturation.
    const T ku_upper_bound = 20;
    const T ku_lower_bound = -20;
    if (u >= ku_upper_bound) u = ku_upper_bound;
    if (u <= ku_lower_bound) u = ku_lower_bound;

    output->SetAtIndex(0, u);
  }

 private:
  AcrobotPlant<T> acrobot_;
  const double m1_,  // Mass of link 1 (kg).
      m2_,           // Mass of link 2 (kg).
      l1_,           // Length of link 1 (m).
      lc1_,  // Vertical distance from shoulder joint to center of mass of
             // link 1 (m).
      lc2_,  // Vertical distance from elbox joint to center of mass of link
      g_;    // Gravitational constant (m/s^2).
  Eigen::Matrix4d S_;
  Eigen::Matrix<T, 1, 4> K_;
};

}  // namespace acrobot
}  // namespace examples
}  // namespace drake

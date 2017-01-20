// This is the same controller as acrobot_energy_shaping_controller.h, where
// the controller is contained in a drake LeafSystem. Here, the input and
// output to the controller are lcm messages instead of system ports. The
// code that computes u from xhat is copy-pasted from
// acrobot_energy_shaping_controller.h.

#include <memory>

#include <gflags/gflags.h>

#include "drake/examples/Acrobot/acrobot_plant.h"
#include "drake/examples/acrobot/acrobot_lcm.h"
#include "drake/examples/acrobot/lcmt_acrobot_u.hpp"
#include "drake/examples/acrobot/lcmt_acrobot_x.hpp"

#include "drake/examples/Acrobot/acrobot_energy_shaping_controller.h"
#include "drake/examples/Acrobot/acrobot_lcm_msg_handler.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/systems/analysis/simulator.h"

#include "drake/common/drake_path.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"

#include "drake/systems/controllers/linear_quadratic_regulator.h"
#include "drake/systems/primitives/linear_system.h"

DEFINE_double(simulation_sec, std::numeric_limits<double>::infinity(),
              "Number of seconds to simulate.");
DEFINE_double(realtime_factor, 1.0,
              "Playback speed.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");

namespace drake {
namespace examples {
namespace acrobot {
namespace {

using std::chrono::milliseconds;
using std::this_thread::sleep_for;

int DoMain() {
  // fetch acrobot parameters from AcrobotPlant
  auto acrobot = std::make_unique<AcrobotPlant<double>>();
  const double m1 = acrobot->getm1();
  const double m2 = acrobot->getm2();
  const double l1 = acrobot->getl1();
  // const double l2 = acrobot->getl2();
  const double lc1 = acrobot->getlc1();
  const double lc2 = acrobot->getlc2();
  const double Ic1 = acrobot->getIc1();
  const double Ic2 = acrobot->getIc2();
  const double b1 = acrobot->getb1();
  const double b2 = acrobot->getb2();
  const double g = acrobot->getg();

  // set up lcm communication
  drake::lcm::DrakeLcm lcm;

  const std::string channel_x = "acrobot_xhat";
  const std::string channel_u = "acrobot_u";
  lcmt_acrobot_u msg_u;
  std::vector<uint8_t> buffer_u(msg_u.getEncodedSize());

  MessageHandler handler;
  lcm.Subscribe(channel_x, &handler);
  lcm.StartReceiveThread();

  // create context for linearization
  auto context0 = acrobot->CreateDefaultContext();
  context0->FixInputPort(0, Vector1d::Constant(0.0));

  // Set nominal state to the upright fixed point.
  AcrobotStateVector<double>* x_nominal =
      dynamic_cast<AcrobotStateVector<double>*>(
          context0->get_mutable_continuous_state_vector());
  DRAKE_ASSERT(x_nominal != nullptr);
  x_nominal->set_theta1(M_PI);
  x_nominal->set_theta2(0.0);
  x_nominal->set_theta1dot(0.0);
  x_nominal->set_theta2dot(0.0);

  auto linear_system = Linearize(*acrobot, *context0);

  Eigen::Matrix4d Q = Eigen::Matrix4d::Identity();
  // Q(0, 0) = 10;
  // Q(1, 1) = 10;
  Vector1d R = Vector1d::Constant(1);

  systems::LinearQuadraticRegulatorResult lqr_result =
      systems::LinearQuadraticRegulator(linear_system->A(), linear_system->B(),
                                        Q, R);
  Eigen::Matrix4d S = lqr_result.S;
  Eigen::Matrix<double, 1, 4> K = lqr_result.K;

  cout << "S=\n" << lqr_result.S << endl;
  cout << "K=\n" << lqr_result.K << endl;

  auto x = std::make_unique<AcrobotStateVector<double>>();

  while (true) {
    double u = 0;

    if (handler.get_receive_channel() == channel_x) {
      // receive x
      const lcmt_acrobot_x received_msg = handler.GetReceivedMessage();
      /*
      std::cout << "theta1 = " << received_msg.theta1
                << " ,theta2 = " << received_msg.theta2
                << " ,theta1Dot = " << received_msg.theta1Dot
                << " ,theta2Dot = " << received_msg.theta2Dot << std::endl;
      */
      const double theta1 = received_msg.theta1;
      const double theta2 = received_msg.theta2;
      const double theta1dot = received_msg.theta1Dot;
      const double theta2dot = received_msg.theta2Dot;

      // calculate u = f(x)
      Eigen::Matrix<double, 4, 1> x_c(theta1, theta2, theta1dot, theta2dot);
      const Eigen::Matrix<double, 4, 1> x0(M_PI, 0, 0, 0);

      x->set_theta1(theta1);
      x->set_theta2(theta2);
      x->set_theta1dot(theta1dot);
      x->set_theta2dot(theta2dot);

      const double I1 = Ic1 + m1 * lc1 * lc1;
      const double I2 = Ic2 + m2 * lc2 * lc2;
      const double m2l1lc2 = m2 * l1 * lc2;  // occurs often!

      using std::sin;
      using std::cos;
      const double c1 = cos(x->theta1()), c2 = cos(x->theta2());
      const double c12 = cos(x->theta1() + x->theta2());
      const double s1 = sin(x->theta1()), s2 = sin(x->theta2());
      const double s12 = sin(x->theta1() + x->theta2());

      const double h12 = I2 + m2l1lc2 * c2;
      Eigen::Matrix<double, 2, 2> H, H_inverse;
      H << I1 + I2 + m2 * l1 * l1 + 2 * m2l1lc2 * c2, h12, h12, I2;
      H_inverse = H.inverse();
      Eigen::Matrix<double, 2, 1> qdot(x->theta1dot(), x->theta2dot());

      Eigen::Matrix<double, 2, 1> C;
      C << -2 * m2l1lc2 * s2 * x->theta2dot() * x->theta1dot() +
               -m2l1lc2 * s2 * x->theta2dot() * x->theta2dot(),
          m2l1lc2 * s2 * x->theta1dot() * x->theta1dot();

      // add in G terms
      C(0) += g * m1 * lc1 * s1 + g * m2 * (l1 * s1 + lc2 * s12);
      C(1) += g * m2 * lc2 * s12;

      // damping terms
      C(0) += b1 * x->theta1dot();
      C(1) += b2 * x->theta2dot();

      // controller gains
      const double k_e = 5;
      const double k_p = 50;
      const double k_d = 5;

      double PE, KE;
      KE = 0.5 * qdot.transpose() * H * qdot;
      PE = -m1 * g * lc1 * c1 - m2 * g * (l1 * c1 + lc2 * c12);
      const double E = KE + PE;
      const double Ed = (m1 * lc1 + m2 * (l1 + lc2)) * g;
      const double Etilde = E - Ed;
      const double u_e = -k_e * Etilde * x->theta2dot();

      const double y = -k_p * x->theta2() - k_d * x->theta2dot();
      double a3 = H_inverse(1, 1), a2 = H_inverse(0, 1);
      double u_p = (a2 * C(0) + y) / a3 + C(1);

      // wrapping of theta1 and theta2
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

      //deciding which contoller to use (LQR or energy shaping)
      double cost = (x_c.transpose() - x0.transpose()) * S * (x_c - x0);
      if (cost < 1e3) {
        Eigen::Matrix<double, 1, 1> u_v = K * (x0 - x_c);
        u = u_v(0, 0);
        // cout << "lqr ";
      } else
        u = u_e + u_p;

      // saturation
      const double ku_upper_bound = 20;
      const double ku_lower_bound = -20;

      if (u >= ku_upper_bound) u = ku_upper_bound;
      if (u <= ku_lower_bound) u = ku_lower_bound;
      /*
      cout << "u_e = " <<u_e << ", u_p= " << u_p << ", E~= " << Etilde << ", "
          "theta1 = " << x_c(0) << ", theta2=" << x_c(1)
           << " ,theta1dot = " << x_c(2) << " ,theta2dot=" << x_c(3)
           << " ,u= " << u << ", cost=" << cost <<endl;
      */
    }

    // publish u
    msg_u.tau = u;
    msg_u.encode(&buffer_u[0], 0, msg_u.getEncodedSize());
    lcm.Publish(channel_u, &buffer_u[0], msg_u.getEncodedSize());

    sleep_for(milliseconds(5));
  }

  return 0;
}
}
}  // namespace acrobot
}  // namespace examples
}  // namespace drake

int main() { return drake::examples::acrobot::DoMain(); }
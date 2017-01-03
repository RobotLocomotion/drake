#include <memory>

#include <gflags/gflags.h>

#include "drake/common/drake_path.h"
#include "drake/examples/Acrobot/acrobot_plant.h"
#include "drake/examples/Acrobot/gen/acrobot_state_vector.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"

#include "drake/systems/primitives/linear_system.h"
#include "drake/systems/controllers/linear_quadratic_regulator.h"

using std::cout;
using std::endl;

namespace drake {
namespace examples {
namespace acrobot {
namespace {


template <typename T>
class AcrobotSwingUpController : public systems::LeafSystem<T> {
 public:
  explicit AcrobotSwingUpController(const AcrobotPlant<T>& acrobot):
      m1(acrobot.getm1()),
      m2(acrobot.getm2()),
      l1(acrobot.getl1()),
      l2(acrobot.getl1()),
      lc1(acrobot.getlc1()),
      lc2(acrobot.getlc2()),
      Ic1(acrobot.getIc1()),
      Ic2(acrobot.getIc2()),
      b1(acrobot.getb1()),
      b2(acrobot.getb2()),
      g(acrobot.getg())
  {

    this->DeclareInputPort(systems::kVectorValued,
                           acrobot.get_output_port(0).get_size());
    this->DeclareOutputPort(systems::kVectorValued,
                            acrobot.get_input_port(0).get_size());

    // create context for linearization
    auto context0 = acrobot.CreateDefaultContext();
    context0->FixInputPort(0, Vector1d::Constant(0.0));

    // Set nominal state to the upright fixed point.
    AcrobotStateVector<double>* x = dynamic_cast<AcrobotStateVector<double>*>(
        context0->get_mutable_continuous_state_vector());
    DRAKE_ASSERT(x != nullptr);
    x->set_theta1(M_PI);
    x->set_theta2(0.0);
    x->set_theta1dot(0.0);
    x->set_theta2dot(0.0);

    auto linear_system = Linearize(acrobot, *context0);

    Eigen::Matrix4d Q = Eigen::Matrix4d::Identity();
    //Q(0, 0) = 10;
    //Q(1, 1) = 10;
    Vector1d R = Vector1d::Constant(1);

    systems::LinearQuadraticRegulatorResult lqr_result =
        systems::LinearQuadraticRegulator(linear_system->A(), linear_system->B()
            , Q, R);
    S = lqr_result.S;
    K = lqr_result.K;

    cout << "S=\n" << lqr_result.S << endl;
    cout << "K=\n" << lqr_result.K << endl;
  }

  void EvalOutput(const systems::Context<T>& context,
                  systems::SystemOutput<T>* output) const override {
    const AcrobotStateVector<T>* x = dynamic_cast<const
    AcrobotStateVector<T>*>(this->EvalVectorInput(context, 0));
    DRAKE_ASSERT(x != nullptr);

    Eigen::Matrix<T,4,1> x_c(x->theta1(), x->theta2(),
                            x->theta1dot(), x->theta2dot());
    const Eigen::Matrix<T,4,1> x0(M_PI, 0,0,0);
    const double I1 = Ic1 + m1 * lc1 * lc1;
    const double I2 = Ic2 + m2 * lc2 * lc2;
    const double m2l1lc2 = m2 * l1 * lc2;  // occurs often!

    using std::sin;
    using std::cos;
    const T c1 = cos(x->theta1()), c2 = cos(x->theta2());
    const T c12 = cos(x->theta1() + x->theta2());
    const T s1 = sin(x->theta1()), s2 = sin(x->theta2());
    const T s12 = sin(x->theta1() + x->theta2());

    const T h12 = I2 + m2l1lc2 * c2;
    Eigen::Matrix<T, 2, 2> H, H_inverse;
    H << I1 + I2 + m2 * l1 * l1 + 2 * m2l1lc2 * c2, h12, h12, I2;
    H_inverse = H.inverse();
    Eigen::Matrix<T, 2, 1> qdot(x->theta1dot(), x->theta2dot());

    Eigen::Matrix<T, 2, 1> C;
    C << -2 * m2l1lc2 * s2 * x->theta2dot() * x->theta1dot() +
        -m2l1lc2 * s2 * x->theta2dot() * x->theta2dot(),
        m2l1lc2 * s2 * x->theta1dot() * x->theta1dot();

    // add in G terms
    C(0) += g * m1 * lc1 * s1 + g * m2 * (l1 * s1 + lc2 * s12);
    C(1) += g * m2 * lc2 * s12;

    // damping terms
    C(0) += b1 * x->theta1dot();
    C(1) += b2 * x->theta2dot();

    T PE,KE;
    KE = 0.5*qdot.transpose()*H*qdot;
    PE = -m1*g*lc1*c1 - m2*g*(l1*c1 + lc2*c12);
    const T E = KE+PE;
    const T Ed = (m1*lc1 + m2*(l1+lc2))*g;
    const T Etilde = E-Ed;
    const T u_e = -5 * Etilde * x->theta2dot();

    const T y = -50*x->theta2() - 5 * x->theta2dot();
    T a3 = H_inverse(1,1), a2 = H_inverse(0,1);
    T u_p = (a2*C(0)+y)/a3+ C(1);

    T u;
    while(x_c(0)>2*M_PI) {x_c(0) -= 2*M_PI;}
    while(x_c(0)<0) {x_c(0) += 2*M_PI;}

    T cost = (x_c.transpose()-x0.transpose()) * S * (x_c-x0);
    if(cost < 1e3) {
      Eigen::Matrix<T,1,1> u_v = K*(x0-x_c);
      //cout << "K=\n" << K << endl;
      //cout << "x0-x_c=\n" << x0-x_c << endl;
      u = u_v(0,0);
      cout << "lqr ";
    }
    else
      u = u_e + u_p;

    cout << "u_e = " <<u_e << ", u_p= " << u_p << ", E~= " << Etilde << ", "
        "theta1 = " << x_c(0) << ", theta2=" << x_c(1)
         << " ,theta1dot = " << x_c(2) << " ,theta2dot=" << x_c(3)
         << " ,u= " << u << ", cost=" << cost <<endl;

    output->GetMutableVectorData(0)->SetAtIndex(0, u);
  }

 private:
  const double m1,  // Mass of link 1 (kg).
      m2,           // Mass of link 2 (kg).
      l1,           // Length of link 1 (m).
      l2,           // Length of link 2 (m).
      lc1,  // Vertical distance from shoulder joint to center of mass of
            // link 1 (m).
      lc2,  // Vertical distance from elbox joint to center of mass of link
            // 2 (m).
      Ic1,  // Inertia of link 1 about the center of mass of link 1
            // (kg*m^2).
      Ic2,   // Inertia of link 2 about the center of mass of link 2
            // (kg*m^2).
      b1,    // Damping coefficient of the shoulder joint (kg*m^2/s).
      b2,    // Damping coefficient of the elbow joint (kg*m^2/s).
      g;    // Gravitational constant (m/s^2).
  Eigen::MatrixXd S;
  Eigen::MatrixXd K;
};



// Simple example which simulates the Acrobot, started near the upright, with an
// LQR controller designed to stabilize the unstable fixed point.  Run
// drake-visualizer to see the animated result.

DEFINE_double(realtime_factor, 1.0,
              "Playback speed.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");

int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  lcm::DrakeLcm lcm;
  auto tree = std::make_unique<RigidBodyTree<double>>();
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      GetDrakePath() + "/examples/Acrobot/Acrobot.urdf",
      multibody::joints::kFixed, tree.get());

  systems::DiagramBuilder<double> builder;
  auto acrobot = builder.AddSystem<AcrobotPlant>();
  auto publisher = builder.AddSystem<systems::DrakeVisualizer>(*tree, &lcm);
  builder.Connect(acrobot->get_output_port(0), publisher->get_input_port(0));
  cout << "Acrobot input ports: " << acrobot->get_input_port(0).get_size() << endl;
  cout << "Acrobot output ports: " << acrobot->get_output_port(0).get_size() <<
                                                                         endl;


  auto controller = builder.AddSystem<AcrobotSwingUpController>(*acrobot);
  builder.Connect(acrobot->get_output_port(0), controller->get_input_port(0));
  builder.Connect(controller->get_output_port(0), acrobot->get_input_port(0));

  //auto controller = builder.AddSystem(BalancingLQRController(acrobot));
  //builder.Connect(acrobot->get_output_port(0), controller->get_input_port());
  //builder.Connect(controller->get_output_port(), acrobot->get_input_port(0));


  auto diagram = builder.Build();
  systems::Simulator<double> simulator(*diagram);
  systems::Context<double>* acrobot_context =
      diagram->GetMutableSubsystemContext(simulator.get_mutable_context(),
                                          acrobot);

  // Set an initial condition near the upright fixed point.
  AcrobotStateVector<double>* x0 = dynamic_cast<AcrobotStateVector<double>*>(
      acrobot_context->get_mutable_continuous_state_vector());
  DRAKE_DEMAND(x0 != nullptr);
  x0->set_theta1(0.1);
  x0->set_theta2(-0.1);
  x0->set_theta1dot(0.0);
  x0->set_theta2dot(0.02);

  //--------------------------------------------------------------------------
  // create context for linearization
  auto context0 = acrobot->CreateDefaultContext();
  context0->FixInputPort(0, Vector1d::Constant(0.0));

  // Set nominal state to the upright fixed point.
  AcrobotStateVector<double>* x = dynamic_cast<AcrobotStateVector<double>*>(
      context0->get_mutable_continuous_state_vector());
  DRAKE_ASSERT(x != nullptr);
  x->set_theta1(M_PI);
  x->set_theta2(0.0);
  x->set_theta1dot(0.0);
  x->set_theta2dot(0.0);

  auto linear_system = Linearize(*acrobot, *context0);

  cout << linear_system->A() << endl;
  cout << linear_system->B() << endl;
  cout << acrobot->EvalEnergy(*context0) << endl;

  Eigen::Matrix4d Q = Eigen::Matrix4d::Identity();
  //Q(0, 0) = 10;
  //Q(1, 1) = 10;
  Vector1d R = Vector1d::Constant(1);

  systems::LinearQuadraticRegulatorResult lqr_result =
      systems::LinearQuadraticRegulator(linear_system->A(), linear_system->B()
          , Q, R);

  cout << "S=\n" << lqr_result.S << endl;
  cout << "K=\n" << lqr_result.K << endl;
  //--------------------------------------------------------------------------


  simulator.set_target_realtime_rate(FLAGS_realtime_factor);
  simulator.Initialize();
  simulator.StepTo(60);


  return 0;
}

}  // namespace
}  // namespace acrobot
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  return drake::examples::acrobot::do_main(argc, argv);
}

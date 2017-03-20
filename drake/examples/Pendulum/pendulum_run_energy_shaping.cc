#include <cmath>
#include <memory>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_path.h"
#include "drake/examples/Pendulum/pendulum_plant.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace pendulum {
namespace {

template <typename T>
class PendulumEnergyShapingController : public systems::LeafSystem<T> {
 public:
  explicit PendulumEnergyShapingController(const PendulumPlant<T>& pendulum)
      : m_(pendulum.m()),
        l_(pendulum.l()),
        b_(pendulum.b()),
        g_(pendulum.g()) {
    this->DeclareInputPort(systems::kVectorValued,
                           pendulum.get_output_port().size());
    this->DeclareOutputPort(systems::kVectorValued,
                            pendulum.get_tau_port().size());
  }

 private:
  void DoCalcOutput(const systems::Context<T>& context,
                    systems::SystemOutput<T>* output) const override {
    const PendulumStateVector<T>* const state =
        dynamic_cast<const PendulumStateVector<T>*>(
            this->EvalVectorInput(context, 0));
    DRAKE_ASSERT(state != nullptr);
    // Pendulum energy shaping formula from Section 3.5.2 of Russ
    // Tedrake. Underactuated Robotics: Algorithms for Walking, Running,
    // Swimming, Flying, and Manipulation (Course Notes for MIT
    // 6.832). Downloaded on 2016-09-30 from
    // http://underactuated.csail.mit.edu/underactuated.html?chapter=3
    const T Etilde = .5 * m_ * l_ * l_ * state->thetadot() * state->thetadot() -
        m_ * g_ * l_ * cos(state->theta()) - 1.1 * m_ * g_ * l_;
    const T tau = b_ * state->thetadot() - .1 * state->thetadot() * Etilde;
    output->GetMutableVectorData(0)->SetAtIndex(0, tau);
  }

  const T m_;
  const T l_;
  const T b_;
  const T g_;
};

int do_main(int argc, char* argv[]) {
  lcm::DrakeLcm lcm;
  auto tree = std::make_unique<RigidBodyTree<double>>();
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      GetDrakePath() + "/examples/Pendulum/Pendulum.urdf",
      multibody::joints::kFixed, tree.get());

  systems::DiagramBuilder<double> builder;
  auto pendulum = builder.AddSystem<PendulumPlant>();
  auto controller =
      builder.AddSystem<PendulumEnergyShapingController>(*pendulum);
  builder.Connect(pendulum->get_output_port(), controller->get_input_port(0));
  builder.Connect(controller->get_output_port(0), pendulum->get_tau_port());

  auto publisher =
      builder.AddSystem<systems::DrakeVisualizer>(*tree, &lcm);
  builder.Connect(pendulum->get_output_port(), publisher->get_input_port(0));

  auto diagram = builder.Build();
  systems::Simulator<double> simulator(*diagram);
  systems::Context<double>* pendulum_context =
      diagram->GetMutableSubsystemContext(
          simulator.get_mutable_context(), pendulum);
  pendulum->set_theta(pendulum_context, 0.1);
  pendulum->set_thetadot(pendulum_context, 0.2);

  simulator.Initialize();
  simulator.StepTo(10);
  return 0;
}

}  // namespace
}  // namespace pendulum
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  return drake::examples::pendulum::do_main(argc, argv);
}

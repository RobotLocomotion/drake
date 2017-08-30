#include <cmath>
#include <memory>

#include <gflags/gflags.h>

#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/examples/pendulum/pendulum_plant.h"
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

DEFINE_double(target_realtime_rate, 1.0,
              "Playback speed.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");

template <typename T>
class PendulumEnergyShapingController : public systems::LeafSystem<T> {
 public:
  explicit PendulumEnergyShapingController(const PendulumParams<T>& params) {
    this->DeclareVectorInputPort(PendulumState<T>());
    this->DeclareVectorOutputPort(PendulumInput<T>(),
                                  &PendulumEnergyShapingController::CalcTau);
    this->DeclareNumericParameter(params);
  }

 private:
  void CalcTau(const systems::Context<T>& context,
               PendulumInput<T>* output) const {
    const PendulumState<T>* const state = dynamic_cast<const PendulumState<T>*>(
        this->EvalVectorInput(context, 0));
    DRAKE_ASSERT(state != nullptr);
    const PendulumParams<T>& params =
        this->template GetNumericParameter<PendulumParams>(context, 0);

    // Pendulum energy shaping formula from Section 3.5.2 of Russ
    // Tedrake. Underactuated Robotics: Algorithms for Walking, Running,
    // Swimming, Flying, and Manipulation (Course Notes for MIT
    // 6.832). Downloaded on 2016-09-30 from
    // http://underactuated.csail.mit.edu/underactuated.html?chapter=3
    using std::pow;
    const T Etilde = .5 * params.mass() * pow(params.length(), 2) *
                         state->thetadot() * state->thetadot() -
                     params.mass() * params.gravity() * params.length() *
                         cos(state->theta()) -
                     1.1 * params.mass() * params.gravity() * params.length();
    output->set_tau(params.damping() * state->thetadot() -
                    .1 * state->thetadot() * Etilde);
  }
};

int do_main() {
  lcm::DrakeLcm lcm;
  auto tree = std::make_unique<RigidBodyTree<double>>();
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      FindResourceOrThrow("drake/examples/pendulum/Pendulum.urdf"),
      multibody::joints::kFixed, tree.get());

  systems::DiagramBuilder<double> builder;
  auto pendulum = builder.AddSystem<PendulumPlant>();
  pendulum->set_name("pendulum");
  auto controller = builder.AddSystem<PendulumEnergyShapingController>(
      PendulumParams<double>());
  controller->set_name("controller");
  builder.Connect(pendulum->get_output_port(), controller->get_input_port(0));
  builder.Connect(controller->get_output_port(0), pendulum->get_tau_port());

  auto publisher = builder.AddSystem<systems::DrakeVisualizer>(*tree, &lcm);
  publisher->set_name("publisher");
  builder.Connect(pendulum->get_output_port(), publisher->get_input_port(0));

  auto diagram = builder.Build();
  systems::Simulator<double> simulator(*diagram);
  systems::Context<double>& pendulum_context =
      diagram->GetMutableSubsystemContext(*pendulum,
                                          simulator.get_mutable_context());
  pendulum->set_theta(&pendulum_context, M_PI);
  pendulum->set_thetadot(&pendulum_context, 0.0);

  const double desired_energy = pendulum->CalcTotalEnergy(pendulum_context);

  pendulum->set_theta(&pendulum_context, 0.1);
  pendulum->set_thetadot(&pendulum_context, 0.2);

  const double initial_energy = pendulum->CalcTotalEnergy(pendulum_context);

  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.Initialize();
  simulator.StepTo(10);

  const double final_energy = pendulum->CalcTotalEnergy(pendulum_context);

  // Adds a numerical test that we have successfully regulated the energy
  // towards the desired energy level.
  DRAKE_DEMAND(std::abs(initial_energy - desired_energy) >
               2.0 * std::abs(final_energy - desired_energy));

  return 0;
}

}  // namespace
}  // namespace pendulum
}  // namespace examples
}  // namespace drake

int main() { return drake::examples::pendulum::do_main(); }

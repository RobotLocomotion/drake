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
    const auto* state =
        this->template EvalVectorInput<PendulumState>(context, 0);
    const PendulumParams<T>& params =
        this->template GetNumericParameter<PendulumParams>(context, 0);

    // Pendulum energy shaping from Section 3.5.2 of
    // http://underactuated.csail.mit.edu/underactuated.html?chapter=3
    using std::pow;
    // Desired energy is slightly more than the energy at the top (want to pass
    // through the upright with non-zero velocity).
    const T desired_energy =
        1.1 * params.mass() * params.gravity() * params.length();
    // Current total energy (see PendulumPlant::CalcTotalEnergy).
    const T current_energy =
        0.5 * params.mass() * pow(params.length() * state->thetadot(), 2) -
        params.mass() * params.gravity() * params.length() *
            cos(state->theta());
    const double kEnergyFeedbackGain = .1;
    output->set_tau(params.damping() * state->thetadot() +
                    kEnergyFeedbackGain * state->thetadot() *
                        (desired_energy - current_energy));
  }
};

int DoMain() {
  lcm::DrakeLcm lcm;
  auto tree = std::make_unique<RigidBodyTree<double>>();
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      FindResourceOrThrow("drake/examples/pendulum/Pendulum.urdf"),
      multibody::joints::kFixed, tree.get());

  systems::DiagramBuilder<double> builder;
  auto pendulum = builder.AddSystem<PendulumPlant>();
  pendulum->set_name("pendulum");
  // Use default pendulum parameters in the controller (real controllers never
  // get the true parameters).
  auto controller = builder.AddSystem<PendulumEnergyShapingController>(
      PendulumParams<double>());
  controller->set_name("controller");
  builder.Connect(pendulum->get_output_port(), controller->get_input_port(0));
  builder.Connect(controller->get_output_port(0), pendulum->get_input_port());

  auto publisher = builder.AddSystem<systems::DrakeVisualizer>(*tree, &lcm);
  publisher->set_name("publisher");
  builder.Connect(pendulum->get_output_port(), publisher->get_input_port(0));

  auto diagram = builder.Build();
  systems::Simulator<double> simulator(*diagram);
  systems::Context<double>& pendulum_context =
      diagram->GetMutableSubsystemContext(*pendulum,
                                          &simulator.get_mutable_context());
  auto& state = pendulum->get_mutable_state(&pendulum_context);

  // Desired energy is the total energy when the pendulum is at the upright.
  state.set_theta(M_PI);
  state.set_thetadot(0.0);
  const double desired_energy = pendulum->CalcTotalEnergy(pendulum_context);

  // Set initial conditions (near the bottom).
  state.set_theta(0.1);
  state.set_thetadot(0.2);
  const double initial_energy = pendulum->CalcTotalEnergy(pendulum_context);

  // Run the simulation.
  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.Initialize();
  simulator.StepTo(10);

  // Compute the final (total) energy.
  const double final_energy = pendulum->CalcTotalEnergy(pendulum_context);

  // Adds a numerical test that we have successfully regulated the energy
  // towards the desired energy level.
  DRAKE_DEMAND(std::abs(final_energy - desired_energy) <
               (0.5 * std::abs(initial_energy - desired_energy)));

  return 0;
}

}  // namespace
}  // namespace pendulum
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::pendulum::DoMain();
}

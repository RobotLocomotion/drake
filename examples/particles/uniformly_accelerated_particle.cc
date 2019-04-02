///
/// @brief  A simple 1DOF, constantly accelerated particle example.
///

#include <cstdlib>
#include <limits>
#include <memory>

#include "drake/common/find_resource.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/examples/particles/particle.h"
#include "drake/examples/particles/utilities.h"
#include "drake/multibody/parsers/sdf_parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/primitives/constant_vector_source.h"

DEFINE_double(initial_position, 0.0,
              "Particle initial x position");
DEFINE_double(initial_velocity, 0.0,
              "Particle initial x velocity");
DEFINE_double(acceleration, 1.0,
              "Particle constant x acceleration");
DEFINE_double(realtime_rate, 1.0,
              "Rate at which to run the simulation, relative to realtime");
DEFINE_double(simulation_time, std::numeric_limits<double>::infinity(),
              "How long to simulate the particle");

namespace drake {
namespace examples {
namespace particles {
namespace {

/// Fixed path to particle SDF model (for visualization purposes only).
static const char* const kParticleSdfPath =
  "drake/examples/particles/models/particle.sdf";

/// A sample diagram for visualizing a 1DOF particle to which a
/// a constant acceleration is applied.
class UniformlyAcceleratedParticle : public systems::Diagram<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(UniformlyAcceleratedParticle)

  /// A constructor that wires up the whole diagram,
  /// taking a constant acceleration to be applied to
  /// the particle.
  ///
  /// @param[in] acceleration in m/s^2 units.
  explicit UniformlyAcceleratedParticle(double acceleration);

  /// Creates a context using AllocateContext() and sets
  /// state variables according to the initial conditions supplied.
  ///
  /// @param[in] position in m units.
  /// @param[in] velocity in m/s units.
  /// @return a newly created Context.
  std::unique_ptr<systems::Context<double>>
  CreateContext(double position, double velocity) const;

 private:
  /// RigidBodyTree particle representation
  /// (for visualizations purposes only).
  std::unique_ptr<RigidBodyTree<double>> tree_{
    std::make_unique<RigidBodyTree<double>>()};
};

UniformlyAcceleratedParticle::UniformlyAcceleratedParticle(
    const double acceleration) {
  // Parse particle sdf into rigid body tree.
  parsers::sdf::AddModelInstancesFromSdfFileToWorld(
      FindResourceOrThrow(kParticleSdfPath),
      multibody::joints::kRollPitchYaw,
      tree_.get());
  // Compile tree one more time just to be sure.
  tree_->compile();
  // Building diagram.
  systems::DiagramBuilder<double> builder;
  // Adding constant acceleration source.
  auto constant_acceleration_vector_source =
    builder.AddSystem<systems::ConstantVectorSource<double>>(acceleration);
  // Adding particle.
  auto particle = builder.AddSystem<Particle<double>>();
  // Adding particle joint.
  MatrixX<double> translating_matrix(6, 1);
  // Only first generalized coordinate gets through.
  translating_matrix.setZero();
  translating_matrix(0, 0) = 1.0;
  auto particle_joint =
      builder.AddSystem(MakeDegenerateEulerJoint(translating_matrix));
  // Adding visualizer client.
  auto lcm = builder.AddSystem<systems::lcm::LcmInterfaceSystem>();
  auto visualizer = builder.AddSystem<systems::DrakeVisualizer>(*tree_, lcm);
  // Wiring all blocks together.
  builder.Connect(*constant_acceleration_vector_source, *particle);
  builder.Connect(*particle, *particle_joint);
  builder.Connect(*particle_joint, *visualizer);
  builder.BuildInto(this);
}

std::unique_ptr<systems::Context<double>>
UniformlyAcceleratedParticle::CreateContext(
    const double position, const double velocity) const {
  // Allocate context.
  auto context = this->AllocateContext();
  // Set continuous state.
  systems::VectorBase<double>& cstate =
    context->get_mutable_continuous_state_vector();
  cstate.SetAtIndex(0, position);
  cstate.SetAtIndex(1, velocity);
  return context;
}

///
/// Main function for demo.
///
int main(int argc, char* argv[]) {
  gflags::SetUsageMessage("A very simple demonstration, "
                          "make sure drake-visualizer is running!");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  logging::HandleSpdlogGflags();
  // Instantiate example system.
  auto system = std::make_unique<UniformlyAcceleratedParticle>(
      FLAGS_acceleration);
  // Get context with initial conditions.
  auto context = system->CreateContext(
      FLAGS_initial_position, FLAGS_initial_velocity);
  // Instantiate and configure simulator.
  auto simulator = std::make_unique<systems::Simulator<double>>(
      *system, std::move(context));
  simulator->set_target_realtime_rate(FLAGS_realtime_rate);
  simulator->Initialize();
  // Run simulation.
  simulator->AdvanceTo(FLAGS_simulation_time);
  return 0;
}

}  // namespace
}  // namespace particles
}  // namespace examples
}  // namespace drake

int main(int argc, char **argv) {
  return drake::examples::particles::main(argc, argv);
}

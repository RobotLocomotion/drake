#include <memory>

#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/common/proto/call_python.h"
#include "drake/examples/rimless_wheel/rimless_wheel.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace examples {
namespace rimless_wheel {
namespace {

DEFINE_double(initial_angular_velocity, 5.0,
              "Initial condition of the wheel are upright with this specified "
              "initial angular velocity.");
DEFINE_double(target_realtime_rate, 1.0,
              "Playback speed.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");

/// Simulates the rimless wheel from various initial velocities (accepted as
/// command-line arguments.  Run drake-visualizer to watch the results.
int DoMain() {
  systems::DiagramBuilder<double> builder;
  auto rimless_wheel = builder.AddSystem<RimlessWheel>();
  rimless_wheel->set_name("rimless_wheel");

  lcm::DrakeLcm lcm;
  auto tree = std::make_unique<RigidBodyTree<double>>();
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      FindResourceOrThrow("drake/examples/rimless_wheel/RimlessWheel.urdf"),
      multibody::joints::kRollPitchYaw, tree.get());

  double ramp_pitch = RimlessWheelParams<double>().slope();
  {  // Add ramp
    // TODO(russt): Consider moving/reusing this block (useful for all passive
    // walkers).
    DrakeShapes::Box geom(Eigen::Vector3d(100, 1, 10));
    Eigen::Isometry3d T_element_to_link = Eigen::Isometry3d::Identity();
    T_element_to_link.translation() << 0, 0,
        -10. / 2;  // Top of the box is at z = 0.
    T_element_to_link.rotate(
        math::RotationMatrix<double>::MakeYRotation(ramp_pitch).matrix());

    // Defines a color called "desert sand" according to htmlcsscolor.com.
    Eigen::Vector4d color;
    color << 0.9297, 0.7930, 0.6758, 1;

    RigidBody<double>& world = tree->world();
    world.AddVisualElement(
        DrakeShapes::VisualElement(geom, T_element_to_link, color));
    tree->addCollisionElement(
        drake::multibody::collision::Element(geom, T_element_to_link, &world),
        world, "terrain");
    tree->compile();
  }
  auto publisher = builder.AddSystem<systems::DrakeVisualizer>(*tree, &lcm);
  publisher->set_name("publisher");

  builder.Connect(rimless_wheel->get_output_port(1),
                  publisher->get_input_port(0));
  auto diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);
  systems::Context<double>& rw_context = diagram->GetMutableSubsystemContext(
      *rimless_wheel, &simulator.get_mutable_context());
  RimlessWheelState<double>& state =
      rimless_wheel->get_mutable_state(&rw_context);
  state.set_theta(0.0);
  state.set_thetadot(FLAGS_initial_angular_velocity);

  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.get_mutable_context().set_accuracy(1e-8);
  simulator.StepTo(10);

  // Check that the state is still inside the expected region (I did not miss
  // any collisions).
  // Note:  This currently fails for initial_angular_velocity = -5.0 (in a way
  // that poses a sort of fundamental challenge to our zero-crossing accuracy
  // checks).
  const RimlessWheelParams<double>& params =
      rimless_wheel->get_parameters(rw_context);
  const double alpha = M_PI / params.number_of_spokes();
  DRAKE_DEMAND(state.theta() >= params.slope() - alpha);
  DRAKE_DEMAND(state.theta() <= params.slope() + alpha);

  return 0;
}

}  // namespace
}  // namespace rimless_wheel
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::rimless_wheel::DoMain();
}

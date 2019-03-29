#include <memory>

#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
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

DEFINE_double(accuracy, 1e-4, "Accuracy of the rimless wheel system (unitless);"
    " must be positive.");
DEFINE_double(initial_angle, 0.0, "Initial angle of the wheel (rad).  Must be"
    " in the interval (slope - alpha, slope + alpha), as described in "
    "http://underactuated.mit.edu/underactuated.html?chapter=simple_legs .");
DEFINE_double(initial_angular_velocity, 5.0,
              "Initial angular velocity of the wheel (rad/sec).");
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

    // In the following use W for world frame and B for box frame.
    Eigen::Isometry3d X_WB = Eigen::Isometry3d::Identity();
    X_WB.translation() << 0, 0, -10. / 2;  // Top of the box is at z = 0.
    X_WB.rotate(
        math::RotationMatrix<double>::MakeYRotation(ramp_pitch).matrix());

    // Defines a color called "desert sand" according to htmlcsscolor.com.
    Eigen::Vector4d color;
    color << 0.9297, 0.7930, 0.6758, 1;

    RigidBody<double>& world = tree->world();
    world.AddVisualElement(DrakeShapes::VisualElement(geom, X_WB, color));
    tree->addCollisionElement(
        drake::multibody::collision::Element(geom, X_WB, &world), world,
        "terrain");
    tree->compile();
  }
  auto publisher = builder.AddSystem<systems::DrakeVisualizer>(*tree, &lcm);
  publisher->set_name("publisher");

  builder.Connect(rimless_wheel->get_floating_base_state_output_port(),
                  publisher->get_input_port(0));
  auto diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);
  systems::Context<double>& rw_context = diagram->GetMutableSubsystemContext(
      *rimless_wheel, &simulator.get_mutable_context());
  RimlessWheelContinuousState<double>& state =
      rimless_wheel->get_mutable_continuous_state(&rw_context);

  // Check that command line argument puts the wheel above the ground.
  const RimlessWheelParams<double>& params =
      rimless_wheel->get_parameters(rw_context);
  const double alpha = rimless_wheel->calc_alpha(params);
  DRAKE_DEMAND(FLAGS_initial_angle > params.slope() - alpha);
  DRAKE_DEMAND(FLAGS_initial_angle < params.slope() + alpha);

  state.set_theta(FLAGS_initial_angle);
  state.set_thetadot(FLAGS_initial_angular_velocity);

  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.get_mutable_context().SetAccuracy(FLAGS_accuracy);
  simulator.AdvanceTo(10);

  // Check that the state is still inside the expected region (I did not miss
  // any collisions).
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

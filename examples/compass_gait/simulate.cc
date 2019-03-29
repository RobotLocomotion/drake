#include <memory>

#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/examples/compass_gait/compass_gait.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace examples {
namespace compass_gait {
namespace {

DEFINE_double(target_realtime_rate, 1.0,
              "Playback speed.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");

/// Simulates the compass gait from various initial velocities (accepted as
/// command-line arguments.  Run drake-visualizer to watch the results.
int DoMain() {
  systems::DiagramBuilder<double> builder;
  auto compass_gait = builder.AddSystem<CompassGait>();
  compass_gait->set_name("compass_gait");

  auto tree = std::make_unique<RigidBodyTree<double>>();
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      FindResourceOrThrow("drake/examples/compass_gait/CompassGait.urdf"),
      multibody::joints::kRollPitchYaw, tree.get());

  {  // Add ramp
    // TODO(russt): Consider moving/reusing this block (useful for all passive
    // walkers).
    const double box_depth = 10.0;
    DrakeShapes::Box geom(Eigen::Vector3d(100, 1, box_depth));

    // In the following use W for world frame and B for box frame.
    Eigen::Isometry3d X_WB = Eigen::Isometry3d::Identity();
    X_WB.translation() << 0, 0, -box_depth / 2;  // Top of the box is at z = 0.
    double ramp_pitch = CompassGaitParams<double>().slope();
    X_WB.rotate(
        math::RotationMatrix<double>::MakeYRotation(ramp_pitch).matrix());

    // Defines a color called "desert sand" according to htmlcsscolor.com.
    Eigen::Vector4d color(0.9297, 0.7930, 0.6758, 1);

    RigidBody<double>& world = tree->world();
    world.AddVisualElement(DrakeShapes::VisualElement(geom, X_WB, color));
    tree->addCollisionElement(
        drake::multibody::collision::Element(geom, X_WB, &world), world,
        "terrain");
    tree->compile();
  }

  lcm::DrakeLcm lcm;
  auto publisher = builder.AddSystem<systems::DrakeVisualizer>(*tree, &lcm);
  publisher->set_name("publisher");

  builder.Connect(compass_gait->get_floating_base_state_output_port(),
                  publisher->get_input_port(0));
  auto diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);
  systems::Context<double>& rw_context = diagram->GetMutableSubsystemContext(
      *compass_gait, &simulator.get_mutable_context());
  CompassGaitContinuousState<double>& state =
      compass_gait->get_mutable_continuous_state(&rw_context);
  state.set_stance(0.0);
  state.set_swing(0.0);
  state.set_stancedot(0.4);
  state.set_swingdot(-2.0);

  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.get_mutable_context().SetAccuracy(1e-4);
  simulator.AdvanceTo(10);

  return 0;
}

}  // namespace
}  // namespace compass_gait
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::compass_gait::DoMain();
}

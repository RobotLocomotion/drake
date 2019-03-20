#include <memory>

#include <gflags/gflags.h>

#include "drake/common/text_logging_gflags.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/geometry/scene_graph.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/benchmarks/inclined_plane/inclined_plane_plant.h"
#include "drake/multibody/plant/contact_results_to_lcm.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace multibody {
namespace examples {
namespace inclined_plane_with_body {
namespace {

// This simulates the motion of a rigid body B (e.g., a sphere or a block) on an
// inclined-plane A (which may be an infinite half-space or a finite box).
//
// To visualize this example, open a terminal, change to the drake directory and
// type something like the following at the operating system command prompt.
// ./bazel-bin/tools/drake_visualizer &
//
// Open another terminal, change to the drake directory, and compile/run this
// example with its default parameters by typing (all on one command line)
// bazel run
// examples/multibody/inclined_plane_with_body:inclined_plane_with_body
//
// To simulate body B as a block that has 4 contacting spheres welded to its
// lowest four corners on an inclined-plane A (modeled as a half-space), pass
// command line arguments to the executable by typing (all on one command line)
// bazel run
// examples/multibody/inclined_plane_with_body:inclined_plane_with_body --
// --target_realtime_rate=0.5 --simulation_time=2.8 --time_step=1.0E-4
// --slope_degrees=30 --is_inclined_plane_half_space=true
// --penetration_allowance=1.0E-4 --muS_bodyB=0.1 --muK_bodyB=0.1
// --is_bodyB_sphere=false --is_bodyB_block_with_4Spheres=true
DEFINE_double(target_realtime_rate, 1.0,
              "Desired rate relative to real time.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");
DEFINE_double(simulation_time, 2.0, "Simulation duration in seconds");
DEFINE_double(time_step, 1.0E-5,
              "If zero, the plant is modeled as a continuous system. "
              "If positive, the period (in seconds) of the discrete updates "
              "for the plant modeled as a discrete system."
              "This parameter must be non-negative.");
DEFINE_double(penetration_allowance, 1.E-5, "Contact penetration allowance.");
DEFINE_double(slope_degrees, 15.0, "Inclined-plane angle in degrees.");
DEFINE_double(muS_inclined_plane, 0.3, "Inclined-plane static friction coef.");
DEFINE_double(muK_inclined_plane, 0.3, "Inclined-plane kinetic friction coef.");
DEFINE_double(muS_bodyB, 0.3, "Body B's static friction coefficient.");
DEFINE_double(muK_bodyB, 0.3, "Body B's kinetic friction coefficient.");
DEFINE_bool(is_inclined_plane_half_space, true,
            "Is inclined-plane a half-space (true) or box (false).");
DEFINE_bool(is_bodyB_sphere, false,
            "Is body B a sphere (true) or block (false).");
DEFINE_bool(is_bodyB_block_with_4Spheres, true,
            "If body B is a block, is it contacting surface modeled with "
            "4 spheres (true) or a single box (false).");

using drake::multibody::MultibodyPlant;

int do_main() {
  // Build a generic multibody plant.
  systems::DiagramBuilder<double> builder;
  auto pair = AddMultibodyPlantSceneGraph(
      &builder, std::make_unique<MultibodyPlant<double>>(FLAGS_time_step));
  MultibodyPlant<double> &plant = pair.plant;

  // Set constants that are relevant whether body B is a sphere or block.
  const double massB = 0.1;       // Body B's mass (kg).
  const double gravity = 9.8;     // Earth's gravitational acceleration (m/s^2).
  const double slope_radians = FLAGS_slope_degrees / 180 * M_PI;
  const drake::multibody::CoulombFriction<double>
      coefficient_friction_bodyB(FLAGS_muS_bodyB,
                                 FLAGS_muK_bodyB);
  const drake::multibody::CoulombFriction<double>
      coefficient_friction_inclined_plane(FLAGS_muS_inclined_plane,
                                          FLAGS_muK_inclined_plane);

  if (FLAGS_is_bodyB_sphere) {
    const double radiusB = 0.04;      // B's radius when modeled as a sphere.
    const double LAx = 20 * radiusB;  // Inclined-plane length in Ax direction.
    const double LAy = 10 * radiusB;  // Inclined-plane length in Ay direction.
    const double LAz = radiusB;       // Inclined-plane length in Az direction.
    benchmarks::inclined_plane::AddInclinedPlaneWithSpherePlant(
        gravity, slope_radians,
        FLAGS_is_inclined_plane_half_space, LAx, LAy, LAz,
        radiusB, massB,
        coefficient_friction_inclined_plane, coefficient_friction_bodyB,
        &plant);
  } else {
    const double LBx = 0.4;      // Block B's length in Bx-direction (meters).
    const double LBy = 0.2;      // Block B's length in By-direction (meters).
    const double LBz = 0.04;     // Block B's length in Bz-direction (meters).
    const double LAx = 8 * LBx;  // Inclined-plane A's length in Ax direction.
    const double LAy = 8 * LBy;  // Inclined-plane A's length in Ay direction.
    const double LAz = 0.04;     // Inclined-plane A's length in Az direction.
    benchmarks::inclined_plane::AddInclinedPlaneWithBlockPlant(
        gravity, slope_radians,
        FLAGS_is_inclined_plane_half_space, LAx, LAy, LAz,
        LBx, LBy, LBz, massB,
        coefficient_friction_inclined_plane, coefficient_friction_bodyB,
        FLAGS_is_bodyB_block_with_4Spheres, &plant);
  }

  plant.Finalize();

  // Set allowable penetration (in meters) for body B to inclined-plane.
  plant.set_penetration_allowance(FLAGS_penetration_allowance);

  // Set the stiction tolerance for the underlying Stribeck friction model.
  plant.set_stiction_tolerance(1.0E-5);

  // Do a reality check that body B is a free-flying rigid body.
  DRAKE_DEMAND(plant.num_velocities() == 6);
  DRAKE_DEMAND(plant.num_positions() == 7);

  // Publish contact results for visualization.
  ConnectContactResultsToDrakeVisualizer(&builder, plant);

  geometry::ConnectDrakeVisualizer(&builder, pair.scene_graph);
  auto diagram = builder.Build();

  // Create a context for this system:
  std::unique_ptr<systems::Context<double>> diagram_context =
      diagram->CreateDefaultContext();
  diagram->SetDefaultContext(diagram_context.get());
  systems::Context<double> &plant_context =
      diagram->GetMutableSubsystemContext(plant, diagram_context.get());
  plant.SetDefaultContext(&plant_context);

  // The default initial configuration and motion of body B in World W are:
  // R_WB (B's initial rotation matrix in W) is 3x3 identity matrix,
  // p_Wo_Bcm [position from Wo (World origin) to Bcm (B's center of mass)] and
  // p_Wo_Bo  [position from Wo to Bo (B's origin)] is zero vector [0; 0; 0],
  // B is stationary in world (B's velocity and angular velocity in W are zero).
  // Set body B's initial position so it is above the inclined plane.
  const drake::multibody::Body<double> &bodyB = plant.GetBodyByName("BodyB");
  const Vector3<double> p_WoBo_W(-1.0, 0, 1.2);
  const math::RigidTransform<double> X_WB(p_WoBo_W);
  plant.SetFreeBodyPoseInWorldFrame(&plant_context, bodyB,
                                    X_WB.GetAsIsometry3());

  systems::Simulator<double> simulator(*diagram, std::move(diagram_context));
  systems::IntegratorBase<double> *integrator =
      simulator.get_mutable_integrator();

  // Set the integration accuracy when the plant is integrated with a variable-
  // step integrator. This value is not used if time_step > 0 (fixed-time step).
  const double integration_accuracy = 1.0E-6;
  integrator->set_target_accuracy(integration_accuracy);

  simulator.set_publish_every_time_step(false);
  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.Initialize();
  simulator.StepTo(FLAGS_simulation_time);

  return 0;
}

}  // namespace
}  // namespace inclined_plane_with_body
}  // namespace examples
}  // namespace multibody
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage(
      "Simulation of a body (sphere or block) on an inclined-plane that may "
      "slip or stick (roll).  The simulation uses Drake's MultibodyPlant with "
      "SceneGraph visualization.  Launch Drake-visualizer before running the "
      "example by typing something like ./bazel-bin/tools/drake_visualizer &");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::logging::HandleSpdlogGflags();
  return drake::multibody::examples::inclined_plane_with_body::do_main();
}

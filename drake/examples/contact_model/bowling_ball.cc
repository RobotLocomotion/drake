/** @file
 A simple example showing a bowling ball knocking over pins.  The ball has an
 initial linear velocity parallel with the axis of the lane, and an angular
 velocity around that same vector (i.e., spinning perpendicular to its direction
 of motion).

 In this case, it illustrates the change from dynamic to static friction as the
 ball slides along the lane.  The slide becomes a roll as the relative velocity
 at the point of contact between ball and lane slows to zero.

 This scenario (the definitions in the URDF and default values here) are
 inspired by *this* discussion of bowling physics:
    http://www.real-world-physics-problems.com/physics-of-bowling.html

 There are physical artifacts that are *not* captured in this simulation.
 This doesn't model the change in the coefficient of friction due to position; a
 real bowling lane is oiled in for the first 2/3 of its length.
 As such, the parameters have to be tweaked to get the desired outcome.
 Rotational velocity has been modified (pointing straight back with a lower
 magnitude) and higher initial linear velocity.
 The lane is oriented along the x-axis (on the x-y plane).
*/

#include <cmath>
#include <iomanip>
#include <limits>
#include <memory>

#include <gflags/gflags.h>

#include "drake/common/eigen_types.h"
#include "drake/common/find_resource.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/systems/analysis/runge_kutta2_integrator.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace systems {

using drake::lcm::DrakeLcm;
using drake::multibody::joints::kQuaternion;
using Eigen::VectorXd;
using std::make_unique;

// Simulation parameters.
DEFINE_double(v, 12, "The ball's initial linear speed down the lane (m/s)");
DEFINE_double(timestep, 2e-4, "The simulator time step (s)");
DEFINE_double(w, 25,
              "The ball's initial angular speed (around [-1, 0 ,0] (rad/s).");
DEFINE_double(youngs_modulus, 1e8, "The contact model's Young's modulus (Pa)");
DEFINE_double(us, 0.4, "The static coefficient of friction");
DEFINE_double(ud, 0.2, "The dynamic coefficient of friction");
DEFINE_double(v_tol, 0.01,
              "The maximum slipping speed allowed during stiction (m/s)");
DEFINE_double(dissipation, 2, "The contact model's dissipation (s/m)");
DEFINE_double(contact_area, 1e-3,
              "The characteristic scale of contact area (m^2)");
DEFINE_double(sim_duration, 3, "The simulation duration (s)");
DEFINE_int32(pin_count, 10, "The number of pins -- in the range [0, 10]");
DEFINE_bool(playback, true, "If true, loops playback of simulation");

// Bowling ball rolled down a conceptual lane to strike pins.
int main() {
  using std::cerr;
  using std::cout;

  cout << "Parameters:\n";
  cout << "\ttimestep:         " << FLAGS_timestep << "\n";
  cout << "\tv:                " << FLAGS_v << "\n";
  cout << "\tω:                " << FLAGS_w << "\n";
  cout << "\tyoung's modulus:  " << FLAGS_youngs_modulus << "\n";
  cout << "\tstatic friction:  " << FLAGS_us << "\n";
  cout << "\tdynamic friction: " << FLAGS_ud << "\n";
  cout << "\tslip threshold:   " << FLAGS_v_tol << "\n";
  cout << "\tContact area:     " << FLAGS_contact_area << "\n";
  cout << "\tdissipation:      " << FLAGS_dissipation << "\n";
  cout << "\tpin count:        " << FLAGS_pin_count << "\n";

  if (FLAGS_pin_count < 0 || FLAGS_pin_count > 10) {
    cerr << "Bad number of pins specified.  Must be in the range [0, 10]\n";
    return 1;
  }

  DiagramBuilder<double> builder;

  // Create RigidBodyTree.
  auto tree_ptr = make_unique<RigidBodyTree<double>>();
  drake::parsers::urdf::AddModelInstanceFromUrdfFile(
      FindResourceOrThrow("drake/examples/contact_model/bowling_ball.urdf"),
      kQuaternion, nullptr /* weld to frame */, tree_ptr.get());

  for (int i = 0; i < FLAGS_pin_count; ++i) {
    drake::parsers::urdf::AddModelInstanceFromUrdfFile(
        FindResourceOrThrow("drake/examples/contact_model/pin.urdf"),
        kQuaternion, nullptr /* weld to frame */, tree_ptr.get());
  }
  multibody::AddFlatTerrainToWorld(tree_ptr.get(), 100., 10.);

  // Instantiate a RigidBodyPlant from the RigidBodyTree.
  auto& plant = *builder.AddSystem<RigidBodyPlant<double>>(move(tree_ptr));
  plant.set_name("plant");

  // Note: this sets identical contact parameters across all object pairs:
  // ball-lane, ball-pin, and pin-pin.
  CompliantMaterial default_material;
  default_material.set_youngs_modulus(FLAGS_youngs_modulus)
      .set_dissipation(FLAGS_dissipation)
      .set_friction(FLAGS_us, FLAGS_ud);
  plant.set_default_compliant_material(default_material);
  CompliantContactModelParameters model_parameters;
  model_parameters.characteristic_area = FLAGS_contact_area;
  model_parameters.v_stiction_tolerance = FLAGS_v_tol;
  plant.set_contact_model_parameters(model_parameters);

  const auto& tree = plant.get_rigid_body_tree();

  // LCM communication.
  DrakeLcm lcm;

  // Visualizer.
  const auto visualizer_publisher =
      builder.template AddSystem<DrakeVisualizer>(tree, &lcm, true);
  visualizer_publisher->set_name("visualizer_publisher");

  // Raw state vector to visualizer.
  builder.Connect(plant.state_output_port(),
                  visualizer_publisher->get_input_port(0));

  auto diagram = builder.Build();

  // Create simulator.
  auto simulator = std::make_unique<Simulator<double>>(*diagram);
  Context<double>& context = simulator->get_mutable_context();
  simulator->reset_integrator<RungeKutta2Integrator<double>>(*diagram,
                                                             FLAGS_timestep,
                                                             &context);
  // Set initial state.
  Context<double>& plant_context =
      diagram->GetMutableSubsystemContext(plant, &context);

  // 1 floating quat joint = |xyz|, |q|, |w|, |xyzdot| = 3 + 4 + 3 + 3.
  const int kStateSize = 13 * (1 + FLAGS_pin_count);
  const double kBallRadius = 0.1085;  // This must agree with the URDF file.

  VectorX<double> initial_state(kStateSize);
  initial_state = VectorX<double>::Zero(kStateSize);
  // Set positions
  int idx = 0;
  // Bowling ball -- simply move it *up* the ball's radius.
  initial_state.segment<7>(idx) << 0, 0, kBallRadius, 1, 0, 0, 0;
  idx += 7;
  // Layout of the pins in an equilateral triangle.  Head pin at a *local*
  // origin, with pins 12 inches apart (diagonally).
  //
  //            0          ------> y
  //          1   2        |
  //        3   4   5      |
  //      6   7   8   9    X
  // Set the origin to be 15 meters "down" the lane (along the x-axis) and
  // half way "across" (along y-axis) a 1-meter-wide lane).
  const double kPinOriginX = 15.0;
  const double kPinOriginY = 0.5;
  // The pin's geometric origin is 0.109 m above the "bottom" of the pin.
  const double kPinZ = 0.109;
  const double kCos60 = std::cos(60.0 / 180 * M_PI) * 12 * 0.0254;
  const double kSin60 = std::sin(60.0 / 180 * M_PI) * 12 * 0.0254;
  const double pins_pos[] = { 0, 0,
                              -kCos60, kSin60,
                              kCos60, kSin60,
                              -kCos60 * 2, kSin60 * 2,
                              0, kSin60 * 2,
                              kCos60 * 2, kSin60 * 2,
                              -kCos60 * 3, kSin60 * 3,
                              -kCos60, kSin60 * 3,
                              kCos60, kSin60 * 3,
                              kCos60 * 3, kSin60 * 3};

  for (int i = 0; i < FLAGS_pin_count; ++i) {
    double y = pins_pos[i * 2] + kPinOriginY;
    double x = pins_pos[i * 2 + 1] + kPinOriginX;
    initial_state.segment<7>(idx) << x, y, kPinZ, 1, 0, 0, 0;
    idx += 7;
  }
  // Axis of rotation is <-1 0 0>: maximal rotation velocity perpendicular
  // to the linear velocity.  Angular speed is provided from input parameter: w.
  initial_state.segment<6>(idx) << -FLAGS_w, 0, 0, FLAGS_v, 0, 0;

  plant.set_state_vector(&plant_context, initial_state);

  simulator->StepTo(FLAGS_sim_duration);

  while (FLAGS_playback) visualizer_publisher->ReplayCachedSimulation();

  return 0;
}
}  // namespace systems
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::systems::main();
}

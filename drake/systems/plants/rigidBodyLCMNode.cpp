#include <limits>
#include <string>

#include <gflags/gflags.h>

#include "drake/common/text_logging.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/system1/LCMSystem.h"
#include "drake/system1/cascade_system.h"
#include "drake/systems/plants/BotVisualizer.h"
#include "drake/systems/plants/RigidBodySystem.h"
#include "drake/systems/plants/joints/floating_base_types.h"

using std::make_shared;
using std::shared_ptr;
using std::string;

using Eigen::Isometry3d;
using Eigen::Vector3d;
using Eigen::Vector4d;

using drake::systems::plants::joints::FloatingBaseType;
using drake::systems::plants::joints::kFixed;
using drake::systems::plants::joints::kQuaternion;
using drake::systems::plants::joints::kRollPitchYaw;

/** @page rigidBodyLCMNode rigidBodyLCMNode Application
 * @ingroup simulation
 * @brief Loads a urdf/sdf and simulates it, subscribing to LCM inputs and
publishing LCM outputs
 *
 * This application loads a robot from urdf or sdf and runs a simulation, with
every input
 * with an LCM type defined subscribed to the associated LCM channels, and every
 * output with an LCM type defined publishing on the associate channels.  See
@ref lcm_vector_concept.
 *
 *
@verbatim
Usage:  rigidBodyLCMNode [options] full_path_to_urdf_or_sdf_file
  with (case sensitive) options:
    --base [floating_type]  // can be "kFixed, kRollPitchYaw, or kQuaternion"
(default: kQuaternion)
@endverbatim
 */

DEFINE_string(base, "QUAT",
              "defines the connection between the root link and the world; "
              "must be kFixed or RPY or QUAT");
DEFINE_bool(add_flat_terrain, false, "add flat terrain");

namespace drake {
namespace systems {
namespace plants {

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage("[options] full_path_to_urdf_or_sdf_file");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  if (argc < 2) {
    gflags::ShowUsageWithFlags(argv[0]);
    return 1;
  }
  logging::HandleSpdlogGflags();

  // todo: consider moving this logic into the RigidBodySystem class so it can
  // be reused
  FloatingBaseType floating_base_type = kQuaternion;
  if (FLAGS_base == "kFixed") {
    floating_base_type = kFixed;
  } else if (FLAGS_base == "RPY") {
    floating_base_type = kRollPitchYaw;
  } else if (FLAGS_base == "QUAT") {
    floating_base_type = kQuaternion;
  } else {
    throw std::runtime_error(string("Unknown base type") + FLAGS_base +
                             "; must be kFixed, RPY, or QUAT");
  }

  auto rigid_body_sys = make_shared<RigidBodySystem>();
  rigid_body_sys->AddModelInstanceFromFile(argv[argc - 1], floating_base_type);
  auto const& tree = rigid_body_sys->getRigidBodyTree();

  if (FLAGS_add_flat_terrain) {
    SPDLOG_TRACE(drake::log(), "adding flat terrain");
    double box_width = 1000;
    double box_depth = 10;
    DrakeShapes::Box geom(Vector3d(box_width, box_width, box_depth));
    Isometry3d T_element_to_link = Isometry3d::Identity();
    T_element_to_link.translation() << 0, 0,
        -box_depth / 2;  // top of the box is at z=0
    RigidBody& world = tree->world();
    Vector4d color;
    color << 0.9297, 0.7930, 0.6758,
        1;  // was hex2dec({'ee','cb','ad'})'/256 in matlab
    world.AddVisualElement(
        DrakeShapes::VisualElement(geom, T_element_to_link, color));
    tree->addCollisionElement(
        DrakeCollision::Element(geom, T_element_to_link, &world), world,
        "terrain");
    tree->updateStaticCollisionElements();
  }

  shared_ptr<lcm::LCM> lcm = make_shared<lcm::LCM>();
  auto visualizer =
      make_shared<BotVisualizer<RigidBodySystem::StateVector>>(lcm, tree);
  auto sys = cascade(rigid_body_sys, visualizer);

  SimulationOptions options;
  options.realtime_factor = 1.0;
  options.timeout_seconds = std::numeric_limits<double>::infinity();
  options.initial_step_size = 5e-3;

  runLCM(sys, lcm, 0, std::numeric_limits<double>::infinity(),
         getInitialState(*sys), options);
  //  simulate(*sys, 0, std::numeric_limits<double>::max(),
  //  getInitialState(*sys), options);

  return 0;
}

}  // namespace plants
}  // namespace systems
}  // namespace drake

int main(int argc, char* argv[]) {
  return drake::systems::plants::main(argc, argv);
}

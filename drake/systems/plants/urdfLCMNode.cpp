
#include "drake/systems/LCMSystem.h"
#include "drake/systems/plants/RigidBodySystem.h"
#include "drake/systems/plants/BotVisualizer.h"
#include "drake/util/drakeAppUtil.h"

using namespace std;
using namespace Eigen;
using namespace Drake;

/** @page urdfLCMNode urdfLCMNode Application
 * @ingroup simulation
 * @brief Loads a urdf and simulates it, subscribing to LCM inputs and publishing LCM outputs
 *
 * This application loads a robot from urdf and runs a simulation, with every input
 * with an LCM type defined subscribed to the associated LCM channels, and every
 * output with an LCM type defined publishing on the associate channels.  See @ref lcm_vector_concept.
 *
 *
@verbatim
Usage:  urdfLCMNode [options] full_path_to_urdf_file
  with (case sensitive) options:
    --base [floating_type]  // can be "FIXED, ROLLPITCHYAW,or QUATERNION" (default: QUATERNION)
@endverbatim
 */

int main(int argc, char* argv[]) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " [options] full_path_to_urdf_file" << std::endl;
    return 1;
  }

  // todo: consider moving this logic into the RigidBodySystem class so it can be reused
  DrakeJoint::FloatingBaseType floating_base_type = DrakeJoint::QUATERNION;
  char* floating_base_option = getCommandLineOption(argv,argc+argv,"--base");
  if (floating_base_option) {
    if (strcmp(floating_base_option,"FIXED")==0) { floating_base_type = DrakeJoint::FIXED; }
    else if (strcmp(floating_base_option,"RPY")==0) { floating_base_type = DrakeJoint::ROLLPITCHYAW; }
    else if (strcmp(floating_base_option,"QUAT")==0) { floating_base_type = DrakeJoint::QUATERNION; }
    else { throw std::runtime_error(string("Unknown base type") + floating_base_option + "; must be FIXED,RPY, or QUAT"); }
  }

  auto rigid_body_sys = make_shared<RigidBodySystem>(argv[argc-1],floating_base_type);
  auto const & tree = rigid_body_sys->getRigidBodyTree();

  if (commandLineOptionExists(argv,argc+argv,"--add_flat_terrain")) {
    double box_width = 1000;
    double box_depth = 10;
    DrakeShapes::Box geom(Vector3d(box_width, box_width, box_depth));
    Isometry3d T_element_to_link = Isometry3d::Identity();
    T_element_to_link.translation() << 0,0,-box_depth/2;  // top of the box is at z=0
    auto & world = tree->bodies[0];
    Vector4d color;  color <<  0.9297, 0.7930, 0.6758, 1;  // was hex2dec({'ee','cb','ad'})'/256 in matlab
    world->addVisualElement(DrakeShapes::VisualElement(geom,T_element_to_link,color));
    tree->addCollisionElement(RigidBody::CollisionElement(geom,T_element_to_link,world),world,"terrain");
    tree->updateStaticCollisionElements();
  }

  shared_ptr<lcm::LCM> lcm = make_shared<lcm::LCM>();
  auto visualizer = make_shared<BotVisualizer<RigidBodySystem::StateVector>>(lcm,tree);
  auto sys = cascade(rigid_body_sys, visualizer);

  SimulationOptions options = default_simulation_options;
  options.realtime_factor = 0.0;
  options.initial_step_size = 5e-3;

  runLCM(sys,lcm,0,std::numeric_limits<double>::max(),getInitialState(*sys),options);
//  simulate(*sys,0,std::numeric_limits<double>::max(),getInitialState(*sys),options);

  return 0;
}
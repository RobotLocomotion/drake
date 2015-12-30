
#include "LCMSystem.h"
#include "RigidBodySystem.h"
#include "BotVisualizer.h"
#include "drakeAppUtil.h"

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

  shared_ptr<lcm::LCM> lcm = make_shared<lcm::LCM>();

  auto tree = make_shared<RigidBodyTree>(argv[argc-1],floating_base_type);
  auto rigid_body_sys = make_shared<RigidBodySystem>(tree);
  auto visualizer = make_shared<BotVisualizer<RigidBodySystem::StateVector>>(lcm,argv[argc-1],floating_base_type);
  auto sys = cascade(rigid_body_sys, visualizer);

  runLCM(sys,lcm,0,std::numeric_limits<double>::max());

  return 0;
}
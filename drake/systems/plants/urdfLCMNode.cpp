
#include "LCMSystem.h"
#include "RigidBodySystem.h"
#include "BotVisualizer.h"

using namespace std;
using namespace Eigen;
using namespace Drake;

int main(int argc, char* argv[]) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " full_path_to_urdf_file" << std::endl;
    return 1;
  }

  shared_ptr<lcm::LCM> lcm = make_shared<lcm::LCM>();

  auto tree = make_shared<RigidBodyTree>(argv[1],DrakeJoint::FIXED);
  auto rigid_body_sys = make_shared<RigidBodySystem>(tree);
  auto visualizer = make_shared<BotVisualizer<RigidBodySystem::StateVector>>(lcm,argv[1],DrakeJoint::FIXED);
  auto sys = cascade(rigid_body_sys, visualizer);

  VectorXd x0 = VectorXd::Random(tree->num_positions+tree->num_velocities);

  runLCM(sys,lcm,0,std::numeric_limits<double>::max(),x0);

  return 0;
}
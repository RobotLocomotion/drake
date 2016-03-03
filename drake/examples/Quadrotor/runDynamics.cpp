#include "drake/systems/plants/BotVisualizer.h"
#include "drake/systems/plants/RigidBodySystem.h"
#include "drake/systems/LCMSystem.h"
#include "drake/systems/LinearSystem.h"

#include "QuadrotorControl.h"
#include "QuadrotorState.h"

using namespace std;
using namespace Drake;
using namespace Eigen;

int main(int argc, char* argv[]) {  
  double final_time = argc >= 2 ? atof(argv[1]) : std::numeric_limits<double>::infinity();
  cout << "Running simulation for " << final_time << " seconds." << endl;
  shared_ptr<lcm::LCM> lcm = make_shared<lcm::LCM>();
  if(!lcm->good())
    return 1;

  DrakeJoint::FloatingBaseType floating_base_type = DrakeJoint::QUATERNION;
  auto rigid_body_sys = make_shared<RigidBodySystem>(getDrakePath()+"/examples/Quadrotor/quadrotor.urdf",floating_base_type);
  auto const & tree = rigid_body_sys->getRigidBodyTree();

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
  
  auto visualizer = make_shared<BotVisualizer<RigidBodySystem::StateVector>>(lcm,tree);
  
  auto quad_control_to_rbsys_input = make_shared<Gain<QuadrotorControl, RigidBodySystem::InputVector>>(Eigen::Matrix4d::Identity());
  auto rbsys_output_to_quad_state = make_shared<Gain<RigidBodySystem::StateVector, QuadrotorState>>(Eigen::Matrix<double, 13, 13>::Identity());
  
  auto sys_with_lcm_input = cascade(quad_control_to_rbsys_input, rigid_body_sys);
  
  auto sys_with_vis = cascade(sys_with_lcm_input, visualizer);

  SimulationOptions options = default_simulation_options;
  options.realtime_factor = 1.0;
  options.initial_step_size = 0.0025;

  VectorXd x0 = VectorXd::Zero(rigid_body_sys->getNumStates());
  x0.head(tree->num_positions) = tree->getZeroConfiguration();
    
  auto lcmio_with_vis = cascade(sys_with_vis, rbsys_output_to_quad_state);

  x0(2) = 1;

  runLCM(lcmio_with_vis, lcm, 0, final_time, x0, options);
  
}

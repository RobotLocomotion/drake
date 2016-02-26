
#include <iostream>
#include "Quadrotor.h"
#include "BotVisualizer.h"
#include "RigidBodySystem.h"
#include "LCMSystem.h"

using namespace std;
using namespace Drake;
using namespace Eigen;

int main(int argc, char* argv[]) {
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
  
  auto sys = cascade(rigid_body_sys, visualizer);

  SimulationOptions options = default_simulation_options;
  options.realtime_factor = 1.0;
  options.initial_step_size = 0.005;

  VectorXd x0(rigid_body_sys->getNumStates());
  x0.head(tree->num_positions) = tree->getZeroConfiguration();

  x0(2) = 0.1;

  runLCM(sys,lcm,0,std::numeric_limits<double>::infinity(),x0,options);
  
}

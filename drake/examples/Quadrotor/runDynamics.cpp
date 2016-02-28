
#include <iostream>
#include "Quadrotor.h"
#include "BotVisualizer.h"
#include "RigidBodySystem.h"
#include "LCMSystem.h"
#include "lcmtypes/drake/lcmt_quadrotor_control_t.hpp"

using namespace std;
using namespace Drake;
using namespace Eigen;

template <typename ScalarType = double>
class QuadrotorControl {
public:
  typedef drake::lcmt_quadrotor_control_t LCMMessageType;
  static std::string channel() { return "QUAD_CONTROL"; };
  
  template <typename Derived>
  QuadrotorControl(const Eigen::MatrixBase<Derived>& x) : motors(x) {};

  template <typename Derived>
  QuadrotorControl& operator=(const Eigen::MatrixBase<Derived>& x) {
    motors = x;
    return *this;
  }

  friend Eigen::Vector4d toEigen(const QuadrotorControl<ScalarType>& vec) {    
    return vec.motors;
  }

  friend std::string getCoordinateName(const QuadrotorControl<ScalarType>& vec, unsigned int index) {    
    return index >= 0 && index < RowsAtCompileTime ? std::string("motor") + std::to_string(index) : std::string("error");
  }

  const static int RowsAtCompileTime = 4;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Matrix<ScalarType, 4, 1> motors;
};

bool decode(const drake::lcmt_quadrotor_control_t& msg, double& t, QuadrotorControl<double>& x) {
  t = double(msg.timestamp)/1000.0;
  x.motors = Eigen::Vector4d(msg.motors);
  return true;
}

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

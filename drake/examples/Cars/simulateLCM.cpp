
#include "LCMSystem.h"
#include "RigidBodySystem.h"
#include "LinearSystem.h"
#include "BotVisualizer.h"
#include "drakeAppUtil.h"
#include "lcmtypes/drake/lcmt_driving_control_cmd_t.hpp"

using namespace std;
using namespace Eigen;
using namespace Drake;

template <typename ScalarType = double>
class DrivingCommand {
public:
  typedef drake::lcmt_driving_control_cmd_t LCMMessageType;
  static std::string channel() { return "DRIVING_COMMAND"; };

  DrivingCommand(void) : throttle(0), brake(0), steering_angle(0) {};
  template <typename Derived>
  DrivingCommand(const Eigen::MatrixBase<Derived>& x) : steering_angle(x(0)), throttle(x(1)), brake(x(2)) {};

  template <typename Derived>
  DrivingCommand& operator=(const Eigen::MatrixBase<Derived>& x) {
    steering_angle = x(0);
    throttle = x(1);
    brake = x(2);
    return *this;
  }

  friend std::string getCoordinateName(const DrivingCommand<ScalarType>& vec, unsigned int index) {
    switch (index) {
      case 0: return "steering_angle";
      case 1: return "throttle";
      case 2: return "brake";
    }
    return "error";
  }
  const static int RowsAtCompileTime = 3;

  ScalarType steering_angle;
  ScalarType throttle;
  ScalarType brake;
};

bool decode(const drake::lcmt_driving_control_cmd_t& msg, double& t, DrivingCommand<double>& x) {
  t = double(msg.timestamp)/1000.0;
  x.steering_angle = msg.steering_angle;
  x.throttle = msg.throttle_value;
  x.brake = msg.brake_value;
  return true;
}


int main(int argc, char* argv[]) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " [options] full_path_to_urdf_file" << std::endl;
    return 1;
  }

  // todo: consider moving this logic into the RigidBodySystem class so it can be reused
  DrakeJoint::FloatingBaseType floating_base_type = DrakeJoint::QUATERNION;

  auto tree = make_shared<RigidBodyTree>(argv[argc-1],floating_base_type);

  { // add flat terrain
    double box_width = 1000;
    double box_depth = 10;
    DrakeShapes::Box geom(Vector3d(box_width, box_width, box_depth));
    Matrix4d T_element_to_link = Matrix4d::Identity();
    T_element_to_link(2,3) = -box_depth/2;  // top of the box is at z=0
    auto & world = tree->bodies[0];
    Vector4d color;  color <<  0.9297, 0.7930, 0.6758, 1;  // was hex2dec({'ee','cb','ad'})'/256 in matlab
    world->addVisualElement(DrakeShapes::VisualElement(geom,T_element_to_link,color));
    tree->addCollisionElement(RigidBody::CollisionElement(geom,T_element_to_link,world),world,"terrain");
    tree->updateStaticCollisionElements();
  }

  shared_ptr<lcm::LCM> lcm = make_shared<lcm::LCM>();
  auto rigid_body_sys = make_shared<RigidBodySystem>(tree);

  MatrixXd D(getNumInputs(*rigid_body_sys),DrivingCommand<double>::RowsAtCompileTime+getNumStates(*rigid_body_sys));
  { // setup PD controller for throttle and steering
    double kp = 100, kd = 20, kThrottle = 100;
    D.setZero();
/*
    D(1,controlInput.findCoordinateIndex('steering')) = -kp; % Steering
            D(1,controlInput.findCoordinateIndex('steeringdot')) = -kd; % Steering Dot
    D(1,controlInput.findCoordinateIndex('steering_angle')) = kp; % Steering Desired
    D(2,controlInput.findCoordinateIndex('throttle_value')) = kThrottle; % Gas
            D(2,controlInput.findCoordinateIndex('brake_value')) = -kThrottle; % Brake
            D(3,controlInput.findCoordinateIndex('throttle_value')) = kThrottle; % Gas
            D(3,controlInput.findCoordinateIndex('brake_value')) = -kThrottle; % Brake
*/
  }
  auto pd_controller = make_shared<Gain<CombinedVectorUtil<DrivingCommand,RigidBodySystem::StateVector>::type,RigidBodySystem::InputVector>>(D);

  auto visualizer = make_shared<BotVisualizer<RigidBodySystem::StateVector>>(lcm,tree);
  auto sys = cascade(rigid_body_sys, visualizer);

  SimulationOptions options = default_simulation_options;
  rigid_body_sys->penetration_stiffness = 5000.0;
  rigid_body_sys->penetration_damping = rigid_body_sys->penetration_stiffness/10.0;
  options.initial_step_size = 5e-3;

  VectorXd x0(rigid_body_sys->getNumStates());
  x0.head(tree->num_positions) = tree->getZeroConfiguration();

//  runLCM(sys,lcm,0,std::numeric_limits<double>::max(),getInitialState(*sys),options);
  simulate(*sys,0,std::numeric_limits<double>::max(),x0,options);

  return 0;
}
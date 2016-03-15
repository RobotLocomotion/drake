
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
  static std::string channel() { return "DRIVING_COMMAND"; }

  DrivingCommand(void) : throttle(0), brake(0), steering_angle(0) {}
  template <typename Derived>
  DrivingCommand(const Eigen::MatrixBase<Derived>& x)
      : steering_angle(x(0)), throttle(x(1)), brake(x(2)) {}

  template <typename Derived>
  DrivingCommand& operator=(const Eigen::MatrixBase<Derived>& x) {
    steering_angle = x(0);
    throttle = x(1);
    brake = x(2);
    return *this;
  }

  friend Eigen::Vector3d toEigen(const DrivingCommand<ScalarType>& vec) {
    Eigen::Vector3d x;
    x << vec.steering_angle, vec.throttle, vec.brake;
    return x;
  }

  friend std::string getCoordinateName(const DrivingCommand<ScalarType>& vec,
                                       unsigned int index) {
    switch (index) {
      case 0:
        return "steering_angle";
      case 1:
        return "throttle";
      case 2:
        return "brake";
    }
    return "error";
  }
  const static int RowsAtCompileTime = 3;

  ScalarType steering_angle;
  ScalarType throttle;
  ScalarType brake;
};

bool decode(const drake::lcmt_driving_control_cmd_t& msg, double& t,
            DrivingCommand<double>& x) {
  t = double(msg.timestamp) / 1000.0;
  x.steering_angle = msg.steering_angle;
  x.throttle = msg.throttle_value;
  x.brake = msg.brake_value;
  return true;
}

/** Driving Simulator
 * Usage:  simulateLCM vehicle_urdf [world_urdf files ...]
 */

int main(int argc, char* argv[]) {
  PRINT_FUNCTION_NAME;
  
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " vehicle_urdf [world sdf files ...]"
              << std::endl;
    return 1;
  }

  // todo: consider moving this logic into the RigidBodySystem class so it can
  // be reused
  DrakeJoint::FloatingBaseType floating_base_type = DrakeJoint::QUATERNION;

  auto rigid_body_sys =
      make_shared<RigidBodySystem>(argv[1], floating_base_type);
  rigid_body_sys->use_multi_contact = false;
  auto const& tree = rigid_body_sys->getRigidBodyTree();
  for (int i = 2; i < argc; i++)
    tree->addRobotFromSDF(argv[i], DrakeJoint::FIXED);  // add environment

  if (argc < 3) {  // add flat terrain
    double box_width = 1000;
    double box_depth = 10;
    DrakeShapes::Box geom(Vector3d(box_width, box_width, box_depth));
    Isometry3d T_element_to_link = Isometry3d::Identity();
    T_element_to_link.translation() << 0, 0,
        -box_depth / 2;  // top of the box is at z=0
    auto& world = tree->bodies[0]; //world is a body with zero mass and zero moment of inertia
    Vector4d color;
    color << 0.9297, 0.7930, 0.6758,
        1;  // was hex2dec({'ee','cb','ad'})'/256 in matlab
    world->addVisualElement(
        DrakeShapes::VisualElement(geom, T_element_to_link, color));
    tree->addCollisionElement(
        RigidBody::CollisionElement(geom, T_element_to_link, world), world,
        "terrain");
    tree->updateStaticCollisionElements();
  }

  //tree->drawKinematicTree("graphiviz_test.dot"); //Convert to png image file: dot -Tpng graphiviz_test.dot -o graphiviz_test.png

  shared_ptr<lcm::LCM> lcm = make_shared<lcm::LCM>();

  PRINT_VAR(tree->bodies.size());
  for(auto body: tree->bodies){
    PRINT_VAR(body->linkname);
    PRINT_VAR(body->mass);
    PRINT_VAR(body->hasParent())
    if(body->hasParent())
      PRINT_VAR(body->getJoint().getName());
  }

  PRINT_VAR(getNumInputs(*rigid_body_sys));
  PRINT_VAR(tree->num_positions);
  PRINT_VAR(tree->num_velocities);

  //this replaces the above commented out code with the "auto sys = cascade(vehicle_sys, visualizer);" at the end
  auto visualizer =
      make_shared<BotVisualizer<RigidBodySystem::StateVector>>(lcm, tree);
  auto sys = cascade(rigid_body_sys, visualizer);  

  SimulationOptions options = default_simulation_options;
  rigid_body_sys->penetration_stiffness = 5000.0;
  rigid_body_sys->penetration_damping =
      rigid_body_sys->penetration_stiffness / 10.0;
  rigid_body_sys->friction_coefficient = 10.0;  // essentially infinite friction
  options.initial_step_size = 5.0e-3;
  options.timeout_seconds = numeric_limits<double>::infinity();
  options.wait_for_keypress = true;
  options.rk2 = true;

  VectorXd x0(rigid_body_sys->getNumStates());
  x0.head(tree->num_positions) = tree->getZeroConfiguration();
  // todo:  call getInitialState instead?  (but currently, that would require
  // snopt).  needs #1627
  // I'm getting away without it, but might be generating large internal forces
  // initially as the ackerman constraint (hopefully) gets enforced by the
  // stabilization terms.

  runLCM(sys, lcm, 0, std::numeric_limits<double>::infinity(), x0, options);
  //  simulate(*sys,0,std::numeric_limits<double>::infinity(),x0,options);

  return 0;
}

#include "drake/systems/LCMSystem.h"
#include "drake/systems/LinearSystem.h"
#include "drake/systems/pd_control_system.h"
#include "drake/systems/plants/BotVisualizer.h"
#include "drake/systems/plants/RigidBodySystem.h"
#include "drake/util/drakeAppUtil.h"
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
  static const int RowsAtCompileTime = 3;

  ScalarType steering_angle;
  ScalarType throttle;
  ScalarType brake;
};

/**
 * A toString method for DrivingCommand.
 */
template <typename ScalarType = double>
std::ostream& operator<<(std::ostream& os,
                         const DrivingCommand<ScalarType>& dc) {
  return os << "[steering_angle = " << dc.steering_angle
            << ", throttle = " << dc.throttle << ", brake = " << dc.brake
            << "]";
}

bool decode(const drake::lcmt_driving_control_cmd_t& msg, double& t,
            DrivingCommand<double>& x) {
  t = double(msg.timestamp) / 1000.0;
  x.steering_angle = msg.steering_angle;
  x.throttle = msg.throttle_value;
  x.brake = msg.brake_value;
  return true;
}

/** Driving Simulator
 * Usage:  simulateLCM vehicle_model_file [world_model files ...]
 */

int main(int argc, char* argv[]) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " vehicle_urdf [world sdf files ...]"
              << std::endl;
    return 1;
  }

  // todo: consider moving this logic into the RigidBodySystem class so it can
  // be reused
  DrakeJoint::FloatingBaseType floating_base_type = DrakeJoint::QUATERNION;

  auto rigid_body_sys = make_shared<RigidBodySystem>();

  // The following variable, weld_to_frame, is only needed if the model is a
  // URDF file. It is needed since URDF does not specify the location and
  // orientation of the car's root node in the world. If the model is an SDF,
  // weld_to_frame is ignored by the parser.
  auto weld_to_frame = allocate_shared<RigidBodyFrame>(
      aligned_allocator<RigidBodyFrame>(),
      // Weld the model to the world link.
      "world",

      // A pointer to a rigid body to which to weld the model is not needed
      // since the model will be welded to the world, which can by automatically
      // found within the rigid body tree.
      nullptr,

      // The following parameter specifies the X,Y,Z position of the car's root
      // link in the world's frame. The kinematics of the car model requires
      // that its root link be elevated along the Z-axis by 0.378326m.
      Eigen::Vector3d(0, 0, 0.378326),

      // The following parameter specifies the roll, pitch, and yaw of the car's
      // root link in the world's frame.
      Eigen::Vector3d(0, 0, 0));

  rigid_body_sys->addRobotFromFile(argv[1], floating_base_type, weld_to_frame);

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
    auto& world = tree->bodies[0];
    Vector4d color;
    color << 0.9297, 0.7930, 0.6758,
        1;  // was hex2dec({'ee','cb','ad'})'/256 in matlab
    world->addVisualElement(
        DrakeShapes::VisualElement(geom, T_element_to_link, color));
    tree->addCollisionElement(
        RigidBody::CollisionElement(geom, T_element_to_link, world), *world,
        "terrain");
    tree->updateStaticCollisionElements();
  }

  shared_ptr<lcm::LCM> lcm = make_shared<lcm::LCM>();

  MatrixXd Kp(getNumInputs(*rigid_body_sys), tree->num_positions),
      Kd(getNumInputs(*rigid_body_sys), tree->num_velocities);
  Matrix<double, Eigen::Dynamic, 3> map_driving_cmd_to_x_d(
      tree->num_positions + tree->num_velocities, 3);
  {  // setup PD controller for throttle and steering
    double kpSteering = 400, kdSteering = 80, kThrottle = 100;
    Kp.setZero();
    Kd.setZero();
    map_driving_cmd_to_x_d.setZero();

    for (int actuator_idx = 0; actuator_idx < tree->actuators.size();
         actuator_idx++) {
      const std::string& actuator_name = tree->actuators[actuator_idx].name;

      if (actuator_name == "steering") {
        auto const& b = tree->actuators[actuator_idx].body;
        Kp(actuator_idx, b->position_num_start) = kpSteering;  // steering
        Kd(actuator_idx, b->velocity_num_start) = kdSteering;  // steeringdot
        map_driving_cmd_to_x_d(b->position_num_start, 0) =
            1;  // steering command

      } else if (actuator_name == "right_wheel_joint" ||
                 actuator_name == "left_wheel_joint") {
        auto const& b = tree->actuators[actuator_idx].body;
        Kd(actuator_idx, b->velocity_num_start) = kThrottle;  // throttle
        map_driving_cmd_to_x_d(tree->num_positions + b->velocity_num_start, 1) =
            20;  // throttle (velocity) command
        map_driving_cmd_to_x_d(tree->num_positions + b->velocity_num_start, 2) =
            -20;  // braking (velocity) command
      }
    }
  }
  auto vehicle_with_pd =
      make_shared<PDControlSystem<RigidBodySystem>>(rigid_body_sys, Kp, Kd);
  auto vehicle_sys = cascade(
      make_shared<
          Gain<DrivingCommand, PDControlSystem<RigidBodySystem>::InputVector>>(
          map_driving_cmd_to_x_d),
      vehicle_with_pd);
  auto visualizer =
      make_shared<BotVisualizer<RigidBodySystem::StateVector>>(lcm, tree);
  auto sys = cascade(vehicle_sys, visualizer);

  SimulationOptions options = default_simulation_options;
  rigid_body_sys->penetration_stiffness = 5000.0;
  rigid_body_sys->penetration_damping =
      rigid_body_sys->penetration_stiffness / 10.0;
  rigid_body_sys->friction_coefficient = 10.0;  // essentially infinite friction
  options.initial_step_size = 5e-3;
  options.timeout_seconds = numeric_limits<double>::infinity();

  VectorXd x0 = VectorXd::Zero(rigid_body_sys->getNumStates());
  x0.head(tree->num_positions) = tree->getZeroConfiguration();
  // todo:  call getInitialState instead?  (but currently, that would require
  // snopt).  needs #1627
  // I'm getting away without it, but might be generating large internal forces
  // initially as the ackerman constraint (hopefully) gets enforced by the
  // stabilization terms.

  runLCM(sys, lcm, 0, std::numeric_limits<double>::infinity(), x0, options);
  //  simulate(*sys, 0, std::numeric_limits<double>::infinity(), x0, options);

  return 0;
}

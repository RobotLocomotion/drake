#include "drake/examples/Cars/car.h"

#include <stdlib.h>

using Eigen::MatrixXd;
using Eigen::Matrix;
using Eigen::VectorXd;

namespace drake {

std::shared_ptr<RigidBodySystem> CreateRigidBodySystem(int argc,
                                                       const char* argv[]) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " vehicle_model [world sdf files ...]"
              << std::endl;
    exit(EXIT_FAILURE);
  }

  // The Z-axis offset between Drake's world frame and the vehicle's world
  // frame.
  double z_offset = 0;

  // TODO(liangfok): Once PR 2171 is merged, modify prius.urdf to contain a
  // world link and proper offset of the chassis_floor. For more information,
  // see: https://github.com/RobotLocomotion/drake/pull/2171 and
  // https://github.com/RobotLocomotion/drake/issues/2247
  if (std::string(argv[1]).find("prius.urdf") != std::string::npos)
    z_offset = 0.378326;

  // The following variable, weld_to_frame, is only needed if the model is a
  // URDF file. It is needed since URDF does not specify the location and
  // orientation of the car's root node in the world. If the model is an SDF,
  // weld_to_frame is ignored by the parser.
  auto weld_to_frame = std::allocate_shared<RigidBodyFrame>(
      Eigen::aligned_allocator<RigidBodyFrame>(),
      // Weld the model to the world link.
      "world",

      // A pointer to a rigid body to which to weld the model is not needed
      // since the model will be welded to the world, which can by automatically
      // found within the rigid body tree.
      nullptr,

      // The following parameter specifies the X,Y,Z position of the car's root
      // link in the world's frame.
      Eigen::Vector3d(0, 0, z_offset),

      // The following parameter specifies the roll, pitch, and yaw of the car's
      // root link in the world's frame.
      Eigen::Vector3d(0, 0, 0));

  // Instantiates a rigid body system and adds the robot to it.
  auto rigid_body_sys = std::make_shared<RigidBodySystem>();
  rigid_body_sys->addRobotFromFile(argv[1], DrakeJoint::QUATERNION,
                                   weld_to_frame);

  // Adds the environment to the rigid body tree.
  auto const& tree = rigid_body_sys->getRigidBodyTree();
  for (int i = 2; i < argc; i++) {
    if (std::string(argv[i]) == "--duration") {
      i++;  // skip the next flag
    } else {
      tree->addRobotFromSDF(argv[i], DrakeJoint::FIXED);
    }
  }

  // If no environment is specified, the following code adds a flat terrain.
  if (argc < 3) {
    double box_width = 1000;
    double box_depth = 10;
    DrakeShapes::Box geom(Eigen::Vector3d(box_width, box_width, box_depth));
    Eigen::Isometry3d T_element_to_link = Eigen::Isometry3d::Identity();
    T_element_to_link.translation() << 0, 0,
        -box_depth / 2;  // top of the box is at z=0
    RigidBody& world = tree->world();
    Eigen::Vector4d color;
    color << 0.9297, 0.7930, 0.6758,
        1;  // was hex2dec({'ee','cb','ad'})'/256 in matlab
    world.addVisualElement(
        DrakeShapes::VisualElement(geom, T_element_to_link, color));
    tree->addCollisionElement(
        RigidBody::CollisionElement(geom, T_element_to_link, &world), world,
        "terrain");
    tree->updateStaticCollisionElements();
  }

  rigid_body_sys->penetration_stiffness = 5000.0;
  rigid_body_sys->penetration_damping =
      rigid_body_sys->penetration_stiffness / 10.0;
  rigid_body_sys->friction_coefficient = 10.0;  // essentially infinite friction

  return rigid_body_sys;
}

std::shared_ptr<CascadeSystem<
    Gain<DrivingCommand, PDControlSystem<RigidBodySystem>::InputVector>,
    PDControlSystem<RigidBodySystem>>>
CreateVehicleSystem(std::shared_ptr<RigidBodySystem> rigid_body_sys) {
  auto const& tree = rigid_body_sys->getRigidBodyTree();

  MatrixXd Kp(getNumInputs(*rigid_body_sys), tree->number_of_positions());
  MatrixXd Kd(getNumInputs(*rigid_body_sys), tree->number_of_velocities());

  Matrix<double, Eigen::Dynamic, 3> map_driving_cmd_to_x_d(
      tree->number_of_positions() + tree->number_of_velocities(), 3);

  // Sets up PD controllers for throttle and steering.
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
      map_driving_cmd_to_x_d(b->position_num_start, 0) = 1;  // steering command

    } else if (actuator_name == "right_wheel_joint" ||
               actuator_name == "left_wheel_joint") {
      auto const& b = tree->actuators[actuator_idx].body;
      Kd(actuator_idx, b->velocity_num_start) = kThrottle;  // throttle
      map_driving_cmd_to_x_d(
          tree->number_of_positions() + b->velocity_num_start, 1) =
          20;  // throttle (velocity) command
      map_driving_cmd_to_x_d(
          tree->number_of_positions() + b->velocity_num_start, 2) =
          -20;  // braking (velocity) command
    }
  }

  auto vehicle_with_pd = std::make_shared<PDControlSystem<RigidBodySystem>>(
      rigid_body_sys, Kp, Kd);

  auto vehicle_sys = cascade(
      std::make_shared<
          Gain<DrivingCommand, PDControlSystem<RigidBodySystem>::InputVector>>(
          map_driving_cmd_to_x_d),
      vehicle_with_pd);

  return vehicle_sys;
}

void SetSimulationOptions(SimulationOptions* sim_options,
    double initial_step_size, double timeout_seconds) {
  if (sim_options == nullptr)
    throw std::runtime_error("ERROR: Simulation options are null!");
  *sim_options = Drake::default_simulation_options;
  sim_options->initial_step_size = initial_step_size;
  sim_options->timeout_seconds = timeout_seconds;
}

VectorXd GetInitialState(std::shared_ptr<RigidBodySystem> rigid_body_sys) {
  auto const& tree = rigid_body_sys->getRigidBodyTree();

  VectorXd x0 = VectorXd::Zero(rigid_body_sys->getNumStates());
  x0.head(tree->number_of_positions()) = tree->getZeroConfiguration();
  return x0;
}

}  // namespace drake

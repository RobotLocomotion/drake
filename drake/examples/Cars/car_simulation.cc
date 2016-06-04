#include "drake/examples/Cars/car_simulation.h"

#include <cstdlib>

#include "drake/systems/plants/collision/collision_element.h"

using Eigen::MatrixXd;
using Eigen::Matrix;
using Eigen::VectorXd;

namespace drake {
namespace examples {
namespace cars {

std::shared_ptr<RigidBodySystem> CreateRigidBodySystem(int argc,
                                                       const char* argv[],
                                                       double* duration) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " vehicle_model [world sdf files ...]"
              << " --duration [duration in seconds]" << std::endl;
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
  auto rigid_body_sys = std::allocate_shared<RigidBodySystem>(
      Eigen::aligned_allocator<RigidBodySystem>());
  rigid_body_sys->addRobotFromFile(argv[1], DrakeJoint::QUATERNION,
                                   weld_to_frame);

  // Initializes duration to be infinity.
  *duration = std::numeric_limits<double>::infinity();

  // Adds the environment to the rigid body tree.
  const auto& tree = rigid_body_sys->getRigidBodyTree();
  for (int i = 2; i < argc; i++) {
    if (std::string(argv[i]) == "--duration") {
      if (++i == argc) {
        throw std::runtime_error(
            "ERROR: Command line option \"--duration\" is not followed by a "
            "value!");
      }
      *duration = atof(argv[i]);
    } else {
      tree->addRobotFromSDF(argv[i], DrakeJoint::FIXED);
    }
  }

  // If no environment is specified, the following code adds a flat terrain.
  if (argc < 3) {
    AddFlatTerrain(tree);
  }

  SetRigidBodySystemParameters(rigid_body_sys.get());

  return rigid_body_sys;
}

void SetRigidBodySystemParameters(RigidBodySystem* rigid_body_sys) {
  rigid_body_sys->penetration_stiffness = 5000.0;
  rigid_body_sys->penetration_damping =
    rigid_body_sys->penetration_stiffness / 10.0;
  rigid_body_sys->friction_coefficient = 10.0;  // essentially infinite friction
}

double ParseDuration(int argc, const char* argv[]) {
  for (int ii = 1; ii < argc; ii++) {
    if (std::string(argv[ii]) == "--duration") {
      if (++ii == argc) {
        throw std::runtime_error(
            "ERROR: Command line option \"--duration\" is not followed by a "
            "value!");
      }
      return atof(argv[ii]);
    }
  }
  return std::numeric_limits<double>::infinity();
}

void AddFlatTerrain(const std::shared_ptr<RigidBodyTree>& rigid_body_tree,
    double box_width, double box_depth) {
  DrakeShapes::Box geom(Eigen::Vector3d(box_width, box_width, box_depth));
  Eigen::Isometry3d T_element_to_link = Eigen::Isometry3d::Identity();
  T_element_to_link.translation() << 0, 0,
      -box_depth / 2;  // top of the box is at z=0
  RigidBody& world = rigid_body_tree->world();
  Eigen::Vector4d color;
  color << 0.9297, 0.7930, 0.6758,
      1;  // was hex2dec({'ee','cb','ad'})'/256 in matlab
  world.addVisualElement(
      DrakeShapes::VisualElement(geom, T_element_to_link, color));
  rigid_body_tree->addCollisionElement(
      RigidBody::CollisionElement(geom, T_element_to_link, &world), world,
      "terrain");
  rigid_body_tree->updateStaticCollisionElements();
}

std::shared_ptr<CascadeSystem<
    Gain<DrivingCommand, PDControlSystem<RigidBodySystem>::InputVector>,
    PDControlSystem<RigidBodySystem>>>
CreateVehicleSystem(std::shared_ptr<RigidBodySystem> rigid_body_sys) {
  const auto& tree = rigid_body_sys->getRigidBodyTree();

  // Sets up PD controllers for throttle and steering.
  const double kpSteering = 400, kdSteering = 80, kThrottle = 100;

  MatrixXd Kp(getNumInputs(*rigid_body_sys), tree->number_of_positions());
  Kp.setZero();

  MatrixXd Kd(getNumInputs(*rigid_body_sys), tree->number_of_velocities());
  Kd.setZero();

  Matrix<double, Eigen::Dynamic, 3> map_driving_cmd_to_x_d(
      tree->number_of_positions() + tree->number_of_velocities(), 3);
  map_driving_cmd_to_x_d.setZero();

  for (int actuator_idx = 0; actuator_idx < tree->actuators.size();
       actuator_idx++) {
    const std::string& actuator_name = tree->actuators[actuator_idx].name;

    if (actuator_name == "steering") {
      // Obtains the rigid body to which the actuator is attached.
      const auto& rigid_body = tree->actuators[actuator_idx].body;

      // Sets the steering actuator's Kp gain.
      Kp(actuator_idx, rigid_body->position_num_start) = kpSteering;

      // Sets the steering actuator's Kd gain.
      Kd(actuator_idx, rigid_body->velocity_num_start) = kdSteering;

      // Saves the mapping between the driving command and the steering command.
      map_driving_cmd_to_x_d(rigid_body->position_num_start,
                             DrivingCommandIndices::kSteeringAngle) =
          1;  // steering command

    } else if (actuator_name == "right_wheel_joint" ||
               actuator_name == "left_wheel_joint") {
      // Obtains the rigid body to which the actuator is attached.
      const auto& rigid_body = tree->actuators[actuator_idx].body;

      // Sets the throttle Kd gain.
      Kd(actuator_idx, rigid_body->velocity_num_start) = kThrottle;

      // Saves the mapping between the driving command and the throttle command.
      map_driving_cmd_to_x_d(
          tree->number_of_positions() + rigid_body->velocity_num_start,
          DrivingCommandIndices::kThrottle) = 20;

      // Saves the mapping between the driving command and the braking command.
      map_driving_cmd_to_x_d(
          tree->number_of_positions() + rigid_body->velocity_num_start,
          DrivingCommandIndices::kBrake) = -20;
    }
  }

  auto vehicle_with_pd = std::allocate_shared<PDControlSystem<RigidBodySystem>>(
      Eigen::aligned_allocator<PDControlSystem<RigidBodySystem>>(),
      rigid_body_sys, Kp, Kd);

  auto vehicle_sys = cascade(
      std::allocate_shared<
          Gain<DrivingCommand, PDControlSystem<RigidBodySystem>::InputVector>>(
          Eigen::aligned_allocator<Gain<
              DrivingCommand, PDControlSystem<RigidBodySystem>::InputVector>>(),
          map_driving_cmd_to_x_d),
      vehicle_with_pd);

  return vehicle_sys;
}

SimulationOptions GetCarSimulationDefaultOptions() {
  SimulationOptions result = Drake::default_simulation_options;
  result.initial_step_size = 5e-3;
  result.timeout_seconds = std::numeric_limits<double>::infinity();
  return result;
}

VectorXd GetInitialState(const RigidBodySystem& rigid_body_sys) {
  const auto& tree = rigid_body_sys.getRigidBodyTree();

  VectorXd x0 = VectorXd::Zero(rigid_body_sys.getNumStates());
  x0.head(tree->number_of_positions()) = tree->getZeroConfiguration();
  return x0;
}

}  // namespace cars
}  // namespace examples
}  // namespace drake

#include "drake/automotive/car_simulation.h"

#include <cstdlib>
#include <limits>

#include "drake/systems/plants/joints/floating_base_types.h"
#include "drake/systems/plants/parser_model_instance_id_table.h"

namespace drake {
namespace automotive {

using Eigen::Matrix;
using Eigen::MatrixXd;
using Eigen::VectorXd;

using drake::NullVector;
using drake::parsers::ModelInstanceIdTable;
using drake::systems::plants::joints::kFixed;
using drake::systems::plants::joints::kQuaternion;

const char kDurationFlag[] = "--duration";

void PrintUsageInstructions(const std::string& executable_name) {
  std::cout
    << "Usage: " << executable_name
    << " vehicle_model_file [world_model_files] ["
    << kDurationFlag << " duration_in_seconds]"
    << std::endl
    << std::endl
    << "Where:" << std::endl
    << "  - vehicle_model_file is the path to the URDF or SDF file defining"
    << std::endl
    << "    the vehicle model(s) and are thus attached to the world via"
    << std::endl
    << "    kQuaternion joints."
    << std::endl
    << std::endl
    << "  - world_model_files is a space-separated list of paths to URDF or"
    << std::endl
    << "    SDF files. This list can be of length zero or more. The models"
    << std::endl
    << "    within these files are connected to the world via kFixed"
    << std::endl
    << "    joints."
    << std::endl
    << std::endl
    << "  - duration_in_seconds is the number of seconds (floating point) to"
    << std::endl
    << "    run the simulation. This value is in simulation time."
    << std::endl;
}

std::shared_ptr<RigidBodySystem> CreateRigidBodySystem(
    int argc, const char* argv[], double* duration,
    ModelInstanceIdTable* model_instance_id_table) {
  if (argc < 2) {
    PrintUsageInstructions(argv[0]);
    exit(EXIT_FAILURE);
  }

  // Instantiates a rigid body system.
  auto rigid_body_sys = std::allocate_shared<RigidBodySystem>(
      Eigen::aligned_allocator<RigidBodySystem>());

  // Adds a model instance.
  ModelInstanceIdTable vehicle_instance_id_table =
      rigid_body_sys->AddModelInstanceFromFile(argv[1],
          kQuaternion);

  // Verifies that only one vehicle was added to the world.
  if (vehicle_instance_id_table.size() != 1) {
    throw std::runtime_error(
        "More than one vehicle model was added to the world.");
  }

  // Saves the vehicle model into the model_instance_id_table output parameter.
  *model_instance_id_table = vehicle_instance_id_table;

  if (duration != nullptr) {
    // Initializes duration to be infinity.
    *duration = std::numeric_limits<double>::infinity();
  }

  // Adds the environment.
  for (int ii = 2; ii < argc; ++ii) {
    if (std::string(argv[ii]) == "--duration") {
      if (++ii == argc) {
        throw std::runtime_error(
            "ERROR: Command line option \"--duration\" is not followed by a "
            "value!");
      }
      if (duration != nullptr)
        *duration = atof(argv[ii]);
    } else {
      ModelInstanceIdTable world_instance_id_table =
          rigid_body_sys->AddModelInstanceFromFile(argv[ii], kFixed);
      drake::parsers::AddModelInstancesToTable(world_instance_id_table,
          model_instance_id_table);
    }
  }

  // Adds a flat terrain if no environment is specified.
  if (argc < 3) {
    const std::shared_ptr<RigidBodyTree>& tree =
        rigid_body_sys->getRigidBodyTree();
    AddFlatTerrainToWorld(tree);
  }

  // Sets various simulation parameters.
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
  for (int ii = 1; ii < argc; ++ii) {
    if (std::string(argv[ii]) == kDurationFlag) {
      if (++ii == argc) {
        PrintUsageInstructions(argv[0]);
        throw std::runtime_error(
            "ERROR: Command line option \"" + std::string(kDurationFlag) +
            "\" is not followed by a value!");
      }
      return atof(argv[ii]);
    }
  }
  return std::numeric_limits<double>::infinity();
}

void AddFlatTerrainToWorld(
    const std::shared_ptr<RigidBodyTree>& rigid_body_tree,
    double box_size, double box_depth) {
  DrakeShapes::Box geom(Eigen::Vector3d(box_size, box_size, box_depth));
  Eigen::Isometry3d T_element_to_link = Eigen::Isometry3d::Identity();
  T_element_to_link.translation() << 0, 0,
      -box_depth / 2;  // top of the box is at z=0
  RigidBody& world = rigid_body_tree->world();
  Eigen::Vector4d color;
  color << 0.9297, 0.7930, 0.6758,
      1;  // was hex2dec({'ee','cb','ad'})'/256 in matlab
  world.AddVisualElement(
      DrakeShapes::VisualElement(geom, T_element_to_link, color));
  rigid_body_tree->addCollisionElement(
      DrakeCollision::Element(geom, T_element_to_link, &world), world,
      "terrain");
  rigid_body_tree->updateStaticCollisionElements();
}

std::shared_ptr<CascadeSystem<
    Gain<DrivingCommand1, PDControlSystem<RigidBodySystem>::InputVector>,
    PDControlSystem<RigidBodySystem>>>
CreateVehicleSystem(std::shared_ptr<RigidBodySystem> rigid_body_sys) {
  const auto& tree = rigid_body_sys->getRigidBodyTree();

  // Sets up PD controllers for throttle and steering.
  const double kpSteering = 400, kdSteering = 80, kThrottle = 100;

  MatrixXd Kp(getNumInputs(*rigid_body_sys), tree->get_num_positions());
  Kp.setZero();

  MatrixXd Kd(getNumInputs(*rigid_body_sys), tree->get_num_velocities());
  Kd.setZero();

  Matrix<double, Eigen::Dynamic, 3> map_driving_cmd_to_x_d(
      tree->get_num_positions() + tree->get_num_velocities(), 3);
  map_driving_cmd_to_x_d.setZero();

  for (int actuator_idx = 0;
       actuator_idx < static_cast<int>(tree->actuators.size());
       actuator_idx++) {
    const std::string& actuator_name = tree->actuators[actuator_idx].name_;

    if (actuator_name == "steering") {
      // Obtains the rigid body to which the actuator is attached.
      const auto& rigid_body = tree->actuators[actuator_idx].body_;

      // Sets the steering actuator's Kp gain.
      Kp(actuator_idx, rigid_body->get_position_start_index()) = kpSteering;

      // Sets the steering actuator's Kd gain.
      Kd(actuator_idx, rigid_body->get_velocity_start_index()) = kdSteering;

      // Saves the mapping between the driving command and the steering command.
      map_driving_cmd_to_x_d(rigid_body->get_position_start_index(),
                             DrivingCommandIndices::kSteeringAngle) =
          1;  // steering command

    } else if (actuator_name == "right_wheel_joint" ||
               actuator_name == "left_wheel_joint") {
      // Obtains the rigid body to which the actuator is attached.
      const auto& rigid_body = tree->actuators[actuator_idx].body_;

      // Sets the throttle Kd gain.
      Kd(actuator_idx, rigid_body->get_velocity_start_index()) = kThrottle;

      // Saves the mapping between the driving command and the throttle command.
      map_driving_cmd_to_x_d(
          tree->get_num_positions() + rigid_body->get_velocity_start_index(),
          DrivingCommandIndices::kThrottle) = 20;

      // Saves the mapping between the driving command and the braking command.
      map_driving_cmd_to_x_d(
          tree->get_num_positions() + rigid_body->get_velocity_start_index(),
          DrivingCommandIndices::kBrake) = -20;
    }
  }

  auto vehicle_with_pd = std::allocate_shared<PDControlSystem<RigidBodySystem>>(
      Eigen::aligned_allocator<PDControlSystem<RigidBodySystem>>(),
      rigid_body_sys, Kp, Kd);

  auto vehicle_sys = cascade(
      std::allocate_shared<
          Gain<DrivingCommand1, PDControlSystem<RigidBodySystem>::InputVector>>(
          Eigen::aligned_allocator<
              Gain<DrivingCommand1,
                   PDControlSystem<RigidBodySystem>::InputVector>>(),
          map_driving_cmd_to_x_d),
      vehicle_with_pd);

  return vehicle_sys;
}

SimulationOptions GetCarSimulationDefaultOptions() {
  SimulationOptions result;
  result.initial_step_size = 5e-3;
  result.timeout_seconds = std::numeric_limits<double>::infinity();
  return result;
}

VectorXd GetInitialState(const RigidBodySystem& rigid_body_sys) {
  const auto& tree = rigid_body_sys.getRigidBodyTree();

  VectorXd x0 = VectorXd::Zero(rigid_body_sys.getNumStates());
  x0.head(tree->get_num_positions()) = tree->getZeroConfiguration();
  return x0;
}

}  // namespace automotive
}  // namespace drake

#include "drake/examples/Cars/car_simulation.h"

#include <cstdlib>

#include "drake/examples/Cars/curve2.h"
#include "drake/examples/Cars/trajectory_car.h"
#include "drake/systems/plants/parser_model_instance_id_table.h"

using drake::AffineSystem;
using drake::NullVector;
using drake::parsers::ModelInstanceIdTable;

using Eigen::Matrix;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using drake::parsers::ModelInstanceIdTable;

namespace drake {
namespace examples {
namespace cars {

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
    << "    DrakeJoint::QUATERNION joints."
    << std::endl
    << std::endl
    << "  - world_model_files is a space-separated list of paths to URDF or"
    << std::endl
    << "    SDF files. This list can be of length zero or more. The models"
    << std::endl
    << "    within these files are connected to the world via DrakeJoint::FIXED"
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
          DrakeJoint::QUATERNION);

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
          rigid_body_sys->AddModelInstanceFromFile(argv[ii], DrakeJoint::FIXED);
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
      RigidBodyCollisionElement(geom, T_element_to_link, &world), world,
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

  MatrixXd Kp(getNumInputs(*rigid_body_sys), tree->number_of_positions());
  Kp.setZero();

  MatrixXd Kd(getNumInputs(*rigid_body_sys), tree->number_of_velocities());
  Kd.setZero();

  // Computes the length of x_d, the desired state of each actuator DOF.
  int length_of_x_d =
      tree->number_of_positions() + tree->number_of_velocities();

  // Instantiates a N x 3 matrix called map_driving_cmd_to_x_d where N is the
  // number position and velocity states in the rigid body tree. The three
  // columns are defined by drake::DrivingCommandIndices
  // (see: drake/examples/Cars/car_simulation.h) to be as follows:
  //
  //  Column Index     Meaning
  //      0            gain of Steering angle
  //      1            gain of Throttle
  //      2            gain of Brake
  //
  Matrix<double, Eigen::Dynamic, 3> map_driving_cmd_to_x_d(length_of_x_d, 3);
  map_driving_cmd_to_x_d.setZero();

  for (int actuator_idx = 0;
       actuator_idx < static_cast<int>(tree->actuators.size());
       actuator_idx++) {
    // Obtains the actuator's name.
    const std::string& actuator_name = tree->actuators[actuator_idx].name_;

    // Obtains the rigid body to which the actuator is attached.
    const auto& rigid_body = tree->actuators[actuator_idx].body_;

    if (actuator_name == "steering") {
      // Sets the steering actuator's Kp gain.
      Kp(actuator_idx, rigid_body->get_position_start_index()) = kpSteering;

      // Sets the steering actuator's Kd gain.
      Kd(actuator_idx, rigid_body->get_velocity_start_index()) = kdSteering;

      // Saves the mapping between the driving command and the steering command.
      map_driving_cmd_to_x_d(
          rigid_body->get_position_start_index(),
          DrivingCommandIndices::kSteeringAngle) =
          1;  // steering command

    } else if (actuator_name == "right_wheel_joint" ||
               actuator_name == "left_wheel_joint") {
      // Sets the throttle Kd gain.
      Kd(actuator_idx, rigid_body->get_velocity_start_index()) = kThrottle;

      // Saves the mapping between the driving command and the throttle command.
      map_driving_cmd_to_x_d(
          tree->number_of_positions() + rigid_body->get_velocity_start_index(),
          DrivingCommandIndices::kThrottle) = 20;

      // Saves the mapping between the driving command and the braking command.
      map_driving_cmd_to_x_d(
          tree->number_of_positions() + rigid_body->get_velocity_start_index(),
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

std::shared_ptr<CascadeSystem<
    Gain<MultiDrivingCommand1, PDControlSystem<RigidBodySystem>::InputVector>,
    PDControlSystem<RigidBodySystem>>>
CreateMultiVehicleSystem(std::shared_ptr<RigidBodySystem> rigid_body_sys,
    const std::map<int, std::string>* model_instance_name_table) {
  const auto& tree = rigid_body_sys->getRigidBodyTree();

  // Sets up PD controllers for throttle and steering.
  const double kpSteering = 400, kdSteering = 80, kThrottle = 100;

  MatrixXd Kp(getNumInputs(*rigid_body_sys), tree->number_of_positions());
  Kp.setZero();

  MatrixXd Kd(getNumInputs(*rigid_body_sys), tree->number_of_velocities());
  Kd.setZero();

  // Computes the number of columns in the map_driving_cmd_to_x_d matrix.
  // Since each vehicle has 3 commands (steering, throttle, and brake), the
  // number of columns should be 3 times the number of vehicles.
  int number_of_driving_commands = 3 * model_instance_name_table->size();

  // Computes the length of x_d, the desired state of each actuator DOF.
  int length_of_x_d =
      tree->number_of_positions() + tree->number_of_velocities();

  // Instantiates a N x number_of_driving_commands matrix called
  // map_driving_cmd_to_x_d where N is the number position and velocity states
  // in the rigid body tree. The number_of_driving_commands is determined by
  // the number of commands per vehicle (three as described below) multipled
  // by the number of vehicles:
  //
  //      Column Index                 Meaning
  //      vehicle number * 3           gain of Steering angle
  //      vehicle number * 3 + 1       gain of Throttle
  //      vehicle number * 3 + 2       gain of Brake
  //
  MatrixX<double> map_driving_cmd_to_x_d(length_of_x_d,
                                         number_of_driving_commands);
  map_driving_cmd_to_x_d.setZero();

  // Obtains the smallest model instance ID from model_instance_name_table.
  // This is used to compute the starting column index in
  // map_driving_cmd_to_x_d for a particular vehicle model instance.
  int smallest_instance_id = std::numeric_limits<int>::max();
  for (const auto& iterator : *model_instance_name_table) {
    if (iterator.first < smallest_instance_id)
      smallest_instance_id = iterator.first;
  }

  for (int actuator_idx = 0;
       actuator_idx < static_cast<int>(tree->actuators.size());
       actuator_idx++) {
    // Obtains the actuator's name.
    const std::string& actuator_name = tree->actuators[actuator_idx].name_;

    // Obtains the rigid body to which the actuator is attached.
    const auto& rigid_body = tree->actuators[actuator_idx].body_;

    // TODO(liang.fok): This is brittle to the model instance ID and the
    // presence of other models within the world. I'm not sure how to avoid this
    // problem at this moment in time.
    int column_starting_index =
      (rigid_body->get_model_instance_id() - smallest_instance_id) * 3;

    if (actuator_name == "steering") {
      // Sets the steering actuator's Kp gain.
      Kp(actuator_idx, rigid_body->get_position_start_index()) = kpSteering;

      // Sets the steering actuator's Kd gain.
      Kd(actuator_idx, rigid_body->get_velocity_start_index()) = kdSteering;

      // Saves the mapping between the driving command and the steering command.
      map_driving_cmd_to_x_d(
          rigid_body->get_position_start_index(),
          column_starting_index + DrivingCommandIndices::kSteeringAngle) =
          1;  // steering command

    } else if (actuator_name == "right_wheel_joint" ||
               actuator_name == "left_wheel_joint") {
      // Sets the throttle Kd gain.
      Kd(actuator_idx, rigid_body->get_velocity_start_index()) = kThrottle;

      // Saves the mapping between the driving command and the throttle command.
      map_driving_cmd_to_x_d(
          tree->number_of_positions() + rigid_body->get_velocity_start_index(),
          column_starting_index + DrivingCommandIndices::kThrottle) = 20;

      // Saves the mapping between the driving command and the braking command.
      map_driving_cmd_to_x_d(
          tree->number_of_positions() + rigid_body->get_velocity_start_index(),
          column_starting_index + DrivingCommandIndices::kBrake) = -20;
    }
  }

  auto vehicle_with_pd = std::allocate_shared<PDControlSystem<RigidBodySystem>>(
      Eigen::aligned_allocator<PDControlSystem<RigidBodySystem>>(),
      rigid_body_sys, Kp, Kd);

  auto vehicle_sys = cascade(
      std::allocate_shared<
          Gain<MultiDrivingCommand1,
              PDControlSystem<RigidBodySystem>::InputVector>>(
            Eigen::aligned_allocator<
                Gain<MultiDrivingCommand1,
                PDControlSystem<RigidBodySystem>::InputVector>>(),
            map_driving_cmd_to_x_d),
      vehicle_with_pd);

  return vehicle_sys;
}

namespace {
// A figure-eight.  One loop has a radius of @p radius - @p inset,
// the other loop has a radius of @p radius + @p inset.
Curve2<double> MakeCurve(double radius, double inset) {
  // TODO(jwnimmer-tri) This function will be rewritten once we have
  // proper splines.  Don't try too hard to understand it.  Run the
  // demo to see it first, and only then try to understand the code.

  typedef Curve2<double>::Point2 Point2d;
  std::vector<Point2d> waypoints;

  // Start (0, +i).
  // Straight right to (+r, +i).
  // Loop around (+i, +r).
  // Straight back to (+i, 0).
  waypoints.push_back({0.0, inset});
  for (int theta_deg = -90; theta_deg <= 180; ++theta_deg) {
    const Point2d center{radius, radius};
    const double theta = theta_deg * M_PI / 180.0;
    const Point2d direction{std::cos(theta), std::sin(theta)};
    waypoints.push_back(center + (direction * (radius - inset)));
  }
  waypoints.push_back({inset, 0.0});

  // Start (+i, 0).
  // Straight down to (+i, -r).
  // Loop around (-r, +i).
  // Straight back to start (implicitly via segment to waypoints[0]).
  for (int theta_deg = 0; theta_deg >= -270; --theta_deg) {
    const Point2d center{-radius, -radius};
    const double theta = theta_deg * M_PI / 180.0;
    const Point2d direction{std::cos(theta), std::sin(theta)};
    waypoints.push_back(center + (direction * (radius + inset)));
  }

  // Many copies.
  const int kNumCopies = 100;
  std::vector<Point2d> looped_waypoints;
  for (int copies = 0; copies < kNumCopies; ++copies) {
    std::copy(waypoints.begin(), waypoints.end(),
              std::back_inserter(looped_waypoints));
  }
  looped_waypoints.push_back(waypoints.front());

  return Curve2<double>(looped_waypoints);
}
}  // namespace anonymous

std::shared_ptr<TrajectoryCar1> CreateTrajectoryCarSystem(int index) {
  // The possible curves to trace (lanes).
  const std::vector<Curve2<double>> curves{
    MakeCurve(40.0, 0.0),  // BR
    MakeCurve(40.0, 4.0),  // BR
    MakeCurve(40.0, 8.0),
  };

  // Magic car placement to make a good visual demo.
  const auto& curve = curves[index % curves.size()];
  const double start_time = (index / curves.size()) * 0.8;
  const double kSpeed = 8.0;
  return std::make_shared<TrajectoryCar1>(curve, kSpeed, start_time);
}

std::shared_ptr<
    AffineSystem<NullVector, SimpleCarState1, EulerFloatingJointState1>>
CreateSimpleCarVisualizationAdapter() {
  const int insize = SimpleCarState1<double>().size();
  const int outsize = EulerFloatingJointState1<double>().size();
  MatrixXd D;
  D.setZero(outsize, insize);
  D(EulerFloatingJointStateIndices::kX, SimpleCarStateIndices::kX) = 1;
  D(EulerFloatingJointStateIndices::kY, SimpleCarStateIndices::kY) = 1;
  D(EulerFloatingJointStateIndices::kYaw, SimpleCarStateIndices::kHeading) = 1;
  EulerFloatingJointState1<double> y0;
  return std::make_shared<
    AffineSystem<
        NullVector,
        SimpleCarState1,
        EulerFloatingJointState1>>(
            MatrixXd::Zero(0, 0),
            MatrixXd::Zero(0, insize),
            VectorXd::Zero(0),
            MatrixXd::Zero(outsize, 0),
            D, toEigen(y0));
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
  x0.head(tree->number_of_positions()) = tree->getZeroConfiguration();
  return x0;
}

}  // namespace cars
}  // namespace examples
}  // namespace drake

#pragma once

#include <memory>
#include <string>

#include <Eigen/Geometry>

#include "drake/systems/LinearSystem.h"
#include "drake/systems/Simulation.h"
#include "drake/systems/pd_control_system.h"
#include "drake/systems/plants/RigidBodySystem.h"

#include "drake/examples/Cars/piecewise_linear_curve.h"

#include <iostream>
#define PRINT_VAR(x) std::cout <<  #x ": " << x << std::endl;

using Drake::RigidBodySystem;
using Drake::PDControlSystem;
using Drake::SimulationOptions;

using Eigen::MatrixXd;
using Eigen::Matrix;
using Eigen::VectorXd;
using Eigen::Vector3d;
using Eigen::Isometry3d;

namespace drake {
namespace examples {
namespace cars {

class SelfDrivingCar {
 public:
  template <typename T>
  using InputVector = Drake::NullVector<T>;
  template <typename T>
  using StateVector = Drake::RigidBodySystem::StateVector<T>;
  template <typename T>
  using OutputVector = Drake::RigidBodySystem::OutputVector<T>;

  SelfDrivingCar(int argc, const char* argv[]){
    if (argc < 2) {
      std::cerr << "Usage: " << argv[0] << " vehicle_model [world sdf files ...]"
          << " --duration [duration in seconds]" << std::endl;
      exit(EXIT_FAILURE);
    }

    // The Z-axis offset between Drake's world frame and the vehicle's world
    // frame.
    double z_offset = 0;

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
    rb_sys_ = std::allocate_shared<RigidBodySystem>(
        Eigen::aligned_allocator<RigidBodySystem>());
    rb_sys_->addRobotFromFile(argv[1], DrakeJoint::QUATERNION,
                                     weld_to_frame);
    const auto& tree = rb_sys_->getRigidBodyTree();

    // Adds the environment to the rigid body tree.
    //double box_width = 1000;
    //double box_depth = 10;
    DrakeShapes::Mesh geom("DrivingSurface_simpler_allTris.obj", "/home/amcastro/Documents/NvidiaTerrain/PaloAltoLoop.obj");
    Eigen::Isometry3d T_element_to_link = Eigen::Isometry3d::Identity();
    T_element_to_link.linear() = Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitX()).toRotationMatrix();
    //DrakeShapes::Box geom(Eigen::Vector3d(box_width, box_width, box_depth));
    //Eigen::Isometry3d T_element_to_link = Eigen::Isometry3d::Identity();
    //T_element_to_link.translation() << 0, 0,
    //    -box_depth / 2;  // top of the box is at z=0
    RigidBody& world = tree->world();
    Eigen::Vector4d color;
    color << 0.9297, 0.7930, 0.6758,
        1;  // was hex2dec({'ee','cb','ad'})'/256 in matlab
    world.addVisualElement(
        DrakeShapes::VisualElement(geom, T_element_to_link, color));
    RigidBody::CollisionElement
        colliding_world(geom, T_element_to_link, &world);
    colliding_world.set_static();
    tree->addCollisionElement(
        colliding_world, world,
        "terrain");
    tree->updateStaticCollisionElements();


    rb_sys_->penetration_stiffness = 5000.0;
    rb_sys_->penetration_damping =
        rb_sys_->penetration_stiffness / 10.0;
    rb_sys_->friction_coefficient = 10.0;  // essentially infinite friction

    CreateSystemWithPD();

    chassis_body = tree->findLink("chassis_floor");
    car_position_dof = chassis_body->position_num_start;

    route_.set_from_file("/home/amcastro/Documents/NvidiaTerrain/full_trajectory.dat");
    current_curve_index = 0;

    PRINT_VAR(route_.number_of_points());
    PRINT_VAR(car_position_dof);
  }

  StateVector<double> dynamics(const double& t,
                               const StateVector<double>& x,
                               const InputVector<double>& u) const {
    //VectorXd u_rb_sys = VectorXd::Zero(rb_sys_->getNumInputs());
    //return rb_sys_->dynamics(t, x, u_rb_sys);
    VectorXd target_state = VectorXd::Zero(rb_sys_->getNumOutputs());

    // Set wheels' angular velocity.
    target_state(right_wheel_dof) = 40;
    target_state(left_wheel_dof) = 40;

    // Extract the car's position.
    //Matrix<double, 3, 1> pos = x.segment<3>(car_position_dof);
    //pos = pos + Vector3d(18.0, 99.0, 4.8+0.378326); // + car initial position.

    auto& tree = rb_sys_->getRigidBodyTree();

    auto nq = tree->number_of_positions();
    auto nv = tree->number_of_velocities();
    auto q = x.topRows(nq);
    auto v = x.bottomRows(nv);
    auto cache = tree->doKinematics(q, v);
    Isometry3d Tb = cache.getElement(*chassis_body).transform_to_world;

    // The position of the car.
    Vector3d pos = Tb.translation();

    Vector3d closest_point;

    int curve_index;
    double distance_along_curve = route_.ComputeClosestPoint(
        pos, closest_point, curve_index);

    // Look at the curve ahead.
    Vector3d look_at = route_.ComputePointAt(
        distance_along_curve-5.0);
    Vector3d direction = look_at - pos;

    PRINT_VAR(distance_along_curve);
    PRINT_VAR(pos.transpose());
    PRINT_VAR(closest_point.transpose());
    PRINT_VAR(look_at);
    PRINT_VAR(direction.transpose());

    // Transform to car's coordinates
    Vector3d direction_car_frame = Tb.linear().transpose() * direction;

    PRINT_VAR(direction_car_frame.transpose());

    // Set the target steering angle.
    target_state(steering_dof) = direction_car_frame.y()/10.0;

    return vehicle_with_pd->dynamics(t, x, target_state);
  }

  StateVector<double> output(const double& t,
                             const StateVector<double>& x,
                             const InputVector<double>& u) const {
    return x;
  }

  bool isTimeVarying() const { return false; }
  bool isDirectFeedthrough() const { return false; }

  const std::shared_ptr<RigidBodyTree>& getRigidBodyTree() const {
    return rb_sys_->getRigidBodyTree();
  }

  const std::shared_ptr<RigidBodySystem>& get_rigid_body_system() const {
    return rb_sys_;
  }

  VectorXd ComputeInitialState() {
    const auto& tree = rb_sys_->getRigidBodyTree();

    VectorXd x0 = VectorXd::Zero(rb_sys_->getNumStates());
    x0.head(tree->number_of_positions()) = tree->getZeroConfiguration();
    return x0;
  }

 private:
  std::shared_ptr<RigidBodySystem> rb_sys_;
  std::shared_ptr<PDControlSystem<RigidBodySystem>> vehicle_with_pd;
  int right_wheel_dof, left_wheel_dof, steering_dof, car_position_dof;
  PieceWiseLinearCurve route_;
  RigidBody* chassis_body;
  int current_curve_index;
  double distance_along_curve;

  void CreateSystemWithPD() {
    const auto& tree = rb_sys_->getRigidBodyTree();

    // Sets up PD controllers for throttle and steering.
    //const double kpSteering = 400, kdSteering = 80, kThrottle = 100;
    const double kpSteering = 800, kdSteering = 0, kThrottle = 5;

    MatrixXd Kp(getNumInputs(*rb_sys_), tree->number_of_positions());
    Kp.setZero();

    MatrixXd Kd(getNumInputs(*rb_sys_), tree->number_of_velocities());
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
        steering_dof = rigid_body->position_num_start;

        // Sets the steering actuator's Kp gain.
        Kp(actuator_idx, rigid_body->position_num_start) = kpSteering;

        // Sets the steering actuator's Kd gain.
        Kd(actuator_idx, rigid_body->velocity_num_start) = kdSteering;

      } else if (actuator_name == "right_wheel_joint" ||
          actuator_name == "left_wheel_joint") {
        // Obtains the rigid body to which the actuator is attached.
        const auto& rigid_body = tree->actuators[actuator_idx].body;

        if(actuator_name == "right_wheel_joint")
          right_wheel_dof = tree->number_of_positions() + rigid_body->velocity_num_start;
        else
          left_wheel_dof = tree->number_of_positions() + rigid_body->velocity_num_start;

        // Sets the throttle Kd gain.
        Kd(actuator_idx, rigid_body->velocity_num_start) = kThrottle;

      }
    }

    vehicle_with_pd = std::allocate_shared<PDControlSystem<RigidBodySystem>>(
        Eigen::aligned_allocator<PDControlSystem<RigidBodySystem>>(),
        rb_sys_, Kp, Kd);

  }
};

}  // namespace cars
}  // namespace examples
}  // namespace drake


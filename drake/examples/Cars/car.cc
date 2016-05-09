#include "drake/examples/Cars/car.h"

#include <stdlib.h>

// bool decode(const drake::lcmt_driving_control_cmd_t& msg, double& t,
//             DrivingCommand<double>& x) {
//   t = double(msg.timestamp) / 1000.0;
//   x.steering_angle = msg.steering_angle;
//   x.throttle = msg.throttle_value;
//   x.brake = msg.brake_value;
//   return true;
// }


namespace drake {

std::shared_ptr<RigidBodySystem> CreateRigidBodySystem(int argc, char* argv[]) {
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
  rigid_body_sys->addRobotFromFile(argv[1], DrakeJoint::QUATERNION, weld_to_frame);

  // Adds the environment to the rigid body tree.
  auto const& tree = rigid_body_sys->getRigidBodyTree();
  for (int i = 2; i < argc; i++)
    tree->addRobotFromSDF(argv[i], DrakeJoint::FIXED);

  // If no environment was specified, adds a flat terrain.
  if (argc < 3) {
    double box_width = 1000;
    double box_depth = 10;
    DrakeShapes::Box geom(Eigen::Vector3d(box_width, box_width, box_depth));
    Eigen::Isometry3d T_element_to_link = Eigen::Isometry3d::Identity();
    T_element_to_link.translation() << 0, 0,
        -box_depth / 2;  // top of the box is at z=0
    auto& world = tree->bodies[0];
    Eigen::Vector4d color;
    color << 0.9297, 0.7930, 0.6758,
        1;  // was hex2dec({'ee','cb','ad'})'/256 in matlab
    world->addVisualElement(
        DrakeShapes::VisualElement(geom, T_element_to_link, color));
    tree->addCollisionElement(
        RigidBody::CollisionElement(geom, T_element_to_link, world), *world,
        "terrain");
    tree->updateStaticCollisionElements();
  }

  return rigid_body_sys;
}

}  // namespace drake

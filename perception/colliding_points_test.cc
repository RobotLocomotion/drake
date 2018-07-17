#include "drake/multibody/joints/roll_pitch_yaw_floating_joint.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"

namespace drake {
namespace perception {
namespace {

int doMain(int argc, char **argv) {
  auto tree_ptr = std::make_unique<RigidBodyTree<double>>();

  auto body = std::make_unique<RigidBody<double>>();

  Eigen::Isometry3d joint_transform;
  {
    const drake::math::RotationMatrix<double> kRotIdentity;
    joint_transform.matrix() << kRotIdentity.matrix(), Eigen::Vector3d::Zero(),
        0, 0, 0, 1;
  }
  auto joint =
      std::make_unique<RollPitchYawFloatingJoint>("box_joint", joint_transform);
  body->add_joint(&tree_ptr->world(), std::move(joint));

  RigidBody<double>* body_in_tree = tree_ptr->add_rigid_body(std::move(body));

  const Eigen::Vector3d size(0.25, 0.25, 0.25);
  const DrakeShapes::Box shape(size);
  drake::multibody::collision::Element body_collision(
      shape, Isometry3<double>::Identity());
  tree_ptr->addCollisionElement(body_collision, *body_in_tree, "default");

  tree_ptr->compile();

  std::vector<Eigen::Vector3d> points;
  points.resize(1);
  points[0] = Eigen::Vector3d(0., 0., 0.);

  Eigen::VectorXd q = tree_ptr->getZeroConfiguration();
  KinematicsCache<double> kinematics_cache = tree_ptr->doKinematics(q);

  std::vector<size_t> indices =
      tree_ptr->collidingPoints(kinematics_cache, points, 0.05);

  log()->info("indices: {}", indices.size());

  return 0;
}
}
}
}

int main(int argc, char **argv) {
  return drake::perception::doMain(argc, argv);
}

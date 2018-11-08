#include <random>

#include "drake/common/find_resource.h"
#include "drake/manipulation/dev/remote_tree_viewer_wrapper.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"

namespace drake {
namespace manipulation {
namespace dev {
int DoMain() {
  RemoteTreeViewerWrapper rm;

  int n_points = 1000;
  double x_size = 3.0;
  double y_size = 2.0;
  double z_size = 1.0;
  Eigen::Matrix3Xd pts(3, n_points);
  std::default_random_engine generator;
  std::uniform_real_distribution<double> distribution(-1, 1);
  for (int i = 0; i < n_points; i++) {
    Eigen::Vector3d xyz(distribution(generator), distribution(generator),
                        distribution(generator));
    xyz /= xyz.norm();
    xyz[0] *= x_size;
    xyz[1] *= y_size;
    xyz[2] *= z_size;
    pts.col(i) = xyz;
  }
  rm.PublishPointCloud(pts, {"test_pc"}, {{1, 0, 0}});
  rm.PublishLine(pts, {"test_line"});
  rm.PublishArrow(Eigen::Vector3d(0, 1, 0), Eigen::Vector3d(0, 1, 3),
                  {"test_arrow"}, 0.01, 0.05, 0.05);

  const Eigen::Vector4d color_gray(0.7, 0.7, 0.7, 0.9);
  const Eigen::Vector4d color_blue(0.3, 0.3, 1.0, 1.0);

  Eigen::Affine3d tf_mesh_pts;
  tf_mesh_pts.setIdentity();
  tf_mesh_pts.translation()(1) = 5;
  rm.PublishGeometry(DrakeShapes::MeshPoints(pts), tf_mesh_pts, color_gray,
                     {"test_mesh_points"});

  Eigen::Affine3d tf_box;
  tf_box.setIdentity();
  tf_box.translation()[1] = y_size * 2;
  rm.PublishGeometry(DrakeShapes::Box({x_size, y_size, z_size}), tf_box,
                     color_gray, {"test_box"});

  Eigen::Affine3d tf_sphere;
  tf_sphere.setIdentity();
  tf_sphere.translation()[1] = y_size * 2 * 2;
  rm.PublishGeometry(DrakeShapes::Sphere(y_size), tf_sphere, color_gray,
                     {"test_sphere"});

  Eigen::Affine3d tf_cylinder;
  tf_cylinder.setIdentity();
  tf_cylinder.translation()[1] = y_size * 2 * 3;
  rm.PublishGeometry(DrakeShapes::Cylinder(y_size, z_size), tf_cylinder,
                     color_gray, {"test_cylinder"});

  Eigen::Affine3d tf_capsule;
  tf_capsule.setIdentity();
  tf_capsule.translation()[1] = y_size * 2 * 4;
  rm.PublishGeometry(DrakeShapes::Capsule(y_size, z_size), tf_capsule,
                     color_gray, {"test_capsule"});
  RigidBodyTree<double> tree;
  const auto iiwa_path = drake::FindResourceOrThrow(
      "drake/examples/atlas/urdf/atlas_convex_hull.urdf");
  parsers::urdf::AddModelInstanceFromUrdfFileWithRpyJointToWorld(iiwa_path,
                                                                 &tree);
  Eigen::VectorXd q0 = tree.getZeroConfiguration();
  q0[1] = -y_size * 2;
  rm.PublishRigidBodyTree(tree, q0, color_blue, {"test_robot_visual"});
  rm.PublishRigidBodyTree(tree, q0, color_blue, {"test_robot_collision"},
                          false);

  Eigen::Affine3d tf_body;
  tf_body.setIdentity();
  tf_body.translation()[0] = 5;
  rm.PublishRigidBody(tree, 1, tf_body, color_blue, {"test_body_visual"}, true);
  rm.PublishRigidBody(tree, 1, tf_body, color_blue, {"test_body_collision"},
                      false);

  std::default_random_engine random_generator{0};
  Eigen::VectorXd q_random = tree.getRandomConfiguration(random_generator);
  q_random.head(6) = q0.head(6);
  rm.UpdateRigidBodyTree(tree, q_random, {"test_robot_visual"});
  q_random = tree.getRandomConfiguration(random_generator);
  q_random.head(6) = q0.head(6);
  q_random(0) = 2;
  rm.UpdateRigidBodyTree(tree, q_random, {"test_robot_collision"}, false);

  return 0;
}
}  // namespace dev
}  // namespace manipulation
}  // namespace drake

int main() { return drake::manipulation::dev::DoMain(); }

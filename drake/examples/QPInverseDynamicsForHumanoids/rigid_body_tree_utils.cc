#include "drake/examples/QPInverseDynamicsForHumanoids/rigid_body_tree_utils.h"

#include <vector>

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

Eigen::Vector6d GetTaskSpaceVel(const RigidBodyTree& r,
                                const KinematicsCache<double>& cache,
                                const RigidBody& body,
                                const Eigen::Vector3d& local_offset) {
  const auto& element = cache.getElement(body);
  Eigen::Vector3d pt = element.transform_to_world.translation();

  // Get the body's task space vel.
  // Converting from Plucker vector (Featherstone's spatial vectors) to spatial
  // vector algebra as defined by Abhinandan Jain.
  Eigen::Vector6d v = element.twist_in_world;
  v.tail<3>() += v.head<3>().cross(pt);

  // Get the global offset between pt and body.
  Eigen::Isometry3d H_pt_to_body(Eigen::Isometry3d::Identity());
  H_pt_to_body.translation() = local_offset;
  auto H_pt_to_world = element.transform_to_world * H_pt_to_body;
  Eigen::Vector3d world_offset =
      H_pt_to_world.translation() - element.transform_to_world.translation();

  // Add the linear vel from the body rotation.
  v.tail<3>() += v.head<3>().cross(world_offset);

  return v;
}

Eigen::MatrixXd GetTaskSpaceJacobian(const RigidBodyTree& r,
                                     const KinematicsCache<double>& cache,
                                     const RigidBody& body,
                                     const Eigen::Vector3d& local_offset) {
  std::vector<int> v_or_q_indices;
  Eigen::MatrixXd Jg = r.geometricJacobian(cache, 0, body.get_body_index(), 0,
                                           true, &v_or_q_indices);
  Eigen::MatrixXd J(6, r.get_num_velocities());
  J.setZero();

  Eigen::Vector3d points =
      r.transformPoints(cache, local_offset, body.get_body_index(), 0);

  int col = 0;
  for (auto it = v_or_q_indices.begin(); it != v_or_q_indices.end(); ++it) {
    // angular part
    J.template block<drake::kSpaceDimension, 1>(0, *it) =
        Jg.block<3, 1>(0, col);
    // linear part
    // Assuming joint velocity = 1, the column is the linear velocity.
    J.template block<drake::kSpaceDimension, 1>(3, *it) =
        Jg.block<3, 1>(3, col);
    J.template block<drake::kSpaceDimension, 1>(3, *it).noalias() +=
        Jg.block<3, 1>(0, col).cross(points);
    col++;
  }

  return J;
}

Eigen::Vector6d GetTaskSpaceJacobianDotTimesV(
    const RigidBodyTree& r, const KinematicsCache<double>& cache,
    const RigidBody& body, const Eigen::Vector3d& local_offset) {
  // position of point in world
  Eigen::Vector3d p =
      r.transformPoints(cache, local_offset, body.get_body_index(), 0);
  Eigen::Vector6d twist = r.relativeTwist(cache, 0, body.get_body_index(), 0);
  Eigen::Vector6d J_geometric_dot_times_v =
      r.geometricJacobianDotTimesV(cache, 0, body.get_body_index(), 0);

  // linear vel of r
  Eigen::Vector3d pdot = twist.head<3>().cross(p) + twist.tail<3>();

  // each column of Jt = [Jg_omega; Jg_v + Jg_omega.cross(p)]
  // for Jtdot * v, the angular part stays the same,
  // for the linear part:
  //  = [\dot{Jg_v} + \dot{Jg_omega}.cross(p) + Jg_omega.cross(pdot)] * v
  //  = [liner part of JgdotV + angular of JgdotV.cross(p) + omega.cross(pdot)]
  Eigen::Vector6d Jdv = J_geometric_dot_times_v;
  Jdv.tail<3>() +=
      twist.head<3>().cross(pdot) + J_geometric_dot_times_v.head<3>().cross(p);

  return Jdv;
}

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake

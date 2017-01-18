#include "drake/examples/QPInverseDynamicsForHumanoids/rigid_body_tree_utils.h"

#include <vector>

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

Vector6<double> GetTaskSpaceVel(const RigidBodyTree<double>& r,
                                const KinematicsCache<double>& cache,
                                const RigidBody<double>& body,
                                const Vector3<double>& local_offset) {
  const auto& element = cache.get_element(body.get_body_index());
  Vector3<double> pt = element.transform_to_world.translation();

  // Get the body's task space vel.
  // Converting from Plucker vector (Featherstone's spatial vectors) to spatial
  // vector algebra as defined by Abhinandan Jain.
  Vector6<double> v = element.twist_in_world;
  v.tail<3>() += v.head<3>().cross(pt);

  // Get the global offset between pt and body.
  Isometry3<double> H_pt_to_body(Isometry3<double>::Identity());
  H_pt_to_body.translation() = local_offset;
  auto H_pt_to_world = element.transform_to_world * H_pt_to_body;
  Vector3<double> world_offset =
      H_pt_to_world.translation() - element.transform_to_world.translation();

  // Add the linear vel from the body rotation.
  v.tail<3>() += v.head<3>().cross(world_offset);

  return v;
}

MatrixX<double> GetTaskSpaceJacobian(const RigidBodyTree<double>& r,
                                     const KinematicsCache<double>& cache,
                                     const RigidBody<double>& body,
                                     const Vector3<double>& local_offset) {
  std::vector<int> v_or_q_indices;
  MatrixX<double> Jg = r.geometricJacobian(cache, 0, body.get_body_index(), 0,
                                           true, &v_or_q_indices);
  MatrixX<double> J(6, r.get_num_velocities());
  J.setZero();

  Vector3<double> points =
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

Vector6<double> GetTaskSpaceJacobianDotTimesV(
    const RigidBodyTree<double>& r, const KinematicsCache<double>& cache,
    const RigidBody<double>& body, const Vector3<double>& local_offset) {
  // position of point in world
  Vector3<double> p =
      r.transformPoints(cache, local_offset, body.get_body_index(), 0);
  Vector6<double> twist = r.relativeTwist(cache, 0, body.get_body_index(), 0);
  Vector6<double> J_geometric_dot_times_v =
      r.geometricJacobianDotTimesV(cache, 0, body.get_body_index(), 0);

  // linear vel of r
  Vector3<double> pdot = twist.head<3>().cross(p) + twist.tail<3>();

  // each column of Jt = [Jg_omega; Jg_v + Jg_omega.cross(p)]
  // for Jtdot * v, the angular part stays the same,
  // for the linear part:
  //  = [\dot{Jg_v} + \dot{Jg_omega}.cross(p) + Jg_omega.cross(pdot)] * v
  //  = [liner part of JgdotV + angular of JgdotV.cross(p) + omega.cross(pdot)]
  Vector6<double> Jdv = J_geometric_dot_times_v;
  Jdv.tail<3>() +=
      twist.head<3>().cross(pdot) + J_geometric_dot_times_v.head<3>().cross(p);

  return Jdv;
}

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake

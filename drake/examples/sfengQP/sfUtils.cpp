#include "sfUtils.h"

Vector6d getTaskSpaceVel(const RigidBodyTree &r, const KinematicsCache<double> &cache, int body_or_frame_id, const Vector3d &local_offset)
{
  Isometry3d H_body_to_frame;
  int body_idx = r.parseBodyOrFrameID(body_or_frame_id, &H_body_to_frame);

  const auto &element = cache.getElement(*(r.bodies[body_idx]));
  Vector6d T = element.twist_in_world;
  Vector3d pt = element.transform_to_world.translation();
  
  // get the body's task space vel
  Vector6d v = T;
  v.tail<3>() += v.head<3>().cross(pt); 

  // global offset between pt and body
  auto H_world_to_frame = element.transform_to_world * H_body_to_frame;
  Isometry3d H_frame_to_pt(Isometry3d::Identity());
  H_frame_to_pt.translation() = local_offset;
  auto H_world_to_pt = H_world_to_frame * H_frame_to_pt; 
  Vector3d world_offset = H_world_to_pt.translation() - element.transform_to_world.translation();
  
  // add the linear vel from the body rotation
  v.tail<3>() += v.head<3>().cross(world_offset);

  return v;
}

MatrixXd getTaskSpaceJacobian(const RigidBodyTree &r, KinematicsCache<double> &cache, int body, const Vector3d &local_offset)
{
  std::vector<int> v_or_q_indices;
  KinematicPath body_path = r.findKinematicPath(0, body);
  MatrixXd Jg = r.geometricJacobian(cache, 0, body, 0, true, &v_or_q_indices);
  MatrixXd J(6, r.number_of_velocities());
  J.setZero();

  Vector3d points = r.transformPoints(cache, local_offset, body, 0);

  int col = 0;
  for (std::vector<int>::iterator it = v_or_q_indices.begin(); it != v_or_q_indices.end(); ++it) {
    // angular
    J.template block<SPACE_DIMENSION, 1>(0,*it) = Jg.block<3,1>(0,col);
    // linear, just like the linear velocity, assume qd = 1, the column is the linear velocity.
    J.template block<SPACE_DIMENSION, 1>(3,*it) = Jg.block<3,1>(3,col);
    J.template block<SPACE_DIMENSION, 1>(3,*it).noalias() += Jg.block<3,1>(0,col).cross(points);
    col++;
  }

  return J;
}

Vector6d getTaskSpaceJacobianDotTimesV(const RigidBodyTree &r, KinematicsCache<double> &cache, int body_or_frame_id, const Vector3d &local_offset)
{
  // position of point in world
  Vector3d p = r.transformPoints(cache, local_offset, body_or_frame_id, 0);
  Vector6d twist = r.relativeTwist(cache, 0, body_or_frame_id, 0);
  Vector6d J_geometric_dot_times_v = r.geometricJacobianDotTimesV(cache, 0, body_or_frame_id, 0);

  // linear vel of r
  Vector3d pdot = twist.head<3>().cross(p) + twist.tail<3>();

  // each column of J_task Jt = [Jg_omega; Jg_v + Jg_omega.cross(p)]
  // Jt * v, angular part stays the same, 
  // linear part = [\dot{Jg_v}v + \dot{Jg_omega}.cross(p) + Jg_omega.cross(rdot)] * v 
  //             = [lin of JgdotV + ang of JgdotV.cross(p) + omega.cross(rdot)]
  Vector6d Jdv = J_geometric_dot_times_v;
  Jdv.tail<3>() += twist.head<3>().cross(pdot) + J_geometric_dot_times_v.head<3>().cross(p);

  return Jdv;
}
 

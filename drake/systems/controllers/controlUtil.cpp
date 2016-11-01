#include "drake/systems/controllers/controlUtil.h"

#include <limits>
#include <utility>

#include "drake/math/autodiff.h"
#include "drake/math/expmap.h"
#include "drake/math/quaternion.h"
#include "drake/math/rotation_matrix.h"
#include "drake/util/drakeUtil.h"

using Eigen::AutoDiffScalar;
using Eigen::Dynamic;
using Eigen::Isometry3d;
using Eigen::Matrix3d;
using Eigen::Matrix;
using Eigen::MatrixBase;
using Eigen::MatrixXd;
using Eigen::Ref;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;
using drake::math::autoDiffToValueMatrix;
using drake::math::expmap2quat;

template <typename DerivedA, typename DerivedB>
// TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
void getRows(std::set<int>& rows, MatrixBase<DerivedA> const& M,
             // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
             MatrixBase<DerivedB>& Msub) {
  if (static_cast<int>(rows.size()) == M.rows()) {
    Msub = M;
    return;
  }

  int i = 0;
  for (std::set<int>::iterator iter = rows.begin(); iter != rows.end(); iter++)
    Msub.row(i++) = M.row(*iter);
}

template <typename DerivedA, typename DerivedB>
// TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
void getCols(std::set<int>& cols, MatrixBase<DerivedA> const& M,
             // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
             MatrixBase<DerivedB>& Msub) {
  if (static_cast<int>(cols.size()) == M.cols()) {
    Msub = M;
    return;
  }
  int i = 0;
  for (std::set<int>::iterator iter = cols.begin(); iter != cols.end(); iter++)
    Msub.col(i++) = M.col(*iter);
}

template <typename DerivedPhi1, typename DerivedPhi2, typename DerivedD>
void angleDiff(MatrixBase<DerivedPhi1> const& phi1,
               MatrixBase<DerivedPhi2> const& phi2,
               // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
               MatrixBase<DerivedD>& d) {
  d = phi2 - phi1;

  for (int i = 0; i < phi1.rows(); i++) {
    for (int j = 0; j < phi1.cols(); j++) {
      if (d(i, j) < -M_PI) {
        d(i, j) = fmod(d(i, j) + M_PI, 2 * M_PI) + M_PI;
      } else {
        d(i, j) = fmod(d(i, j) + M_PI, 2 * M_PI) - M_PI;
      }
    }
  }
}

bool inSupport(
    const drake::eigen_aligned_std_vector<SupportStateElement>& supports,
    int body_idx) {
  // HANDLE IF BODY_IDX IS A FRAME ID?
  for (size_t i = 0; i < supports.size(); i++) {
    if (supports[i].body_idx == body_idx) return true;
  }
  return false;
}

void surfaceTangents(const Vector3d& normal,
                     // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
                     Matrix<double, 3, m_surface_tangents>& d) {
  Vector3d t1, t2;
  double theta;

  if (1 - normal(2) < EPSILON) {  // handle the unit-normal case (since it's
                                  // unit length, just check z)
    t1 << 1, 0, 0;
  } else if (1 + normal(2) < EPSILON) {
    t1 << -1, 0, 0;  // same for the reflected case
  } else {           // now the general case
    t1 << normal(1), -normal(0), 0;
    t1 /= sqrt(normal(1) * normal(1) + normal(0) * normal(0));
  }

  t2 = t1.cross(normal);

  for (int k = 0; k < m_surface_tangents; k++) {
    theta = k * M_PI / m_surface_tangents;
    d.col(k) = cos(theta) * t1 + sin(theta) * t2;
  }
}

int contactPhi(const RigidBodyTree& r, const KinematicsCache<double>& cache,
               // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
               SupportStateElement& supp, VectorXd& phi) {
  int nc = static_cast<int>(supp.contact_pts.size());
  phi.resize(nc);

  if (nc < 1) return nc;

  int i = 0;
  for (auto pt_iter = supp.contact_pts.begin();
       pt_iter != supp.contact_pts.end(); pt_iter++) {
    Vector3d contact_pos = r.transformPoints(cache, *pt_iter, supp.body_idx, 0);
    phi(i) = supp.support_surface.head<3>().dot(contact_pos) +
             supp.support_surface(3);
    i++;
  }
  return nc;
}

int contactConstraintsBV(
    const RigidBodyTree& r, const KinematicsCache<double>& cache, int nc,
    std::vector<double> support_mus,
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    drake::eigen_aligned_std_vector<SupportStateElement>& supp, MatrixXd& B,
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    MatrixXd& JB, MatrixXd& Jp, VectorXd& Jpdotv, MatrixXd& normals) {
  int j, k = 0, nq = r.get_num_positions();

  B.resize(3, nc * 2 * m_surface_tangents);
  JB.resize(nq, nc * 2 * m_surface_tangents);
  Jp.resize(3 * nc, nq);
  Jpdotv.resize(3 * nc);
  normals.resize(3, nc);

  Vector3d contact_pos, pos, normal;
  MatrixXd J(3, nq);
  Matrix<double, 3, m_surface_tangents> d;

  for (auto iter = supp.begin(); iter != supp.end(); iter++) {
    double mu = support_mus[iter - supp.begin()];
    double norm = sqrt(1 + mu * mu);  // because normals and ds are orthogonal,
                                      // the norm has a simple form
    if (nc > 0) {
      for (auto pt_iter = iter->contact_pts.begin();
           pt_iter != iter->contact_pts.end(); pt_iter++) {
        contact_pos = r.transformPoints(cache, *pt_iter, iter->body_idx, 0);
        J = r.transformPointsJacobian(cache, *pt_iter, iter->body_idx, 0, true);

        normal = iter->support_surface.head(3);
        surfaceTangents(normal, d);
        for (j = 0; j < m_surface_tangents; j++) {
          B.col(2 * k * m_surface_tangents + j) =
              (normal + mu * d.col(j)) / norm;
          B.col((2 * k + 1) * m_surface_tangents + j) =
              (normal - mu * d.col(j)) / norm;

          JB.col(2 * k * m_surface_tangents + j) =
              J.transpose() * B.col(2 * k * m_surface_tangents + j);
          JB.col((2 * k + 1) * m_surface_tangents + j) =
              J.transpose() * B.col((2 * k + 1) * m_surface_tangents + j);
        }

        // store away kin sols into Jp and Jpdotv
        // NOTE: I'm cheating and using a slightly different ordering of J and
        // Jdot here
        Jp.block(3 * k, 0, 3, nq) = J;
        Vector3d pt = (*pt_iter).head(3);
        Jpdotv.block(3 * k, 0, 3, 1) =
            r.transformPointsJacobianDotTimesV(cache, pt, iter->body_idx, 0);
        normals.col(k) = normal;

        k++;
      }
    }
  }

  return k;
}

MatrixXd individualSupportCOPs(
    const RigidBodyTree& r, const KinematicsCache<double>& cache,
    const drake::eigen_aligned_std_vector<SupportStateElement>& active_supports,
    const MatrixXd& normals, const MatrixXd& B, const VectorXd& beta) {
  const int n_basis_vectors_per_contact =
      static_cast<int>(B.cols() / normals.cols());
  const int n = static_cast<int>(active_supports.size());

  int normals_start = 0;
  int beta_start = 0;

  MatrixXd individual_cops(3, n);
  individual_cops.fill(std::numeric_limits<double>::quiet_NaN());

  for (size_t j = 0; j < active_supports.size(); j++) {
    auto active_support = active_supports[j];
    auto contact_pts = active_support.contact_pts;

    int ncj = static_cast<int>(contact_pts.size());
    int active_support_length = n_basis_vectors_per_contact * ncj;
    auto normalsj = normals.middleCols(normals_start, ncj);
    Vector3d normal = normalsj.col(0);
    bool normals_identical =
        (normalsj.colwise().operator-(normal)).squaredNorm() < 1e-15;

    if (normals_identical) {  // otherwise computing a COP doesn't make sense
      const auto& Bj = B.middleCols(beta_start, active_support_length);
      const auto& betaj = beta.segment(beta_start, active_support_length);

      const auto& contact_positions =
          r.bodies[active_support.body_idx]->get_contact_points();
      Vector3d force = Vector3d::Zero();
      Vector3d torque = Vector3d::Zero();

      for (size_t k = 0; k < contact_pts.size(); k++) {
        // for (auto k = contact_pts.begin(); k!= contact_pts.end(); k++) {
        const auto& Bblock = Bj.middleCols(k * n_basis_vectors_per_contact,
                                           n_basis_vectors_per_contact);
        const auto& betablock = betaj.segment(k * n_basis_vectors_per_contact,
                                              n_basis_vectors_per_contact);
        Vector3d point_force = Bblock * betablock;
        force += point_force;
        Vector3d contact_pt = contact_pts[k].head(3);
        auto torquejk = contact_pt.cross(point_force);
        torque += torquejk;
      }

      Vector3d point_on_contact_plane = contact_positions.col(0);
      std::pair<Vector3d, double> cop_and_normal_torque =
          resolveCenterOfPressure(torque, force, normal,
                                  point_on_contact_plane);
      Vector3d cop_world = r.transformPoints(cache, cop_and_normal_torque.first,
                                             active_support.body_idx, 0);
      individual_cops.col(j) = cop_world;
    }

    normals_start += ncj;
    beta_start += active_support_length;
  }
  return individual_cops;
}

bool isSupportElementActive(SupportStateElement* se,
                            bool contact_force_detected,
                            bool kinematic_contact_detected) {
  bool is_active;
  //  std::cout << "checking element with body: " << se->body_idx << " force: "
  //  << contact_force_detected << " kin: " << kinematic_contact_detected <<
  //  std::endl;
  // Implement the logic described in QPInputConstantHeight.m
  if (!contact_force_detected && !kinematic_contact_detected) {
    is_active = se->support_logic_map[0];
  } else if (!contact_force_detected && kinematic_contact_detected) {
    is_active = se->support_logic_map[1];
  } else if (contact_force_detected && !kinematic_contact_detected) {
    is_active = se->support_logic_map[2];
  } else {  // (contact_force_detected && kinematic_contact_detected)
    is_active = se->support_logic_map[3];
  }
  return is_active;
}

Matrix<bool, Dynamic, 1> getActiveSupportMask(
    const RigidBodyTree& r, VectorXd q, VectorXd qd,
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    drake::eigen_aligned_std_vector<SupportStateElement>& available_supports,
    const Ref<const Matrix<bool, Dynamic, 1>>& contact_force_detected,
    double contact_threshold) {
  KinematicsCache<double> cache = r.doKinematics(q, qd);

  size_t nsupp = available_supports.size();
  Matrix<bool, Dynamic, 1> active_supp_mask =
      Matrix<bool, Dynamic, 1>::Zero(nsupp);
  VectorXd phi;
  SupportStateElement se;
  bool needs_kin_check;
  bool kin_contact;
  bool force_contact;

  for (size_t i = 0; i < nsupp; i++) {
    // mexPrintf("evaluating support: %d\n", i);
    se = available_supports[i];

    force_contact = (contact_force_detected(se.body_idx) != 0);
    // Determine if the body needs to be checked for kinematic contact. We only
    // need to check for kin contact if the logic map indicates that the
    // presence or absence of such contact would  affect the decision about
    // whether to use that body as a support.
    needs_kin_check = (((se.support_logic_map[1] != se.support_logic_map[0]) &&
                        (contact_force_detected(se.body_idx) == 0)) ||
                       ((se.support_logic_map[3] != se.support_logic_map[2]) &&
                        (contact_force_detected(se.body_idx) == 1)));

    if (needs_kin_check) {
      if (contact_threshold == -1) {
        kin_contact = true;
      } else {
        contactPhi(r, cache, se, phi);
        kin_contact = (phi.minCoeff() <= contact_threshold);
      }
    } else {
      kin_contact = false;  // we've determined already that kin contact doesn't
                            // matter for this support element
    }

    active_supp_mask(i) =
        isSupportElementActive(&se, force_contact, kin_contact);
    // mexPrintf("needs check: %d force contact: %d kin_contact: %d is_active:
    // %d\n", needs_kin_check, force_contact, kin_contact, active_supp_mask(i));
  }
  return active_supp_mask;
}

drake::eigen_aligned_std_vector<SupportStateElement> getActiveSupports(
    const RigidBodyTree& r, const VectorXd& q, const VectorXd& qd,
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    drake::eigen_aligned_std_vector<SupportStateElement>& available_supports,
    const Ref<const Matrix<bool, Dynamic, 1>>& contact_force_detected,
    double contact_threshold) {
  Matrix<bool, Dynamic, 1> active_supp_mask = getActiveSupportMask(
      r, q, qd, available_supports, contact_force_detected, contact_threshold);

  drake::eigen_aligned_std_vector<SupportStateElement> active_supports;

  for (size_t i = 0; i < available_supports.size(); i++) {
    if (active_supp_mask(i)) {
      active_supports.push_back(available_supports[i]);
    }
  }
  return active_supports;
}

Vector6d bodySpatialMotionPD(
    const RigidBodyTree& r, const DrakeRobotState& robot_state,
    const int body_index, const Isometry3d& body_pose_des,
    const Ref<const Vector6d>& body_v_des,
    const Ref<const Vector6d>& body_vdot_des, const Ref<const Vector6d>& Kp,
    const Ref<const Vector6d>& Kd, const Isometry3d& T_task_to_world) {
  // @param body_pose_des  desired pose in the task frame, this is the
  // homogeneous transformation from desired body frame to task frame
  // @param body_v_des    desired [xyzdot;angular_velocity] in task frame
  // @param body_vdot_des    desired [xyzddot;angular_acceleration] in task
  // frame
  // @param Kp     The gain in task frame
  // @param Kd     The gain in task frame
  // @param T_task_to_world  The homogeneous transform from task to world
  // @retval twist_dot, [angular_acceleration, xyz_acceleration] in body frame

  Isometry3d T_world_to_task = T_task_to_world.inverse();
  KinematicsCache<double> cache = r.doKinematics(robot_state.q, robot_state.qd);

  auto body_pose = r.relativeTransform(cache, 0, body_index);
  const auto& body_xyz = body_pose.translation();
  Vector3d body_xyz_task = T_world_to_task * body_xyz;
  Vector4d body_quat = drake::math::rotmat2quat(body_pose.linear());
  std::vector<int> v_indices;
  auto J_geometric =
      r.geometricJacobian(cache, 0, body_index, body_index, true, &v_indices);
  VectorXd v_compact(v_indices.size());
  for (size_t i = 0; i < v_indices.size(); i++) {
    v_compact(i) = robot_state.qd(v_indices[i]);
  }
  Vector6d body_twist = J_geometric * v_compact;
  Matrix3d R_body_to_world = drake::math::quat2rotmat(body_quat);
  Matrix3d R_world_to_body = R_body_to_world.transpose();
  Matrix3d R_body_to_task = T_world_to_task.linear() * R_body_to_world;
  Vector3d body_angular_vel =
      R_body_to_world *
      body_twist.head<3>();  // body_angular velocity in world frame
  Vector3d body_xyzdot =
      R_body_to_world * body_twist.tail<3>();  // body_xyzdot in world frame
  Vector3d body_angular_vel_task = T_world_to_task.linear() * body_angular_vel;
  Vector3d body_xyzdot_task = T_world_to_task.linear() * body_xyzdot;

  Vector3d body_xyz_des = body_pose_des.translation();
  Vector3d body_angular_vel_des = body_v_des.tail<3>();
  Vector3d body_angular_vel_dot_des = body_vdot_des.tail<3>();

  Vector3d xyz_err_task = body_xyz_des - body_xyz_task;

  Matrix3d R_des = body_pose_des.linear();
  Matrix3d R_err_task = R_des * R_body_to_task.transpose();
  Vector4d angleAxis_err_task = drake::math::rotmat2axis(R_err_task);
  Vector3d angular_err_task =
      angleAxis_err_task.head<3>() * angleAxis_err_task(3);

  Vector3d xyzdot_err_task = body_v_des.head<3>() - body_xyzdot_task;
  Vector3d angular_vel_err_task = body_angular_vel_des - body_angular_vel_task;

  Vector3d Kp_xyz = Kp.head<3>();
  Vector3d Kd_xyz = Kd.head<3>();
  Vector3d Kp_angular = Kp.tail<3>();
  Vector3d Kd_angular = Kd.tail<3>();
  Vector3d body_xyzddot_task =
      (Kp_xyz.array() * xyz_err_task.array()).matrix() +
      (Kd_xyz.array() * xyzdot_err_task.array()).matrix() +
      body_vdot_des.head<3>();
  Vector3d body_angular_vel_dot_task =
      (Kp_angular.array() * angular_err_task.array()).matrix() +
      (Kd_angular.array() * angular_vel_err_task.array()).matrix() +
      body_angular_vel_dot_des;

  Vector6d twist_dot = Vector6d::Zero();
  Vector3d body_xyzddot = T_task_to_world.linear() * body_xyzddot_task;
  Vector3d body_angular_vel_dot =
      T_task_to_world.linear() * body_angular_vel_dot_task;
  twist_dot.head<3>() = R_world_to_body * body_angular_vel_dot;
  twist_dot.tail<3>() = R_world_to_body * body_xyzddot -
                        body_twist.head<3>().cross(body_twist.tail<3>());
  return twist_dot;
}

void evaluateXYZExpmapCubicSpline(
    double t,
    const PiecewisePolynomial<double>& spline,
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    Isometry3d& body_pose_des,
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    Vector6d& xyzdot_angular_vel,
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    Vector6d& xyzddot_angular_accel) {
  Vector6d xyzexp = spline.value(t);
  auto derivative = spline.derivative();
  Vector6d xyzexpdot = derivative.value(t);
  Vector6d xyzexpddot = derivative.derivative().value(t);

  // translational part
  body_pose_des.translation() = xyzexp.head<3>();
  xyzdot_angular_vel.head<3>() = xyzexpdot.head<3>();
  xyzddot_angular_accel.head<3>() = xyzexpddot.head<3>();

  // rotational part
  auto expmap = xyzexp.tail<3>();
  auto expmap_dot = xyzexpdot.tail<3>();
  auto expmap_ddot = xyzexpddot.tail<3>();

  // construct autodiff version of expmap
  // autodiff derivatives represent first and second derivative w.r.t. time
  // TODO(tkoolen): should use 1 instead of dynamic, but causes issues
  // with eigen on MSVC 32 bit; should be fixed in 3.3
  typedef AutoDiffScalar<Matrix<double, Dynamic, 1>> ADScalar;
  // TODO(tkoolen): should use 1 instead of dynamic, but causes issues
  // with eigen on MSVC 32 bit; should be fixed in 3.3
  typedef AutoDiffScalar<Matrix<ADScalar, Dynamic, 1>> ADScalarSecondDeriv;
  Matrix<ADScalarSecondDeriv, 3, 1> expmap_autodiff;
  for (int i = 0; i < expmap_autodiff.size(); i++) {
    expmap_autodiff(i).value() = expmap(i);
    expmap_autodiff(i).derivatives().resize(1);
    expmap_autodiff(i).derivatives()(0) = expmap_dot(i);
    expmap_autodiff(i).derivatives()(0).derivatives().resize(1);
    expmap_autodiff(i).derivatives()(0).derivatives()(0) = expmap_ddot(i);
  }

  auto quat_autodiff = expmap2quat(expmap_autodiff);
  Vector4d quat = autoDiffToValueMatrix(autoDiffToValueMatrix(quat_autodiff));
  body_pose_des.linear() = drake::math::quat2rotmat(quat);

  // angular velocity and acceleration are computed from quaternion derivative
  // meaning of derivative vectors remains the same: first and second
  // derivatives w.r.t. time
  decltype(quat_autodiff) quat_dot_autodiff;
  for (int i = 0; i < quat_dot_autodiff.size(); i++) {
    quat_dot_autodiff(i).value() = quat_autodiff(i).derivatives()(0).value();
    quat_dot_autodiff(i).derivatives().resize(1);
    quat_dot_autodiff(i).derivatives()(0).value() =
        quat_autodiff(i).derivatives()(0).derivatives()(0);
    quat_dot_autodiff(i).derivatives()(0).derivatives().resize(1);
    quat_dot_autodiff(i).derivatives()(0).derivatives()(0) =
        std::numeric_limits<double>::quiet_NaN();  // we're not interested in
                                                   // second deriv of angular
                                                   // velocity
  }

  auto omega_autodiff =
      (quatdot2angularvelMatrix(quat_autodiff) * quat_dot_autodiff).eval();
  auto omega = xyzdot_angular_vel.tail<3>();
  auto omega_dot = xyzddot_angular_accel.tail<3>();
  for (int i = 0; i < omega_autodiff.size(); i++) {
    omega(i) = omega_autodiff(i).value().value();
    omega_dot(i) = omega_autodiff(i).derivatives()(0).value();
  }
}

void getRobotJointIndexMap(JointNames* joint_names,
                           RobotJointIndexMap* joint_map) {
  if (joint_names->drake.size() != joint_names->robot.size()) {
    throw std::runtime_error(
        "Cannot create joint name map: joint_names->drake and "
        "joint_names->robot must have the same length");
  }
  int njoints = static_cast<int>(joint_names->drake.size());
  joint_map->drake_to_robot.resize(njoints);
  joint_map->robot_to_drake.resize(njoints);

  bool has_match;
  for (int i = 0; i < njoints; i++) {
    has_match = false;
    for (int j = 0; j < njoints; j++) {
      if (joint_names->drake[i].compare(joint_names->robot[j]) == 0) {
        has_match = true;
        joint_map->drake_to_robot[i] = j;
        joint_map->robot_to_drake[j] = i;
      }
    }
    if (!has_match) {
      std::cout << "Could not match joint: " << joint_names->drake[i]
                << std::endl;
      throw std::runtime_error("Could not find a match for drake joint name");
    }
  }
  return;
}

template DRAKE_EXPORT void getRows(std::set<int>&,
                                              const MatrixBase<MatrixXd>&,
                                              MatrixBase<MatrixXd>&);
template DRAKE_EXPORT void getCols(std::set<int>&,
                                              const MatrixBase<MatrixXd>&,
                                              MatrixBase<MatrixXd>&);
template DRAKE_EXPORT void angleDiff(const MatrixBase<MatrixXd>&,
                                                const MatrixBase<MatrixXd>&,
                                                MatrixBase<MatrixXd>&);
template DRAKE_EXPORT void angleDiff(const MatrixBase<Vector3d>&,
                                                const MatrixBase<Vector3d>&,
                                                MatrixBase<Vector3d>&);

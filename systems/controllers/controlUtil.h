#pragma once

#include <cmath>
#include <set>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "drake/common/eigen_stl_types.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/multibody/rigid_body_tree.h"

const int m_surface_tangents =
    2;  // number of faces in the friction cone approx

#define EPSILON 10e-8

typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 7, 1> Vector7d;

typedef struct _support_state_element {
  int body_idx;
  drake::eigen_aligned_std_vector<Eigen::Vector3d> contact_pts;
  bool support_logic_map[4];
  Eigen::Vector4d support_surface;  // 4-vector describing a support surface:
                                    // [v; b] such that v' * [x;y;z] + b == 0
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} SupportStateElement;

struct DrakeRobotState {
  // drake-ordered position and velocity vectors, with timestamp (in s)
  double t;
  Eigen::VectorXd q;
  Eigen::VectorXd qd;
};

bool isSupportElementActive(
    SupportStateElement* se, bool contact_force_detected,
    bool kinematic_contact_detected);

Eigen::Matrix<bool, Eigen::Dynamic, 1>
getActiveSupportMask(
    RigidBodyTree<double>* r, Eigen::VectorXd q, Eigen::VectorXd qd,
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    drake::eigen_aligned_std_vector<SupportStateElement>& available_supports,
    const Eigen::Ref<const Eigen::Matrix<bool, Eigen::Dynamic, 1>>&
        contact_force_detected,
    double contact_threshold);

drake::eigen_aligned_std_vector<SupportStateElement>
getActiveSupports(
    const RigidBodyTree<double>& r,
    const Eigen::VectorXd& q, const Eigen::VectorXd& qd,
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    drake::eigen_aligned_std_vector<SupportStateElement>& available_supports,
    const Eigen::Ref<const Eigen::Matrix<bool, Eigen::Dynamic, 1>>&
        contact_force_detected,
    double contact_threshold);

template <typename DerivedA, typename DerivedB>
// TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
void getRows(std::set<int>& rows,
             Eigen::MatrixBase<DerivedA> const& M,
             // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
             Eigen::MatrixBase<DerivedB>& Msub);

template <typename DerivedA, typename DerivedB>
// TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
void getCols(std::set<int>& cols,
             Eigen::MatrixBase<DerivedA> const& M,
             // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
             Eigen::MatrixBase<DerivedB>& Msub);

template <typename DerivedPhi1, typename DerivedPhi2, typename DerivedD>
void angleDiff(
    Eigen::MatrixBase<DerivedPhi1> const& phi1,
    Eigen::MatrixBase<DerivedPhi2> const& phi2,
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    Eigen::MatrixBase<DerivedD>& d);

bool inSupport(
    const drake::eigen_aligned_std_vector<SupportStateElement>& supports,
    int body_idx);
void surfaceTangents(
    const Eigen::Vector3d& normal,
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    Eigen::Matrix<double, 3, m_surface_tangents>& d);
int contactPhi(
    const RigidBodyTree<double>& r,
    const KinematicsCache<double>& cache,
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    SupportStateElement& supp,
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    Eigen::VectorXd& phi);
int contactConstraintsBV(
    const RigidBodyTree<double>& r,
    const KinematicsCache<double>& cache, int nc,
    std::vector<double> support_mus,
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    drake::eigen_aligned_std_vector<SupportStateElement>& supp,
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    Eigen::MatrixXd& B, Eigen::MatrixXd& JB, Eigen::MatrixXd& Jp,
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    Eigen::VectorXd& Jpdotv, Eigen::MatrixXd& normals);
Eigen::MatrixXd individualSupportCOPs(
    const RigidBodyTree<double>& r, const KinematicsCache<double>& cache,
    const drake::eigen_aligned_std_vector<SupportStateElement>& active_supports,
    const Eigen::MatrixXd& normals, const Eigen::MatrixXd& B,
    const Eigen::VectorXd& beta);
Vector6d bodySpatialMotionPD(
    const RigidBodyTree<double>& r, const DrakeRobotState& robot_state,
    const int body_index, const Eigen::Isometry3d& body_pose_des,
    const Eigen::Ref<const Vector6d>& body_v_des,
    const Eigen::Ref<const Vector6d>& body_vdot_des,
    const Eigen::Ref<const Vector6d>& Kp, const Eigen::Ref<const Vector6d>& Kd,
    const Eigen::Isometry3d& T_task_to_world = Eigen::Isometry3d::Identity());

void evaluateXYZExpmapCubicSpline(
    double t, const PiecewisePolynomial<double>& spline,
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    Eigen::Isometry3d& body_pose_des, Vector6d& xyzdot_angular_vel,
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    Vector6d& xyzddot_angular_accel);

struct RobotJointIndexMap {
  Eigen::VectorXi drake_to_robot;
  Eigen::VectorXi robot_to_drake;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct JointNames {
  std::vector<std::string> robot;
  std::vector<std::string> drake;
};

void getRobotJointIndexMap(
    JointNames* joint_names, RobotJointIndexMap* joint_map);

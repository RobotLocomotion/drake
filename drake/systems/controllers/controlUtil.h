#ifndef _CONTROL_UTIL_H_
#define _CONTROL_UTIL_H_

#include <math.h>
#include <set>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/StdVector>

#include "drake/systems/plants/RigidBodyTree.h"
#include "drake/systems/trajectories/PiecewisePolynomial.h"
#include "drake/drakeControlUtil_export.h"


const int m_surface_tangents = 2;  // number of faces in the friction cone approx

#define EPSILON 10e-8

typedef Eigen::Matrix<double, 6,1> Vector6d;
typedef Eigen::Matrix<double, 7,1> Vector7d;

typedef struct _support_state_element
{
  int body_idx;
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> contact_pts;
  bool support_logic_map[4];
  Eigen::Vector4d support_surface; // 4-vector describing a support surface: [v; b] such that v' * [x;y;z] + b == 0
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
} SupportStateElement;

struct DrakeRobotState {
  // drake-ordered position and velocity vectors, with timestamp (in s)
  double t;
  Eigen::VectorXd q;
  Eigen::VectorXd qd;
};


DRAKECONTROLUTIL_EXPORT bool isSupportElementActive(SupportStateElement* se, bool contact_force_detected, bool kinematic_contact_detected);

DRAKECONTROLUTIL_EXPORT Eigen::Matrix<bool, Eigen::Dynamic, 1> getActiveSupportMask(RigidBodyTree * r, Eigen::VectorXd q, Eigen::VectorXd qd, std::vector<SupportStateElement,Eigen::aligned_allocator<SupportStateElement>> &available_supports, const Eigen::Ref<const Eigen::Matrix<bool, Eigen::Dynamic, 1>> &contact_force_detected, double contact_threshold);

DRAKECONTROLUTIL_EXPORT std::vector<SupportStateElement,Eigen::aligned_allocator<SupportStateElement>> getActiveSupports(RigidBodyTree * r, Eigen::VectorXd q, Eigen::VectorXd qd, std::vector<SupportStateElement,Eigen::aligned_allocator<SupportStateElement>> &available_supports, const Eigen::Ref<const Eigen::Matrix<bool, Eigen::Dynamic, 1>> &contact_force_detected, double contact_threshold);

template <typename DerivedA, typename DerivedB>
DRAKECONTROLUTIL_EXPORT void getRows(std::set<int> &rows, Eigen::MatrixBase<DerivedA> const &M, Eigen::MatrixBase<DerivedB> &Msub);

template <typename DerivedA, typename DerivedB>
DRAKECONTROLUTIL_EXPORT void getCols(std::set<int> &cols, Eigen::MatrixBase<DerivedA> const &M, Eigen::MatrixBase<DerivedB> &Msub);

template <typename DerivedPhi1, typename DerivedPhi2, typename DerivedD>
DRAKECONTROLUTIL_EXPORT void angleDiff(Eigen::MatrixBase<DerivedPhi1> const &phi1, Eigen::MatrixBase<DerivedPhi2> const &phi2, Eigen::MatrixBase<DerivedD> &d);

DRAKECONTROLUTIL_EXPORT bool inSupport(std::vector<SupportStateElement,Eigen::aligned_allocator<SupportStateElement>> &supports, int body_idx);
DRAKECONTROLUTIL_EXPORT void surfaceTangents(const Eigen::Vector3d & normal, Eigen::Matrix<double,3,m_surface_tangents> & d);
DRAKECONTROLUTIL_EXPORT int contactPhi(RigidBodyTree * r, const KinematicsCache<double>& cache, SupportStateElement& supp, Eigen::VectorXd &phi);
DRAKECONTROLUTIL_EXPORT int contactConstraintsBV(RigidBodyTree *r, const KinematicsCache<double>& cache, int nc, std::vector<double> support_mus, std::vector<SupportStateElement,Eigen::aligned_allocator<SupportStateElement>>& supp, Eigen::MatrixXd &B, Eigen::MatrixXd &JB, Eigen::MatrixXd &Jp, Eigen::VectorXd &Jpdotv, Eigen::MatrixXd &normals);
DRAKECONTROLUTIL_EXPORT Eigen::MatrixXd individualSupportCOPs(RigidBodyTree * r, const KinematicsCache<double>& cache, const std::vector<SupportStateElement,Eigen::aligned_allocator<SupportStateElement>>& active_supports, const Eigen::MatrixXd& normals, const Eigen::MatrixXd& B, const Eigen::VectorXd& beta);
DRAKECONTROLUTIL_EXPORT Vector6d bodySpatialMotionPD(RigidBodyTree *r, DrakeRobotState &robot_state, const int body_index, const Eigen::Isometry3d &body_pose_des, const Eigen::Ref<const Vector6d> &body_v_des, const Eigen::Ref<const Vector6d> &body_vdot_des, const Eigen::Ref<const Vector6d> &Kp, const Eigen::Ref<const Vector6d> &Kd, const Eigen::Isometry3d &T_task_to_world=Eigen::Isometry3d::Identity());

DRAKECONTROLUTIL_EXPORT void evaluateXYZExpmapCubicSpline(double t, const PiecewisePolynomial<double> &spline, Eigen::Isometry3d &body_pose_des, Vector6d &xyzdot_angular_vel, Vector6d &xyzddot_angular_accel);

struct RobotJointIndexMap {
  Eigen::VectorXi drake_to_robot;
  Eigen::VectorXi robot_to_drake;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

struct JointNames {
  std::vector<std::string> robot;
  std::vector<std::string> drake;
};

DRAKECONTROLUTIL_EXPORT void getRobotJointIndexMap(JointNames *joint_names, RobotJointIndexMap *joint_map);

#endif

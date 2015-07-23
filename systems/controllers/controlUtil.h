#ifndef _CONTROL_UTIL_H_
#define _CONTROL_UTIL_H_

#define _USE_MATH_DEFINES
#include <math.h>
#include <set>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/StdVector>

#include "RigidBodyManipulator.h"
#include "PiecewisePolynomial.h"

#if defined(WIN32) || defined(WIN64)
  #if defined(drakeControlUtil_EXPORTS)
    #define drakeControlUtilEXPORT __declspec( dllexport )
  #else
    #define drakeControlUtilEXPORT __declspec( dllimport )
  #endif
#else
  #define drakeControlUtilEXPORT
#endif


const int m_surface_tangents = 2;  // number of faces in the friction cone approx

#define EPSILON 10e-8

typedef Matrix<double, 6,1> Vector6d;
typedef Matrix<double, 7,1> Vector7d;

typedef struct _support_state_element
{
  int body_idx;
  std::vector<Vector3d, aligned_allocator<Vector3d>> contact_pts;
  bool support_logic_map[4];
  bool use_support_surface;
  Vector4d support_surface; // 4-vector describing a support surface: [v; b] such that v' * [x;y;z] + b == 0
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
} SupportStateElement;

struct DrakeRobotState {
  // drake-ordered position and velocity vectors, with timestamp (in s)
  double t;
  VectorXd q;
  VectorXd qd;
};


drakeControlUtilEXPORT bool isSupportElementActive(SupportStateElement* se, bool contact_force_detected, bool kinematic_contact_detected);

drakeControlUtilEXPORT Matrix<bool, Dynamic, 1> getActiveSupportMask(RigidBodyManipulator* r, void* map_ptr, VectorXd q, VectorXd qd, std::vector<SupportStateElement,Eigen::aligned_allocator<SupportStateElement>> &available_supports, const Ref<const Matrix<bool, Dynamic, 1>> &contact_force_detected, double contact_threshold, double terrain_height);

drakeControlUtilEXPORT std::vector<SupportStateElement,Eigen::aligned_allocator<SupportStateElement>> getActiveSupports(RigidBodyManipulator* r, void* map_ptr, VectorXd q, VectorXd qd, std::vector<SupportStateElement,Eigen::aligned_allocator<SupportStateElement>> &available_supports, const Ref<const Matrix<bool, Dynamic, 1>> &contact_force_detected, double contact_threshold, double terrain_height);

template <typename DerivedA, typename DerivedB>
drakeControlUtilEXPORT void getRows(std::set<int> &rows, MatrixBase<DerivedA> const &M, MatrixBase<DerivedB> &Msub);

template <typename DerivedA, typename DerivedB>
drakeControlUtilEXPORT void getCols(std::set<int> &cols, MatrixBase<DerivedA> const &M, MatrixBase<DerivedB> &Msub);

template <typename DerivedPhi1, typename DerivedPhi2, typename DerivedD>
drakeControlUtilEXPORT void angleDiff(MatrixBase<DerivedPhi1> const &phi1, MatrixBase<DerivedPhi2> const &phi2, MatrixBase<DerivedD> &d);

drakeControlUtilEXPORT bool inSupport(std::vector<SupportStateElement,Eigen::aligned_allocator<SupportStateElement>> &supports, int body_idx);
drakeControlUtilEXPORT void collisionDetect(void* map_ptr, Vector3d const & contact_pos, Vector3d &pos, Vector3d *normal, double terrain_height);
drakeControlUtilEXPORT void surfaceTangents(const Vector3d & normal, Matrix<double,3,m_surface_tangents> & d);
drakeControlUtilEXPORT int contactPhi(RigidBodyManipulator* r, SupportStateElement& supp, void *map_ptr, VectorXd &phi, double terrain_height);
drakeControlUtilEXPORT int contactConstraints(RigidBodyManipulator *r, int nc, std::vector<SupportStateElement,Eigen::aligned_allocator<SupportStateElement>>& supp, void *map_ptr, MatrixXd &n, MatrixXd &D, MatrixXd &Jp, MatrixXd &Jpdot,double terrain_height);
drakeControlUtilEXPORT int contactConstraintsBV(RigidBodyManipulator *r, int nc, std::vector<double> support_mus, std::vector<SupportStateElement,Eigen::aligned_allocator<SupportStateElement>>& supp, void *map_ptr, MatrixXd &B, MatrixXd &JB, MatrixXd &Jp, VectorXd &Jpdotv, MatrixXd &normals, double terrain_height);
drakeControlUtilEXPORT MatrixXd individualSupportCOPs(RigidBodyManipulator* r, const std::vector<SupportStateElement,Eigen::aligned_allocator<SupportStateElement>>& active_supports, const MatrixXd& normals, const MatrixXd& B, const VectorXd& beta);
drakeControlUtilEXPORT Vector6d bodySpatialMotionPD(RigidBodyManipulator *r, DrakeRobotState &robot_state, const int body_index, const Isometry3d &body_pose_des, const Ref<const Vector6d> &body_v_des, const Ref<const Vector6d> &body_vdot_des, const Ref<const Vector6d> &Kp, const Ref<const Vector6d> &Kd, const Isometry3d &T_task_to_world=Isometry3d::Identity());

drakeControlUtilEXPORT void evaluateXYZExpmapCubicSpline(double t, const PiecewisePolynomial<double> &spline, Isometry3d &body_pose_des, Vector6d &xyzdot_angular_vel, Vector6d &xyzddot_angular_accel);

struct RobotJointIndexMap {
  VectorXi drake_to_robot;
  VectorXi robot_to_drake;
};

struct JointNames {
  std::vector<std::string> robot;
  std::vector<std::string> drake;
};

drakeControlUtilEXPORT void getRobotJointIndexMap(JointNames *joint_names, RobotJointIndexMap *joint_map);

#endif

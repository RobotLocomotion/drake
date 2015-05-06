#ifndef _QPCOMMON_H_
#define _QPCOMMON_H_

#include "controlUtil.h"
#include "drakeUtil.h"
#include "fastQP.h"
#include "gurobiQP.h"
#include "drake/lcmt_qp_controller_input.hpp"
#include "ExponentialPlusPiecewisePolynomial.h"
#include <vector>

const double REG = 1e-8;

struct QPControllerData {
  GRBenv *env;
  RigidBodyManipulator* r;
  double slack_limit; // maximum absolute magnitude of acceleration slack variable values
  VectorXd umin,umax;
  void* map_ptr;
  std::set<int> active;

  // preallocate memory
  MatrixXd H, H_float, H_act;
  VectorXd C, C_float, C_act;
  MatrixXd B, B_act;
  MatrixXd J;
  Vector3d Jdotv;
  MatrixXd J_xy;
  Vector2d Jdotv_xy;
  MatrixXd Hqp;
  RowVectorXd fqp;
  
  // momentum controller-specific
  MatrixXd Ag; // centroidal momentum matrix
  Vector6d Agdot_times_v; // centroidal momentum velocity-dependent bias
  MatrixXd Ak; // centroidal angular momentum matrix
  Vector3d Akdot_times_v; // centroidal angular momentum velocity-dependent bias

  MatrixXd W_kdot; // quadratic cost for angular momentum rate: (kdot_des - kdot)'*W*(kdot_des - kdot)
  VectorXd w_qdd; 
  double w_grf; 
  double w_slack; 
  double Kp_ang; // angular momentum (k) P gain 
  double Kp_accel; // gain for support acceleration constraint: accel=-Kp_accel*vel

  int n_body_accel_inputs;
  int n_body_accel_eq_constraints;
  VectorXd body_accel_input_weights;
  int n_body_accel_bounds;
  std::vector<int> accel_bound_body_idx;
  std::vector<Vector6d,aligned_allocator<Vector6d>> min_body_acceleration;
  std::vector<Vector6d,aligned_allocator<Vector6d>> max_body_acceleration;

  // gurobi active set params
  int *vbasis;
  int *cbasis;
  int vbasis_len;
  int cbasis_len;
};

struct QPControllerState {
  double t_prev;
  bool foot_contact_prev[2];
  VectorXd vref_integrator_state;
  VectorXd q_integrator_state;
  std::set<int> active;
  int num_active_contact_pts;

  // gurobi active set params
  int *vbasis;
  int *cbasis;
  int vbasis_len;
  int cbasis_len;
};

struct BodyIdsCache {
  int r_foot;
  int l_foot;
  int pelvis;
};
   
struct RobotPropertyCache {
  typedef std::map<std::string, Eigen::Matrix3Xd> ContactGroupNameToContactPointsMap;
  std::vector<ContactGroupNameToContactPointsMap> contact_groups; // one for each support
  std::map<std::string, Eigen::VectorXi> position_indices;
  BodyIdsCache body_ids;
  VectorXi actuated_indices;
  int num_bodies;
};

struct VRefIntegratorParams {
  bool zero_ankles_on_contact;
  double eta;
  double delta_max;
};

struct IntegratorParams {
  VectorXd gains;
  VectorXd clamps;
  double eta;
};

struct Bounds {
  VectorXd min;
  VectorXd max;
};

struct JointSoftLimitParams {
  Matrix<bool, Dynamic, 1> enabled;
  VectorXi disable_when_body_in_support;
  VectorXd lb;
  VectorXd ub;
  VectorXd kp;
  VectorXd kd;
  VectorXd weight;
  VectorXd k_logistic;
};

struct WholeBodyParams {
  VectorXd Kp;
  VectorXd Kd;
  VectorXd w_qdd;

  double damping_ratio;
  IntegratorParams integrator;
  Bounds qdd_bounds;
};

struct BodyMotionParams {
  VectorXd Kp;
  VectorXd Kd;
  Bounds accel_bounds;
  double weight;
};

struct AtlasHardwareGains {
  VectorXd k_f_p;
  VectorXd k_q_p;
  VectorXd k_q_i;
  VectorXd k_qd_p;
  VectorXd ff_qd;
  VectorXd ff_f_d;
  VectorXd ff_const;
  VectorXd ff_qd_d;
};

struct AtlasHardwareParams {
  AtlasHardwareGains gains;
  Matrix<bool, Dynamic, 1> joint_is_force_controlled;
  Matrix<bool, Dynamic, 1> joint_is_position_controlled;
};

struct AtlasParams {
  WholeBodyParams whole_body;
  std::vector<BodyMotionParams> body_motion;
  VRefIntegratorParams vref_integrator;
  JointSoftLimitParams joint_soft_limits;
  AtlasHardwareParams hardware;
  Matrix3d W_kdot;
  double Kp_ang;
  double w_slack;
  double slack_limit;
  double w_grf;
  double Kp_accel;
  double contact_threshold;
  double min_knee_angle;
};

struct NewQPControllerData {
  GRBenv *env;
  RigidBodyManipulator* r;
  std::map<std::string,AtlasParams> param_sets;
  RobotPropertyCache rpc;
  void* map_ptr;
  double default_terrain_height;
  VectorXd umin,umax;
  int use_fast_qp;
  JointNames input_joint_names;
  std::vector<std::string> state_coordinate_names;

  // preallocate memory
  MatrixXd H, H_float, H_act;
  VectorXd C, C_float, C_act;
  MatrixXd B, B_act;
  MatrixXd J;
  Vector3d Jdotv;
  MatrixXd J_xy;
  Vector2d Jdotv_xy;
  MatrixXd Hqp;
  RowVectorXd fqp;
  VectorXd qdd_lb;
  VectorXd qdd_ub;
  
  // momentum controller-specific
  MatrixXd Ag; // centroidal momentum matrix
  Vector6d Agdot_times_v; // centroidal momentum velocity-dependent bias
  MatrixXd Ak; // centroidal angular momentum matrix
  Vector3d Akdot_times_v; // centroidal angular momentum velocity-dependent bias

  // logical separation for the "state", that is, things we expect to change at every iteration
  // and which must persist to the next iteration
  QPControllerState state;

};

struct DesiredBodyAcceleration {
  int body_or_frame_id0;
  Vector6d body_vdot;
  double weight;
  Bounds accel_bounds;
  bool control_pose_when_in_contact;
  bool use_spatial_velocity;
  KinematicPath body_path;
  Isometry3d T_task_to_world;
  Vector6d weight_multiplier;
};

struct QPControllerOutput {
  VectorXd q_ref;
  VectorXd qd_ref;
  VectorXd qdd;
  VectorXd u;
};

struct QPControllerDebugData {
  std::vector<SupportStateElement,Eigen::aligned_allocator<SupportStateElement>> active_supports;
  int nc;
  MatrixXd normals;
  MatrixXd B;
  VectorXd alpha;
  VectorXd f;
  MatrixXd Aeq;
  VectorXd beq;
  MatrixXd Ain_lb_ub;
  VectorXd bin_lb_ub;
  MatrixXd Qnfdiag;
  MatrixXd Qneps;
  VectorXd x_bar;
  MatrixXd S;
  VectorXd s1;
  VectorXd s1dot;
  double s2dot;
  MatrixXd A_ls;
  MatrixXd B_ls;
  MatrixXd Jcom;
  VectorXd Jcomdotv;
  VectorXd beta;
};

struct PIDOutput {
  VectorXd q_ref;
  VectorXd qddot_des;
};

//enum PlanShiftMode {NONE, XYZ, Z_ONLY, Z_AND_ZMP};


PIDOutput wholeBodyPID(NewQPControllerData *pdata, double t, const Ref<const VectorXd> &q, const Ref<const VectorXd> &qd, const Ref<const VectorXd> &q_des, WholeBodyParams *params);

VectorXd velocityReference(NewQPControllerData *pdata, double t, const Ref<VectorXd> &q, const Ref<VectorXd> &qd, const Ref<VectorXd> &qdd, bool foot_contact[2], VRefIntegratorParams *params, RobotPropertyCache *rpc);

std::vector<SupportStateElement,Eigen::aligned_allocator<SupportStateElement>> loadAvailableSupports(std::shared_ptr<drake::lcmt_qp_controller_input> qp_input);

int setupAndSolveQP(NewQPControllerData *pdata, std::shared_ptr<drake::lcmt_qp_controller_input> qp_input, DrakeRobotState &robot_state, const Ref<Matrix<bool, Dynamic, 1>> &b_contact_force, QPControllerOutput *qp_output, std::shared_ptr<QPControllerDebugData> debug);


#endif

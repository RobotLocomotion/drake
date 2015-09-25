#ifndef _QPCOMMON_H_
#define _QPCOMMON_H_

#include "controlUtil.h"
#include "drakeUtil.h"
#include "fastQP.h"
#include "lcmtypes/drake/lcmt_qp_controller_input.hpp"
#include "ExponentialPlusPiecewisePolynomial.h"
#include <vector>
#include "ForceTorqueMeasurement.h"
#include "Side.h"
#include "gurobiQP.h"

const double REG = 1e-8;

struct QPControllerData {
  GRBenv *env;
  RigidBodyManipulator* r;
  double slack_limit; // maximum absolute magnitude of acceleration slack variable values
  Eigen::VectorXd umin,umax;
  void* map_ptr;
  std::set<int> active;

  // preallocate memory
  Eigen::MatrixXd H, H_float, H_act;
  Eigen::VectorXd C, C_float, C_act;
  Eigen::MatrixXd B, B_act;
  Eigen::MatrixXd J;
  Eigen::Vector3d Jdotv;
  Eigen::MatrixXd J_xy;
  Eigen::Vector2d Jdotv_xy;
  Eigen::MatrixXd Hqp;
  Eigen::RowVectorXd fqp;
  
  // momentum controller-specific
  Eigen::MatrixXd Ag; // centroidal momentum matrix
  Vector6d Agdot_times_v; // centroidal momentum velocity-dependent bias
  Eigen::MatrixXd Ak; // centroidal angular momentum matrix
  Eigen::Vector3d Akdot_times_v; // centroidal angular momentum velocity-dependent bias

  Eigen::MatrixXd W_kdot; // quadratic cost for angular momentum rate: (kdot_des - kdot)'*W*(kdot_des - kdot)
  Eigen::VectorXd w_qdd; 
  double w_grf; 
  double w_slack; 
  double Kp_ang; // angular momentum (k) P gain 
  double Kp_accel; // gain for support acceleration constraint: accel=-Kp_accel*vel

  int n_body_accel_inputs;
  int n_body_accel_eq_constraints;
  Eigen::VectorXd body_accel_input_weights;
  int n_body_accel_bounds;
  std::vector<int> accel_bound_body_idx;
  std::vector<Vector6d,Eigen::aligned_allocator<Vector6d>> min_body_acceleration;
  std::vector<Vector6d,Eigen::aligned_allocator<Vector6d>> max_body_acceleration;

  // gurobi active set params
  int *vbasis;
  int *cbasis;
  int vbasis_len;
  int cbasis_len;
};

struct QPControllerState {
  double t_prev;
  bool foot_contact_prev[2];
  Eigen::VectorXd vref_integrator_state;
  Eigen::VectorXd q_integrator_state;
  std::set<int> active;
  int num_active_contact_pts;

  // center of mass observer
  Eigen::Vector4d center_of_mass_observer_state;
  Eigen::Vector3d last_com_ddot;

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
  Eigen::VectorXi actuated_indices;
  int num_bodies;
};

struct VRefIntegratorParams {
  bool zero_ankles_on_contact;
  double eta;
  double delta_max;
};

struct IntegratorParams {
  Eigen::VectorXd gains;
  Eigen::VectorXd clamps;
  double eta;
};

struct Bounds {
  Eigen::VectorXd min;
  Eigen::VectorXd max;
};

struct JointSoftLimitParams {
  Eigen::Matrix<bool, Eigen::Dynamic, 1> enabled;
  Eigen::VectorXi disable_when_body_in_support;
  Eigen::VectorXd lb;
  Eigen::VectorXd ub;
  Eigen::VectorXd kp;
  Eigen::VectorXd kd;
  Eigen::VectorXd weight;
  Eigen::VectorXd k_logistic;
};

struct WholeBodyParams {
  Eigen::VectorXd Kp;
  Eigen::VectorXd Kd;
  Eigen::VectorXd w_qdd;

  double damping_ratio;
  IntegratorParams integrator;
  Bounds qdd_bounds;
};

struct BodyMotionParams {
  Eigen::VectorXd Kp;
  Eigen::VectorXd Kd;
  Bounds accel_bounds;
  double weight;
};

struct AtlasHardwareGains {
  Eigen::VectorXd k_f_p;
  Eigen::VectorXd k_q_p;
  Eigen::VectorXd k_q_i;
  Eigen::VectorXd k_qd_p;
  Eigen::VectorXd ff_qd;
  Eigen::VectorXd ff_f_d;
  Eigen::VectorXd ff_const;
  Eigen::VectorXd ff_qd_d;
};

struct AtlasHardwareParams {
  AtlasHardwareGains gains;
  Eigen::Matrix<bool, Eigen::Dynamic, 1> joint_is_force_controlled;
  Eigen::Matrix<bool, Eigen::Dynamic, 1> joint_is_position_controlled;
};

struct AtlasParams {
  WholeBodyParams whole_body;
  std::vector<BodyMotionParams> body_motion;
  VRefIntegratorParams vref_integrator;
  JointSoftLimitParams joint_soft_limits;
  AtlasHardwareParams hardware;
  Eigen::Matrix3d W_kdot;
  double Kp_ang;
  double w_slack;
  double slack_limit;
  double w_grf;
  double Kp_accel;
  double contact_threshold;
  double min_knee_angle;
  bool use_center_of_mass_observer;
  Eigen::Matrix4d center_of_mass_observer_gain;
};

class NewQPControllerData {
public:
  GRBenv *env;
  RigidBodyManipulator* r;
  std::map<std::string,AtlasParams> param_sets;
  RobotPropertyCache rpc;
  void* map_ptr;
  Eigen::VectorXd umin,umax;
  int use_fast_qp;
  JointNames input_joint_names;
  std::vector<std::string> state_coordinate_names;

  // preallocate memory
  KinematicsCache<double> cache;
  Eigen::MatrixXd H, H_float, H_act;
  Eigen::VectorXd C, C_float, C_act;
  Eigen::MatrixXd B, B_act;
  Eigen::MatrixXd J;
  Eigen::Vector3d Jdotv;
  Eigen::MatrixXd J_xy;
  Eigen::Vector2d Jdotv_xy;
  Eigen::MatrixXd Hqp;
  Eigen::RowVectorXd fqp;
  Eigen::VectorXd qdd_lb;
  Eigen::VectorXd qdd_ub;
  
  // momentum controller-specific
  Eigen::MatrixXd Ag; // centroidal momentum matrix
  Vector6d Agdot_times_v; // centroidal momentum velocity-dependent bias
  Eigen::MatrixXd Ak; // centroidal angular momentum matrix
  Eigen::Vector3d Akdot_times_v; // centroidal angular momentum velocity-dependent bias

  // logical separation for the "state", that is, things we expect to change at every iteration
  // and which must persist to the next iteration
  QPControllerState state;

  NewQPControllerData(RigidBodyManipulator* r) :
      r(r), cache(r->bodies, 0)
  {
    // empty
  }
};

struct DesiredBodyAcceleration {
  int body_or_frame_id0;
  Vector6d body_vdot;
  double weight;
  Bounds accel_bounds;
  bool control_pose_when_in_contact;
  bool use_spatial_velocity;
  KinematicPath body_path;
  Eigen::Isometry3d T_task_to_world;
  Vector6d weight_multiplier;
};

struct QPControllerOutput {
  Eigen::VectorXd q_ref;
  Eigen::VectorXd qd_ref;
  Eigen::VectorXd qdd;
  Eigen::VectorXd u;
};

struct QPControllerDebugData {
  std::vector<SupportStateElement,Eigen::aligned_allocator<SupportStateElement>> active_supports;
  int nc;
  Eigen::MatrixXd normals;
  Eigen::MatrixXd B;
  Eigen::VectorXd alpha;
  Eigen::VectorXd f;
  Eigen::MatrixXd Aeq;
  Eigen::VectorXd beq;
  Eigen::MatrixXd Ain_lb_ub;
  Eigen::VectorXd bin_lb_ub;
  Eigen::MatrixXd Qnfdiag;
  Eigen::MatrixXd Qneps;
  Eigen::VectorXd x_bar;
  Eigen::MatrixXd S;
  Eigen::VectorXd s1;
  Eigen::VectorXd s1dot;
  double s2dot;
  Eigen::MatrixXd A_ls;
  Eigen::MatrixXd B_ls;
  Eigen::MatrixXd Jcom;
  Eigen::VectorXd Jcomdotv;
  Eigen::VectorXd beta;
};

struct PIDOutput {
  Eigen::VectorXd q_ref;
  Eigen::VectorXd qddot_des;
};

//enum PlanShiftMode {NONE, XYZ, Z_ONLY, Z_AND_ZMP};


PIDOutput wholeBodyPID(NewQPControllerData *pdata, double t, const Eigen::Ref<const Eigen::VectorXd> &q, const Eigen::Ref<const Eigen::VectorXd> &qd, const Eigen::Ref<const Eigen::VectorXd> &q_des, WholeBodyParams *params);

Eigen::VectorXd velocityReference(NewQPControllerData *pdata, double t, const Eigen::Ref<Eigen::VectorXd> &q, const Eigen::Ref<Eigen::VectorXd> &qd, const Eigen::Ref<Eigen::VectorXd> &qdd, bool foot_contact[2], VRefIntegratorParams *params, RobotPropertyCache *rpc);

std::vector<SupportStateElement,Eigen::aligned_allocator<SupportStateElement>> loadAvailableSupports(std::shared_ptr<drake::lcmt_qp_controller_input> qp_input);

int setupAndSolveQP(
		NewQPControllerData *pdata, std::shared_ptr<drake::lcmt_qp_controller_input> qp_input, DrakeRobotState &robot_state,
		const Eigen::Ref<Eigen::Matrix<bool, Eigen::Dynamic, 1>> &b_contact_force, const std::map<Side, ForceTorqueMeasurement>& foot_force_torque_measurements,
		QPControllerOutput *qp_output, std::shared_ptr<QPControllerDebugData> debug);


#endif

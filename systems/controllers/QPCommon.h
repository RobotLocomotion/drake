#include "controlUtil.h"
#include "drake/fastQP.h"
#include "drake/gurobiQP.h"

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
  MatrixXd J, Jdot;
  MatrixXd J_xy, Jdot_xy;
  MatrixXd Hqp;
  RowVectorXd fqp;
  
  // momentum controller-specific
  MatrixXd Ag, Agdot; // centroidal momentum matrix
  MatrixXd Ak, Akdot; // centroidal angular momentum matrix
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

  // gurobi active set params
  int *vbasis;
  int *cbasis;
  int vbasis_len;
  int cbasis_len;
};

struct PositionIndicesCache {
  VectorXi r_leg_kny;
  VectorXi l_leg_kny;
  VectorXi r_leg;
  VectorXi l_leg;
  VectorXi r_leg_ak;
  VectorXi l_leg_ak;
};

struct BodyIdsCache {
  int r_foot;
  int l_foot;
  int pelvis;
};
   
struct RobotPropertyCache {
  PositionIndicesCache position_indices;
  BodyIdsCache body_ids;
  VectorXi actuated_indices;
};

struct MapTest {
  MapTest (double *A_data, size_t A_size): A(A_data, A_size) {};
  Map<VectorXd> A;
};

struct VRefIntegratorParams {
  bool zero_ankles_on_contact;
  double eta;
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

struct AtlasParams {
  WholeBodyParams whole_body;
  std::vector<BodyMotionParams> body_motion;
  VRefIntegratorParams vref_integrator;
  Matrix3d W_kdot;
  double Kp_ang;
  double w_slack;
  double slack_limit;
  double w_grf;
  double Kp_accel;
  double contact_threshold;
  double min_knee_angle;
};

struct AtlasParamSets {
  AtlasParams walking;
  AtlasParams standing;
  AtlasParams kinematic;
};

struct NewQPControllerData {
  GRBenv *env;
  RigidBodyManipulator* r;
  AtlasParamSets param_sets;
  RobotPropertyCache rpc;
  void* map_ptr;
  VectorXd umin,umax;

  // preallocate memory
  MatrixXd H, H_float, H_act;
  VectorXd C, C_float, C_act;
  MatrixXd B, B_act;
  MatrixXd J, Jdot;
  MatrixXd J_xy, Jdot_xy;
  MatrixXd Hqp;
  RowVectorXd fqp;
  
  // momentum controller-specific
  MatrixXd Ag, Agdot; // centroidal momentum matrix
  MatrixXd Ak, Akdot; // centroidal angular momentum matrix

  // logical separation for the "state", that is, things we expect to change at every iteration
  // and which must persist to the next iteration
  QPControllerState state;

};



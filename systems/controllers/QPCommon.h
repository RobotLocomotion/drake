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
  IntegratorParams (double *gains_data, size_t gains_size,
                    double *clamps_data, size_t clamps_size):
    gains(gains_data, gains_size),
    clamps(clamps_data, clamps_size)
    {}
  Map<VectorXd> gains;
  Map<VectorXd> clamps;
  double eta;
};

struct QDDBounds {
  QDDBounds (double *min_data, size_t min_size,
             double *max_data, size_t max_size):
    min(min_data, min_size),
    max(max_data, max_size)
    {}
  Map<VectorXd> min;
  Map<VectorXd> max;
};

struct WholeBodyParams {
  WholeBodyParams (double *Kp_data, size_t Kp_size,
                   double *Kd_data, size_t Kd_size,
                   double *w_qdd_data, size_t w_qdd_size,
                   double *gains_data, size_t gains_size,
                   double *clamps_data, size_t clamps_size,
                   double *qdd_min_data, size_t qdd_min_size,
                   double *qdd_max_data, size_t qdd_max_size
                   ):
    Kp(Kp_data, Kp_size),
    Kd(Kd_data, Kd_size),
    w_qdd(w_qdd_data, w_qdd_size),
    integrator(gains_data, gains_size,
               clamps_data, clamps_size),
    qdd_bounds(qdd_min_data, qdd_min_size, 
               qdd_max_data, qdd_max_size)
    {};
  Map<VectorXd> Kp;
  Map<VectorXd> Kd;
  Map<VectorXd> w_qdd;

  double damping_ratio;
  IntegratorParams integrator;
  QDDBounds qdd_bounds;
};


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
#include <drakeQP_export.h> // TODO: do exports

const double REG = 1e-8;

struct QPControllerData {
  GRBenv *env;
  RigidBodyTree * r;
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

struct PositionIndices {
  std::map<Side, std::vector<int>> legs;
  std::map<Side, int> knees;
  std::map<Side, std::vector<int>> ankles;
  std::map<Side, std::vector<int>> arms;
  std::vector<int> neck;
  int back_bkz;
  int back_bky;
};
   
struct RobotPropertyCache {
  PositionIndices position_indices;
  std::map<Side, int> foot_ids;
};

struct VRefIntegratorParams {
  VRefIntegratorParams():
    zero_ankles_on_contact(false),
    eta(0.0),
    delta_max(0.0) {}

  bool zero_ankles_on_contact;
  double eta;
  double delta_max;

  friend bool operator==(const VRefIntegratorParams& lhs, const VRefIntegratorParams& rhs) {
    // std::cout << "comparing VRefIntegratorParams" << std::endl;
    return lhs.zero_ankles_on_contact == rhs.zero_ankles_on_contact
        && lhs.eta == rhs.eta
        && lhs.delta_max == rhs.delta_max;
  }
};

struct IntegratorParams {
  IntegratorParams(const RigidBodyTree &robot):
    gains(Eigen::VectorXd::Zero(robot.num_positions)),
    clamps(Eigen::VectorXd::Zero(robot.num_positions)),
    eta(0.0) {}

  Eigen::VectorXd gains;
  Eigen::VectorXd clamps;
  double eta;

  friend std::ostream& operator<<(std::ostream& os, const IntegratorParams& params) {
    os << "gains: " << params.gains.transpose() << std::endl
      << "clamps: " << params.clamps.transpose() << std::endl
      << "eta: " << params.eta << std::endl;
      return os;
  }
  friend bool operator==(const IntegratorParams& lhs, const IntegratorParams& rhs) {
    // std::cout << "comparing IntegratorParams" << std::endl;
    return lhs.gains.isApprox(rhs.gains)
        && lhs.clamps.isApprox(rhs.clamps)
        && lhs.eta == rhs.eta;
  }
};

struct Bounds {
  Bounds(const Eigen::Ref<const Eigen::VectorXd> &min_, const Eigen::Ref<const Eigen::VectorXd> &max_):
    min(min_),
    max(max_) {}

  Eigen::VectorXd min;
  Eigen::VectorXd max;

  friend std::ostream& operator<<(std::ostream& os, const Bounds& bounds) {
    os << "min: " << bounds.min.transpose() << std::endl
       << "max: " << bounds.max.transpose() << std::endl;
    return os;
   }
  friend bool operator==(const Bounds& lhs, const Bounds& rhs) {
    // std::cout << "comparing Bounds" << std::endl;
    return lhs.min.isApprox(rhs.min)
        && lhs.max.isApprox(rhs.max);
  }
};

struct JointSoftLimitParams {
  JointSoftLimitParams(const RigidBodyTree &robot):
    enabled(Eigen::Matrix<bool, Eigen::Dynamic, 1>::Zero(robot.num_positions)),
    disable_when_body_in_support(Eigen::VectorXi::Zero(robot.num_positions)),
    lb(Eigen::VectorXd::Zero(robot.num_positions)),
    ub(Eigen::VectorXd::Zero(robot.num_positions)),
    kp(Eigen::VectorXd::Zero(robot.num_positions)),
    kd(Eigen::VectorXd::Zero(robot.num_positions)),
    weight(Eigen::VectorXd::Zero(robot.num_positions)),
    k_logistic(Eigen::VectorXd::Zero(robot.num_positions)) {}

  Eigen::Matrix<bool, Eigen::Dynamic, 1> enabled;
  Eigen::VectorXi disable_when_body_in_support;
  Eigen::VectorXd lb;
  Eigen::VectorXd ub;
  Eigen::VectorXd kp;
  Eigen::VectorXd kd;
  Eigen::VectorXd weight;
  Eigen::VectorXd k_logistic;

  friend std::ostream& operator<<(std::ostream& os, const JointSoftLimitParams& params) {
    os << "enabled: " << params.enabled.transpose() << std::endl
       << "disable_when_body_in_support: " << params.disable_when_body_in_support.transpose() << std::endl
       << "lb: " << params.lb.transpose() << std::endl
       << "ub: " << params.ub.transpose() << std::endl
       << "kp: " << params.kp.transpose() << std::endl
       << "kd: " << params.kd.transpose() << std::endl
       << "weight: " << params.weight.transpose() << std::endl
       << "k_logistic: " << params.k_logistic.transpose() << std::endl;
    return os;
  }

  friend bool operator==(const JointSoftLimitParams& lhs, const JointSoftLimitParams& rhs) {
    std::cout << "comparing JointSoftLimitParams" << std::endl;
    bool is_equal = lhs.enabled == rhs.enabled
        && lhs.disable_when_body_in_support == rhs.disable_when_body_in_support
        && lhs.lb == rhs.lb
        && lhs.ub == rhs.ub
        && lhs.kp.isApprox(rhs.kp)
        && lhs.kd.isApprox(rhs.kd)
        && lhs.weight.isApprox(rhs.weight)
        && lhs.k_logistic.isApprox(rhs.k_logistic);

    if (!is_equal) {
      std::cout << "lhs: " << std::endl << lhs << std::endl;
      std::cout << "rhs: " << std::endl << rhs << std::endl;
    }
    return is_equal;
    // std::cout << "lhs: " << std::endl;
    // std::cout << "enabled: " << lhs.enabled.transpose() << std::endl;
    // std::cout << "disable_when_body_in_support: " << lhs.disable_when_body_in_support.transpose() << std::endl;
    // std::cout << "lb: " << lhs.lb.transpose() << std::endl;
    // std::cout << "ub: " << lhs.ub.transpose() << std::endl;
    // std::cout << "kp: " << lhs.kp.transpose() << std::endl;
    // std::cout << "kd: " << lhs.kd.transpose() << std::endl;
    // std::cout << "weight: " << lhs.weight.transpose() << std::endl;
    // std::cout << "k_logistic: " << lhs.k_logistic.transpose() << std::endl;
    // std::cout << "rhs: " << std::endl;
    // std::cout << "enabled: " << rhs.enabled.transpose() << std::endl;
    // std::cout << lhs.enabled.isApprox(rhs.enabled) << std::endl;
    // std::cout << "disable_when_body_in_support: " << rhs.disable_when_body_in_support.transpose() << std::endl;
    // std::cout << lhs.disable_when_body_in_support.isApprox(rhs.disable_when_body_in_support) << std::endl;
    // std::cout << "lb: " << rhs.lb.transpose() << std::endl;
    // std::cout << lhs.lb.isApprox(rhs.lb) << std::endl;
    // std::cout << "ub: " << rhs.ub.transpose() << std::endl;
    // std::cout << lhs.ub.isApprox(rhs.ub) << std::endl;
    // std::cout << "kp: " << rhs.kp.transpose() << std::endl;
    // std::cout << lhs.kp.isApprox(rhs.kp) << std::endl;
    // std::cout << "kd: " << rhs.kd.transpose() << std::endl;
    // std::cout << lhs.kd.isApprox(rhs.kd) << std::endl;
    // std::cout << "weight: " << rhs.weight.transpose() << std::endl;
    // std::cout << lhs.weight.isApprox(rhs.weight) << std::endl;
    // std::cout << "k_logistic: " << rhs.k_logistic.transpose() << std::endl;
    // std::cout << lhs.k_logistic.isApprox(rhs.k_logistic) << std::endl;
  }
};

struct WholeBodyParams {
  WholeBodyParams(const RigidBodyTree &robot):
    Kp(Eigen::VectorXd::Zero(robot.num_positions)),
    Kd(Eigen::VectorXd::Zero(robot.num_positions)),
    w_qdd(Eigen::VectorXd::Zero(robot.num_velocities)),
    integrator(robot),
    qdd_bounds(Eigen::VectorXd::Zero(robot.num_velocities), Eigen::VectorXd::Zero(robot.num_velocities))
     {}

  Eigen::VectorXd Kp;
  Eigen::VectorXd Kd;
  Eigen::VectorXd w_qdd;

  // double damping_ratio;
  IntegratorParams integrator;
  Bounds qdd_bounds;

  friend std::ostream& operator<<(std::ostream& os, const WholeBodyParams& params) {
    os << "Kp: " << params.Kp.transpose() << std::endl
       << "Kd: " << params.Kd.transpose() << std::endl
       << "w_qdd: " << params.w_qdd.transpose() << std::endl
       << "integrator: " << std::endl << params.integrator << std::endl
       << "qdd_bounds: " << std::endl << params.qdd_bounds << std::endl;
    return os;
  }

  friend bool operator==(const WholeBodyParams& lhs, const WholeBodyParams& rhs) {
    std::cout << "comparing WholeBodyParams" << std::endl;
    bool is_equal = lhs.Kp.isApprox(rhs.Kp)
        && lhs.Kd.isApprox(rhs.Kd)
        && lhs.w_qdd.isApprox(rhs.w_qdd)
        && lhs.integrator == rhs.integrator
        && lhs.qdd_bounds == rhs.qdd_bounds;
    if (!is_equal) {
      std::cout << "lhs: " << std::endl << lhs << std::endl
        << "rhs: " << std::endl << rhs << std::endl;
    }
    // std::cout << "lhs: " << std::endl;
    // std::cout << "Kp: " << lhs.Kp.transpose() << std::endl;
    // std::cout << "Kd: " << lhs.Kd.transpose() << std::endl;
    // std::cout << "w_qdd: " << lhs.w_qdd.transpose() << std::endl;
    // std::cout << "rhs: " << std::endl;
    // std::cout << "Kp: " << rhs.Kp.transpose() << std::endl;
    // std::cout << "Kd: " << rhs.Kd.transpose() << std::endl;
    // std::cout << "w_qdd: " << rhs.w_qdd.transpose() << std::endl;
    // std::cout << (lhs.Kp.isApprox(rhs.Kp)) << std::endl;
    // std::cout << (lhs.Kd.isApprox(rhs.Kd)) << std::endl;
    // std::cout << (lhs.w_qdd.isApprox(rhs.w_qdd)) << std::endl;
    // std::cout << (lhs.integrator == rhs.integrator) << std::endl;
    // std::cout << (lhs.qdd_bounds == rhs.qdd_bounds) << std::endl;
    return lhs.Kp.isApprox(rhs.Kp)
        && lhs.Kd.isApprox(rhs.Kd)
        && lhs.w_qdd.isApprox(rhs.w_qdd)
        && lhs.integrator == rhs.integrator
        && lhs.qdd_bounds == rhs.qdd_bounds;
  }
};

struct BodyMotionParams {
  BodyMotionParams():
    Kp(Vector6d::Zero()),
    Kd(Vector6d::Zero()),
    accel_bounds(Eigen::VectorXd::Zero(6), Eigen::VectorXd::Zero(6)),
    weight(0.0) {}

  Vector6d Kp;
  Vector6d Kd;
  Bounds accel_bounds;
  double weight;

  friend bool operator==(const BodyMotionParams& lhs, const BodyMotionParams& rhs) {
    std::cout << "comparing BodyMotionParams" << std::endl;
    // std::cout << "lhs: " << std::endl;
    // std::cout << "Kp: " << lhs.Kp.transpose() << std::endl;
    // std::cout << "Kd: " << lhs.Kd.transpose() << std::endl;
    // std::cout << "rhs: " << std::endl;
    // std::cout << "Kp: " << rhs.Kp.transpose() << std::endl;
    // std::cout << "Kd: " << rhs.Kd.transpose() << std::endl;
    return lhs.Kp.isApprox(rhs.Kp)
        && lhs.Kd.isApprox(rhs.Kd)
        && lhs.accel_bounds == rhs.accel_bounds
        && lhs.weight == rhs.weight;
  }
};

struct HardwareGains {
  HardwareGains(const RigidBodyTree &robot):
    k_f_p(Eigen::VectorXd::Zero(robot.actuators.size())),
    k_q_p(Eigen::VectorXd::Zero(robot.actuators.size())),
    k_q_i(Eigen::VectorXd::Zero(robot.actuators.size())),
    k_qd_p(Eigen::VectorXd::Zero(robot.actuators.size())),
    ff_qd(Eigen::VectorXd::Zero(robot.actuators.size())),
    ff_f_d(Eigen::VectorXd::Zero(robot.actuators.size())),
    ff_const(Eigen::VectorXd::Zero(robot.actuators.size())),
    ff_qd_d(Eigen::VectorXd::Zero(robot.actuators.size())) {}

  Eigen::VectorXd k_f_p;
  Eigen::VectorXd k_q_p;
  Eigen::VectorXd k_q_i;
  Eigen::VectorXd k_qd_p;
  Eigen::VectorXd ff_qd;
  Eigen::VectorXd ff_f_d;
  Eigen::VectorXd ff_const;
  Eigen::VectorXd ff_qd_d;

  friend bool operator==(const HardwareGains& lhs, const HardwareGains& rhs) {
    std::cout << "comparing HardwareGains" << std::endl;
    bool is_equal = lhs.k_f_p.isApprox(rhs.k_f_p)
        && lhs.k_q_p.isApprox(rhs.k_q_p)
        && lhs.k_q_i.isApprox(rhs.k_q_i)
        && lhs.k_qd_p.isApprox(rhs.k_qd_p)
        && lhs.ff_qd.isApprox(rhs.ff_qd)
        && lhs.ff_f_d.isApprox(rhs.ff_f_d)
        && lhs.ff_const.isApprox(rhs.ff_const)
        && lhs.ff_qd_d.isApprox(rhs.ff_qd_d);

    if (!is_equal) {
      std::cout << (lhs.k_f_p.isApprox(rhs.k_f_p)) << std::endl;
      std::cout << "lhs: " << lhs.k_f_p.transpose() << std::endl;
      std::cout << "rhs: " << rhs.k_f_p.transpose() << std::endl;
      std::cout << (lhs.k_q_p.isApprox(rhs.k_q_p)) << std::endl;
      std::cout << "lhs: " << lhs.k_q_p.transpose() << std::endl;
      std::cout << "rhs: " << rhs.k_q_p.transpose() << std::endl;
      std::cout << (lhs.k_q_i.isApprox(rhs.k_q_i)) << std::endl;
      std::cout << (lhs.k_qd_p.isApprox(rhs.k_qd_p)) << std::endl;
      std::cout << "lhs: " << lhs.k_qd_p.transpose() << std::endl;
      std::cout << "rhs: " << rhs.k_qd_p.transpose() << std::endl;
      std::cout << (lhs.ff_qd.isApprox(rhs.ff_qd)) << std::endl;
      std::cout << (lhs.ff_f_d.isApprox(rhs.ff_f_d)) << std::endl;
      std::cout << (lhs.ff_const.isApprox(rhs.ff_const)) << std::endl;
      std::cout << (lhs.ff_qd_d.isApprox(rhs.ff_qd_d)) << std::endl;
      std::cout << "lhs: " << lhs.ff_qd_d.transpose() << std::endl;
      std::cout << "rhs: " << rhs.ff_qd_d.transpose() << std::endl;
    }
    return is_equal;
  }
};

struct HardwareParams {
  HardwareParams(const RigidBodyTree &robot):
    gains(robot),
    joint_is_force_controlled(Eigen::Matrix<bool, Eigen::Dynamic, 1>::Zero(robot.actuators.size())),
    joint_is_position_controlled(Eigen::Matrix<bool, Eigen::Dynamic, 1>::Zero(robot.actuators.size())) {}

  HardwareGains gains;
  Eigen::Matrix<bool, Eigen::Dynamic, 1> joint_is_force_controlled;
  Eigen::Matrix<bool, Eigen::Dynamic, 1> joint_is_position_controlled;

  friend bool operator==(const HardwareParams& lhs, const HardwareParams& rhs) {
    std::cout << "comparing HardwareParams" << std::endl;
    return lhs.gains == rhs.gains
        && lhs.joint_is_force_controlled == rhs.joint_is_force_controlled
        && lhs.joint_is_position_controlled == rhs.joint_is_position_controlled;
    return lhs.gains == rhs.gains
        && lhs.joint_is_force_controlled == rhs.joint_is_force_controlled
        && lhs.joint_is_position_controlled == rhs.joint_is_position_controlled;
  }
};

struct QPControllerParams {
  QPControllerParams(const RigidBodyTree &robot):
    whole_body(robot),
    body_motion(robot.bodies.size()),
    vref_integrator(),
    joint_soft_limits(robot),
    hardware(robot),
    W_kdot(Eigen::Matrix3d::Zero()),
    Kp_ang(0.0),
    w_slack(0.0),
    slack_limit(0.0),
    w_grf(0.0),
    Kp_accel(0.0),
    contact_threshold(0.0),
    min_knee_angle(0.0),
    use_center_of_mass_observer(false),
    center_of_mass_observer_gain(Eigen::Matrix4d::Zero()) {}

  WholeBodyParams whole_body;
  std::vector<BodyMotionParams> body_motion;
  VRefIntegratorParams vref_integrator;
  JointSoftLimitParams joint_soft_limits;
  HardwareParams hardware;
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

  // friend std::ostream& operator<<(std::ostream& os, const QPControllerParams& params) {
  //   os << "whole_body: " << std::endl << params.whole_body << std::endl
  //     << "body_motion: " << std::endl;
  //   for (auto it = body_motion.begin(); it != body_motion.end(); ++it) {
  //     os << it - body_motion.begin() << ": " << std::endl << *it << std::endl;
  //   }
  //   os << "vref_integrator: " << std::endl << params.vref_integrator << std::endl
  //     << "joint_soft_limits: " << std::endl << params.joint_soft_limits << std::endl
  //     << "hardware: " << std::endl << params.hardware << std::endl
  //     << "W_kdot: " << std::endl << params.W_kdot << std::endl
  //     << "Kp_ang: " << std::endl << params.Kp_ang << std::endl
  //     << "w_slack: " << std::endl << params.w_slack << std::endl
  //     << "slack_limit: " << std::endl << params.slack_limit << std::endl
  //     << "w_grf: " << std::endl << params.w_grf << std::endl
  //     << "Kp_accel: " << std::endl << params.Kp_accel << std::endl
  //     << "contact_threshold: " << std::endl << params.contact_threshold << std::endl
  //     << "min_knee_angle: " << std::endl << params.min_knee_angle << std::endl
  //     << "use_center_of_mass_observer: " << std::endl << params.use_center_of_mass_observer << std::endl
  //     << "center_of_mass_observer_gain: " << std::endl << params.center_of_mass_observer_gain << std::endl;
  //   return os;
  // }

  friend bool operator==(const QPControllerParams& lhs, const QPControllerParams& rhs) {
    std::cout << "comparing QPControllerParams" << std::endl;

    bool is_equal = lhs.whole_body == rhs.whole_body
        // && lhs.body_motion == rhs.body_motion
        && lhs.vref_integrator == rhs.vref_integrator
        && lhs.joint_soft_limits == rhs.joint_soft_limits
        && lhs.hardware == rhs.hardware
        && lhs.W_kdot.isApprox(rhs.W_kdot)
        && lhs.Kp_ang == rhs.Kp_ang
        && lhs.w_slack == rhs.w_slack
        && lhs.slack_limit == rhs.slack_limit
        && lhs.w_grf == rhs.w_grf
        && lhs.Kp_accel == rhs.Kp_accel
        && lhs.contact_threshold == rhs.contact_threshold
        && lhs.min_knee_angle == rhs.min_knee_angle
        && lhs.use_center_of_mass_observer == rhs.use_center_of_mass_observer
        && lhs.center_of_mass_observer_gain.isApprox(rhs.center_of_mass_observer_gain);


    std::cout << (lhs.joint_soft_limits == rhs.joint_soft_limits) << std::endl;
    std::cout << (lhs.hardware == rhs.hardware) << std::endl;
    std::cout << (lhs.W_kdot.isApprox(rhs.W_kdot)) << std::endl;
    std::cout << (lhs.Kp_ang == rhs.Kp_ang) << std::endl;
    std::cout << (lhs.w_slack == rhs.w_slack) << std::endl;
    std::cout << (lhs.slack_limit == rhs.slack_limit) << std::endl;
    std::cout << (lhs.w_grf == rhs.w_grf) << std::endl;
    std::cout << (lhs.Kp_accel == rhs.Kp_accel) << std::endl;
    std::cout << (lhs.contact_threshold == rhs.contact_threshold) << std::endl;
    std::cout << (lhs.min_knee_angle == rhs.min_knee_angle) << std::endl;
    std::cout << (lhs.use_center_of_mass_observer == rhs.use_center_of_mass_observer) << std::endl;
    std::cout << (lhs.center_of_mass_observer_gain.isApprox(rhs.center_of_mass_observer_gain)) << std::endl;

    return is_equal;
  }
};

class NewQPControllerData {
public:
  GRBenv *env;
  std::unique_ptr<RigidBodyTree> r;
  std::map<std::string,QPControllerParams> param_sets;
  RobotPropertyCache rpc;
  void* map_ptr;
  Eigen::VectorXd umin,umax;
  int use_fast_qp;
  JointNames input_joint_names;

  // preallocate memory
  KinematicsCache<double> cache;
  Eigen::MatrixXd H, H_float, H_act;
  Eigen::VectorXd C, C_float, C_act;
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

  NewQPControllerData(std::unique_ptr<RigidBodyTree> r) :
      r(std::move(r)), cache(this->r->bodies)
  {
    // empty
  }
};

struct DesiredBodyAcceleration {
  DesiredBodyAcceleration():
    accel_bounds(Eigen::VectorXd(6), Eigen::VectorXd(6)) {}
    
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

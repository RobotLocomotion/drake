#pragma once

#include <algorithm>
#include <map>
#include <set>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "drake/common/eigen_stl_types.h"
#include "drake/systems/controllers/controlUtil.h"
#include "drake/systems/plants/ForceTorqueMeasurement.h"
#include "drake/systems/plants/RigidBodyTree.h"
#include "drake/systems/plants/joints/floating_base_types.h"
#include "drake/systems/robotInterfaces/Side.h"
#include "drake/util/drakeUtil.h"

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
  int* vbasis;
  int* cbasis;
  int vbasis_len;
  int cbasis_len;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
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
  VRefIntegratorParams()
      : zero_ankles_on_contact(false), eta(0.0), delta_max(0.0) {}

  bool zero_ankles_on_contact;
  double eta;
  double delta_max;

  friend bool operator==(const VRefIntegratorParams& lhs,
                         const VRefIntegratorParams& rhs) {
    return lhs.zero_ankles_on_contact == rhs.zero_ankles_on_contact &&
           lhs.eta == rhs.eta && lhs.delta_max == rhs.delta_max;
  }
};

struct IntegratorParams {
  explicit IntegratorParams(const RigidBodyTree& robot)
      : gains(Eigen::VectorXd::Zero(robot.get_num_positions())),
        clamps(Eigen::VectorXd::Zero(robot.get_num_positions())),
        eta(0.0) {}

  Eigen::VectorXd gains;
  Eigen::VectorXd clamps;
  double eta;

  friend bool operator==(const IntegratorParams& lhs,
                         const IntegratorParams& rhs) {
    return lhs.gains.isApprox(rhs.gains) && lhs.clamps.isApprox(rhs.clamps) &&
           lhs.eta == rhs.eta;
  }
};

struct Bounds {
  Bounds(const Eigen::Ref<const Eigen::VectorXd>& min_,
         const Eigen::Ref<const Eigen::VectorXd>& max_)
      : min(min_), max(max_) {}

  Eigen::VectorXd min;
  Eigen::VectorXd max;

  friend bool operator==(const Bounds& lhs, const Bounds& rhs) {
    return lhs.min.isApprox(rhs.min) && lhs.max.isApprox(rhs.max);
  }
};

struct JointSoftLimitParams {
  explicit JointSoftLimitParams(const RigidBodyTree& robot)
      : enabled(Eigen::Matrix<bool, Eigen::Dynamic, 1>::Zero(
            robot.get_num_positions())),
        disable_when_body_in_support(
            Eigen::VectorXi::Zero(robot.get_num_positions())),
        lb(Eigen::VectorXd::Zero(robot.get_num_positions())),
        ub(Eigen::VectorXd::Zero(robot.get_num_positions())),
        kp(Eigen::VectorXd::Zero(robot.get_num_positions())),
        kd(Eigen::VectorXd::Zero(robot.get_num_positions())),
        weight(Eigen::VectorXd::Zero(robot.get_num_positions())),
        k_logistic(Eigen::VectorXd::Zero(robot.get_num_positions())) {}

  Eigen::Matrix<bool, Eigen::Dynamic, 1> enabled;
  Eigen::VectorXi disable_when_body_in_support;
  Eigen::VectorXd lb;
  Eigen::VectorXd ub;
  Eigen::VectorXd kp;
  Eigen::VectorXd kd;
  Eigen::VectorXd weight;
  Eigen::VectorXd k_logistic;

  friend bool operator==(const JointSoftLimitParams& lhs,
                         const JointSoftLimitParams& rhs) {
    bool is_equal =
        lhs.enabled == rhs.enabled &&
        lhs.disable_when_body_in_support == rhs.disable_when_body_in_support &&
        lhs.lb == rhs.lb && lhs.ub == rhs.ub && lhs.kp.isApprox(rhs.kp) &&
        lhs.kd.isApprox(rhs.kd) && lhs.weight.isApprox(rhs.weight) &&
        lhs.k_logistic.isApprox(rhs.k_logistic);
    return is_equal;
  }
};

struct WholeBodyParams {
  explicit WholeBodyParams(const RigidBodyTree& robot)
      : Kp(Eigen::VectorXd::Zero(robot.get_num_positions())),
        Kd(Eigen::VectorXd::Zero(robot.get_num_positions())),
        w_qdd(Eigen::VectorXd::Zero(robot.get_num_velocities())),
        integrator(robot),
        qdd_bounds(Eigen::VectorXd::Zero(robot.get_num_velocities()),
                   Eigen::VectorXd::Zero(robot.get_num_velocities())) {}

  Eigen::VectorXd Kp;
  Eigen::VectorXd Kd;
  Eigen::VectorXd w_qdd;

  // double damping_ratio;
  IntegratorParams integrator;
  Bounds qdd_bounds;

  friend bool operator==(const WholeBodyParams& lhs,
                         const WholeBodyParams& rhs) {
    return lhs.Kp.isApprox(rhs.Kp) && lhs.Kd.isApprox(rhs.Kd) &&
           lhs.w_qdd.isApprox(rhs.w_qdd) && lhs.integrator == rhs.integrator &&
           lhs.qdd_bounds == rhs.qdd_bounds;
  }
};

struct BodyMotionParams {
  BodyMotionParams()
      : Kp(Vector6d::Zero()),
        Kd(Vector6d::Zero()),
        accel_bounds(Eigen::VectorXd::Zero(6), Eigen::VectorXd::Zero(6)),
        weight(0.0) {}

  Vector6d Kp;
  Vector6d Kd;
  Bounds accel_bounds;
  double weight;

  friend bool operator==(const BodyMotionParams& lhs,
                         const BodyMotionParams& rhs) {
    return lhs.Kp.isApprox(rhs.Kp) && lhs.Kd.isApprox(rhs.Kd) &&
           lhs.accel_bounds == rhs.accel_bounds && lhs.weight == rhs.weight;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct HardwareGains {
  explicit HardwareGains(const RigidBodyTree& robot)
      : k_f_p(Eigen::VectorXd::Zero(robot.actuators.size())),
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
    bool is_equal =
        lhs.k_f_p.isApprox(rhs.k_f_p) && lhs.k_q_p.isApprox(rhs.k_q_p) &&
        lhs.k_q_i.isApprox(rhs.k_q_i) && lhs.k_qd_p.isApprox(rhs.k_qd_p) &&
        lhs.ff_qd.isApprox(rhs.ff_qd) && lhs.ff_f_d.isApprox(rhs.ff_f_d) &&
        lhs.ff_const.isApprox(rhs.ff_const) &&
        lhs.ff_qd_d.isApprox(rhs.ff_qd_d);
    return is_equal;
  }
};

struct HardwareParams {
  explicit HardwareParams(const RigidBodyTree& robot)
      : gains(robot),
        joint_is_force_controlled(Eigen::Matrix<bool, Eigen::Dynamic, 1>::Zero(
            robot.actuators.size())),
        joint_is_position_controlled(
            Eigen::Matrix<bool, Eigen::Dynamic, 1>::Zero(
                robot.actuators.size())) {}

  HardwareGains gains;
  Eigen::Matrix<bool, Eigen::Dynamic, 1> joint_is_force_controlled;
  Eigen::Matrix<bool, Eigen::Dynamic, 1> joint_is_position_controlled;

  friend bool operator==(const HardwareParams& lhs, const HardwareParams& rhs) {
    return lhs.gains == rhs.gains &&
           lhs.joint_is_force_controlled == rhs.joint_is_force_controlled &&
           lhs.joint_is_position_controlled == rhs.joint_is_position_controlled;
  }
};

struct QPControllerParams {
  explicit QPControllerParams(const RigidBodyTree& robot)
      : whole_body(robot),
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
  drake::eigen_aligned_std_vector<BodyMotionParams> body_motion;
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

  friend bool operator==(const QPControllerParams& lhs,
                         const QPControllerParams& rhs) {
    bool is_equal =
        lhs.whole_body == rhs.whole_body
        // && lhs.body_motion == rhs.body_motion
        && lhs.vref_integrator == rhs.vref_integrator &&
        lhs.joint_soft_limits == rhs.joint_soft_limits &&
        lhs.hardware == rhs.hardware && lhs.W_kdot.isApprox(rhs.W_kdot) &&
        lhs.Kp_ang == rhs.Kp_ang && lhs.w_slack == rhs.w_slack &&
        lhs.slack_limit == rhs.slack_limit && lhs.w_grf == rhs.w_grf &&
        lhs.Kp_accel == rhs.Kp_accel &&
        lhs.contact_threshold == rhs.contact_threshold &&
        lhs.min_knee_angle == rhs.min_knee_angle &&
        lhs.use_center_of_mass_observer == rhs.use_center_of_mass_observer &&
        lhs.center_of_mass_observer_gain.isApprox(
            rhs.center_of_mass_observer_gain);
    return is_equal;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

std::unordered_map<std::string, int> computeBodyOrFrameNameToIdMap(
    const RigidBodyTree& robot);

struct DesiredBodyAcceleration {
  DesiredBodyAcceleration()
      : accel_bounds(Eigen::VectorXd(6), Eigen::VectorXd(6)) {}

  int body_or_frame_id0;
  Vector6d body_vdot;
  double weight;
  Bounds accel_bounds;
  bool control_pose_when_in_contact;
  bool use_spatial_velocity;
  KinematicPath body_path;
  Eigen::Isometry3d T_task_to_world;
  Vector6d weight_multiplier;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct QPControllerOutput {
  Eigen::VectorXd q_ref;
  Eigen::VectorXd qd_ref;
  Eigen::VectorXd qdd;
  Eigen::VectorXd u;
};

struct QPControllerDebugData {
  drake::eigen_aligned_std_vector<SupportStateElement> active_supports;
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

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct PIDOutput {
  Eigen::VectorXd q_ref;
  Eigen::VectorXd qddot_des;
};

// enum PlanShiftMode {NONE, XYZ, Z_ONLY, Z_AND_ZMP};

class Attachment {
 public:
  std::string attach_to_frame;
  std::string urdf_filename;
  drake::systems::plants::joints::FloatingBaseType joint_type;

  Attachment(
      const std::string& attach_to_frame_, const std::string& urdf_filename_,
      const drake::systems::plants::joints::FloatingBaseType&
          joint_type_ = drake::systems::plants::joints::kFixed)
      : attach_to_frame(attach_to_frame_),
        urdf_filename(urdf_filename_),
        joint_type(joint_type_) {
    // empty
  }

  Attachment() {
    // empty no-argument constructor for YAML::decode
  }
};

class KinematicModifications {
 public:
  std::set<std::string> collision_groups_to_keep;
  std::vector<Attachment> attachments;

  KinematicModifications(
      const std::set<std::string>& collision_groups_to_keep_ = {"heel", "toe"},
      const std::vector<Attachment>& attachments_ = std::vector<Attachment>())
      : collision_groups_to_keep(collision_groups_to_keep_),
        attachments(attachments_) {
    // empty
  }
};

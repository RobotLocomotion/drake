#pragma once

#include <map>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <lcm/lcm-cpp.hpp>

#include "drake/systems/trajectories/PiecewisePolynomial.h"
#include "drake/systems/trajectories/ExponentialPlusPiecewisePolynomial.h"
#include "drake/systems/plants/RigidBodyTree.h"
#include "drake/lcmt_qp_controller_input.hpp"
#include "BodyMotionData.h"
#include "drake/systems/robotInterfaces/Side.h"
#include "drake/systems/controllers/zmpUtil.h"
#include "drake/common/drake_export.h"

class QuadraticLyapunovFunction {
  // TODO(tkoolen): move into its own file
  // TODO(tkoolen): make part of a Lyapunov function class hierarchy
  // TODO(tkoolen): more functionality
 private:
  Eigen::MatrixXd S_;
  ExponentialPlusPiecewisePolynomial<double> s1_;

 public:
  QuadraticLyapunovFunction() {}

  template <typename DerivedS>
  QuadraticLyapunovFunction(
      const Eigen::MatrixBase<DerivedS>& S,
      const ExponentialPlusPiecewisePolynomial<double>& s1)
      : S_(S), s1_(s1) {}

  const Eigen::MatrixXd& getS() const { return S_; }

  const ExponentialPlusPiecewisePolynomial<double>& getS1() const {
    return s1_;
  }
  void setS1(const ExponentialPlusPiecewisePolynomial<double>& new_s1) {
    s1_ = new_s1;
  }
};

struct RigidBodySupportStateElement {
  // TODO(tkoolen): turn this into a class with more functionality
  // TODO(tkoolen): consolidate with SupportStateElement?
  int body;  // TODO(tkoolen): should probably be a RigidBody smart pointer
  Eigen::Matrix3Xd contact_points;
  bool use_contact_surface;
  // TODO(tkoolen): should probably be a different type
  Eigen::Vector4d support_surface;
};

typedef std::vector<RigidBodySupportStateElement> RigidBodySupportState;

enum SupportLogicType {
  REQUIRE_SUPPORT,
  ONLY_IF_FORCE_SENSED,
  ONLY_IF_KINEMATIC,
  KINEMATIC_OR_SENSED,
  PREVENT_SUPPORT
};

struct KneeSettings {
  double min_knee_angle;
  double knee_kp;
  double knee_kd;
  double knee_weight;
};

struct QPLocomotionPlanSettings {
  QPLocomotionPlanSettings() : min_foot_shift_delay(0.1) {}

  double duration;
  std::vector<RigidBodySupportState> supports;
  std::vector<double> support_times;  // length: supports.size() + 1
  typedef std::map<std::string, Eigen::Matrix3Xd> ContactNameToContactPointsMap;
  std::vector<ContactNameToContactPointsMap>
      contact_groups;  // one for each RigidBody
  std::vector<bool> planned_support_command;
  double early_contact_allowed_fraction;

  std::vector<BodyMotionData> body_motions;
  PiecewisePolynomial<double> zmp_trajectory;
  TVLQRData zmp_data;
  Eigen::MatrixXd D_control;
  QuadraticLyapunovFunction V;
  PiecewisePolynomial<double> q_traj;
  ExponentialPlusPiecewisePolynomial<double> com_traj;

  std::string gain_set;
  double mu;
  bool use_plan_shift;
  std::vector<Eigen::Index> plan_shift_body_motion_indices;
  double g;
  double min_foot_shift_delay;  // seconds to wait before updating foot-specific
                                // plan shifts (default value set in the
                                // constructor above)
  bool is_quasistatic;
  KneeSettings knee_settings;
  double ankle_limits_tolerance;
  std::string pelvis_name;
  std::map<Side, std::string> foot_names;
  std::map<Side, std::string> knee_names;
  std::map<Side, std::string> aky_names;
  std::map<Side, std::string> akx_names;
  double zmp_safety_margin;
  std::vector<int> constrained_position_indices;
  std::vector<int> untracked_position_indices;

  void addSupport(
      const RigidBodySupportState& support_state,
      const ContactNameToContactPointsMap& contact_group_name_to_contact_points,
      double duration_in) {
    supports.push_back(support_state);
    contact_groups.push_back(contact_group_name_to_contact_points);
    if (support_times.empty()) support_times.push_back(0.0);
    support_times.push_back(support_times[support_times.size() - 1] + duration);
  }

  // may be useful later
  static KneeSettings createDefaultKneeSettings() {
    KneeSettings knee_settings;
    knee_settings.min_knee_angle = 0.7;
    knee_settings.knee_kp = 40.0;
    knee_settings.knee_kd = 4.0;
    knee_settings.knee_weight = 1.0;
    return knee_settings;
  }

  // may be useful later in setting up constrained_position_indices
  static std::vector<int> findPositionIndices(
      // TODO(#2274) Fix this NOLINTNEXTLINE(runtime/references).
      RigidBodyTree& robot,
      const std::vector<std::string>& joint_name_substrings) {
    std::vector<int> ret;
    for (auto body_it = robot.bodies.begin(); body_it != robot.bodies.end();
         ++body_it) {
      RigidBody& body = **body_it;
      if (body.has_parent_body()) {
        const DrakeJoint& joint = body.getJoint();
        for (auto joint_name_it = joint_name_substrings.begin();
             joint_name_it != joint_name_substrings.end(); ++joint_name_it) {
          if (joint.get_name().find(*joint_name_it) != std::string::npos) {
            for (int i = 0; i < joint.get_num_positions(); i++) {
              ret.push_back(body.get_position_start_index() + i);
            }
            break;
          }
        }
      }
    }
    return ret;
  }
};

class DRAKE_EXPORT QPLocomotionPlan {
 private:
  RigidBodyTree& robot_;  // TODO(tkoolen): const correctness
  QPLocomotionPlanSettings settings_;
  const std::map<Side, int> foot_body_ids_;
  const std::map<Side, int> knee_indices_;
  const std::map<Side, int> aky_indices_;
  const std::map<Side, int> akx_indices_;
  const int pelvis_id_;

  lcm::LCM lcm_;
  std::string lcm_channel_;

  double start_time_;
  Eigen::Vector3d plan_shift_;
  std::map<Side, Eigen::Vector3d> foot_shifts_;
  double last_foot_shift_time_;
  drake::lcmt_qp_controller_input last_qp_input_;
  std::map<Side, bool> toe_off_active_;
  std::map<Side, bool> knee_pd_active_;
  std::map<Side, KneeSettings> knee_pd_settings_;
  PiecewisePolynomial<double> shifted_zmp_trajectory_;

  /*
   * when the plan says a given body is in support, require the controller to
   * use that support.
   * To allow the controller to use that support only if it thinks the body is
   * in contact with the terrain, try KINEMATIC_OR_SENSED
   */
  static const std::map<SupportLogicType, std::vector<bool> >
      support_logic_maps_;

 public:
  // TODO(#2274) Fix this NOLINTNEXTLINE(runtime/references).
  QPLocomotionPlan(RigidBodyTree& robot,
                   const QPLocomotionPlanSettings& settings,
                   const std::string& lcm_channel);

  /*
   * Get the input structure which can be passed to the stateless QP control
   * loop
   * @param t the current time
   * @param q the current robot configuration
   * @param v the current robot velocity
   * @param contact_force_detected num_bodies vector indicating whether contact
   * force
   */
  template <typename DerivedQ, typename DerivedV>
  drake::lcmt_qp_controller_input createQPControllerInput(
      double t_global, const Eigen::MatrixBase<DerivedQ>& q,
      const Eigen::MatrixBase<DerivedV>& v,
      const std::vector<bool>& contact_force_detected);

  void setDuration(double duration);

  void setStartTime(double start_time);

  double getStartTime() const;

  double getDuration() const;

  bool isFinished(double t) const;

  drake::lcmt_qp_controller_input getLastQPInput() const;

  const RigidBodyTree& getRobot() const;

 private:
  drake::lcmt_zmp_data createZMPData(double t_plan) const;

  drake::lcmt_support_data createSupportDataElement(
      const RigidBodySupportStateElement& element,
      const std::vector<bool>& support_logic);

  bool isSupportingBody(int body_index,
                        const RigidBodySupportState& support_state) const;

  std::vector<Side> getSupportSides(
      const RigidBodySupportState& support_state) const;

  // TODO(#2274) Fix this NOLINTNEXTLINE(runtime/references).
  void updateSwingTrajectory(double t_plan, BodyMotionData& body_motion_data,
                             int body_motion_segment_index,
                             const KinematicsCache<double>& cache);

  void updatePlanShift(const KinematicsCache<double>& cache, double t_plan,
                       const std::vector<bool>& contact_force_detected,
                       int support_index);

  void findPlannedSupportFraction(
      double t_plan, int support_index,
      // TODO(#2274) Fix this NOLINTNEXTLINE(runtime/references).
      double& last_support_fraction,
      // TODO(#2274) Fix this NOLINTNEXTLINE(runtime/references).
      double& next_support_fraction,
      // TODO(#2274) Fix this NOLINTNEXTLINE(runtime/references).
      double& transition_fraction) const;

  void updateZMPController(const double t_plan,
                           const double last_support_fraction,
                           const double next_support_fraction,
                           const double transition_fraction);

  void updateZMPTrajectory(const double t_plan,
                           const double last_support_fraction,
                           const double next_support_fraction,
                           const double transition_fraction);

  void updateS1Trajectory();

  // TODO(#2274) Fix this NOLINTNEXTLINE(runtime/references).
  void applyKneePD(Side side, drake::lcmt_qp_controller_input& qp_input);

  static const std::map<SupportLogicType, std::vector<bool> >
  createSupportLogicMaps();

  static const std::map<Side, int> createFootBodyIdMap(
      const RigidBodyTree& robot,
      const std::map<Side, std::string>& foot_names);

  static const std::map<Side, int> createJointIndicesMap(
      // TODO(#2274) Fix this NOLINTNEXTLINE(runtime/references).
      RigidBodyTree& robot, const std::map<Side, std::string>& foot_body_ids);
};

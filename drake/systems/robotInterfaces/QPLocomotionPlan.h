#ifndef SYSTEMS_ROBOTINTERFACES_QPLOCOMOTIONPLAN_H_
#define SYSTEMS_ROBOTINTERFACES_QPLOCOMOTIONPLAN_H_

#include <vector>
#include <map>
#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "PiecewisePolynomial.h"
#include "ExponentialPlusPiecewisePolynomial.h"
#include "RigidBodyManipulator.h"
#include "lcmtypes/drake/lcmt_qp_controller_input.hpp"
#include "BodyMotionData.h"
#include "Side.h"
#include <lcm/lcm-cpp.hpp>
#include "zmpUtil.h"
 

class QuadraticLyapunovFunction {
  // TODO: move into its own file
  // TODO: make part of a Lyapunov function class hierarchy
  // TODO: more functionality
private:
  Eigen::MatrixXd S;
  ExponentialPlusPiecewisePolynomial<double> s1;

public:
  QuadraticLyapunovFunction() { }

  template<typename DerivedS>
  QuadraticLyapunovFunction(const Eigen::MatrixBase<DerivedS>& S, const ExponentialPlusPiecewisePolynomial<double>& s1) :
      S(S), s1(s1) { }

  const Eigen::MatrixXd& getS() const
  {
    return S;
  }

  const ExponentialPlusPiecewisePolynomial<double>& getS1() const
  {
    return s1;
  }
  void setS1(ExponentialPlusPiecewisePolynomial<double>& new_s1)
  {
    s1 = new_s1;
  }

};

struct RigidBodySupportStateElement {
  // TODO: turn this into a class with more functionality
  // TODO: consolidate with SupportStateElement?
  int body; // TODO: should probably be a RigidBody smart pointer
  Eigen::Matrix3Xd contact_points;
  bool use_contact_surface;
  Eigen::Vector4d support_surface; // TODO: should probably be a different type
};

typedef std::vector<RigidBodySupportStateElement> RigidBodySupportState;


enum SupportLogicType {
  REQUIRE_SUPPORT, ONLY_IF_FORCE_SENSED, ONLY_IF_KINEMATIC, KINEMATIC_OR_SENSED, PREVENT_SUPPORT
};

struct KneeSettings {
  double min_knee_angle;
  double knee_kp;
  double knee_kd;
  double knee_weight;
};

struct QPLocomotionPlanSettings {

  QPLocomotionPlanSettings() : min_foot_shift_delay(0.1) {};

  double duration;
  std::vector<RigidBodySupportState> supports;
  std::vector<double> support_times; // length: supports.size() + 1
  typedef std::map<std::string, Eigen::Matrix3Xd> ContactNameToContactPointsMap;
  std::vector<ContactNameToContactPointsMap> contact_groups; // one for each RigidBody
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
  std::vector<Eigen::DenseIndex> plan_shift_body_motion_indices;
  double g;
  double min_foot_shift_delay; // seconds to wait before updating foot-specific plan shifts (default value set in the constructor above)
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

  void addSupport(const RigidBodySupportState& support_state, const ContactNameToContactPointsMap& contact_group_name_to_contact_points, double duration) {
    supports.push_back(support_state);
    contact_groups.push_back(contact_group_name_to_contact_points);
    if (support_times.empty())
      support_times.push_back(0.0);
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
  static std::vector<int> findPositionIndices(RigidBodyManipulator& robot, const std::vector<std::string>& joint_name_substrings)
  {
    std::vector<int> ret;
    for (auto body_it = robot.bodies.begin(); body_it != robot.bodies.end(); ++body_it) {
      RigidBody& body = **body_it;
      if (body.hasParent()) {
        const DrakeJoint& joint = body.getJoint();
        for (auto joint_name_it = joint_name_substrings.begin(); joint_name_it != joint_name_substrings.end(); ++joint_name_it) {
          if (joint.getName().find(*joint_name_it) != std::string::npos) {
            for (int i = 0; i < joint.getNumPositions(); i++) {
              ret.push_back(body.position_num_start + i);
            }
            break;
          }
        }
      }
    }
    return ret;
  }

};

class QPLocomotionPlan
{
private:
  RigidBodyManipulator& robot; // TODO: const correctness
  QPLocomotionPlanSettings settings;
  const std::map<Side, int> foot_body_ids;
  const std::map<Side, int> knee_indices;
  const std::map<Side, int> aky_indices;
  const std::map<Side, int> akx_indices;
  const int pelvis_id;

  lcm::LCM lcm;
  std::string lcm_channel;

  double start_time;
  Eigen::Vector3d plan_shift;
  std::map<Side,Eigen::Vector3d> foot_shifts;
  double last_foot_shift_time;
  drake::lcmt_qp_controller_input last_qp_input;
  std::map<Side, bool> toe_off_active;
  std::map<Side, bool> knee_pd_active;
  std::map<Side, KneeSettings> knee_pd_settings;
  PiecewisePolynomial<double> shifted_zmp_trajectory;

  /*
   * when the plan says a given body is in support, require the controller to use that support.
   * To allow the controller to use that support only if it thinks the body is in contact with the terrain, try KINEMATIC_OR_SENSED
   */
  const static std::map<SupportLogicType, std::vector<bool> > support_logic_maps;

public:
  QPLocomotionPlan(RigidBodyManipulator& robot, const QPLocomotionPlanSettings& settings, const std::string& lcm_channel);

  /*
   * Get the input structure which can be passed to the stateless QP control loop
   * @param t the current time
   * @param q the current robot configuration
   * @param v the current robot velocity
   * @param contact_force_detected num_bodies vector indicating whether contact force
   */
  template <typename DerivedQ, typename DerivedV>
  drake::lcmt_qp_controller_input createQPControllerInput(double t_global, const Eigen::MatrixBase<DerivedQ>& q, const Eigen::MatrixBase<DerivedV>& v, const std::vector<bool>& contact_force_detected);

  void setDuration(double duration);

  void setStartTime(double start_time);

  double getStartTime() const;

  double getDuration() const;

  bool isFinished(double t) const;

  drake::lcmt_qp_controller_input getLastQPInput() const;

  const RigidBodyManipulator& getRobot() const;

private:
  drake::lcmt_zmp_data createZMPData(double t_plan) const;

  drake::lcmt_support_data createSupportDataElement(const RigidBodySupportStateElement& element, const std::vector<bool>& support_logic);

  bool isSupportingBody(int body_index, const RigidBodySupportState& support_state) const;

  std::vector<Side> getSupportSides(const RigidBodySupportState &support_state) const;

  void updateSwingTrajectory(double t_plan, BodyMotionData& body_motion_data, int body_motion_segment_index, const KinematicsCache<double>& cache, const Eigen::VectorXd& v);

  void updatePlanShift(const KinematicsCache<double>& cache, double t_plan, const std::vector<bool>& contact_force_detected, int support_index);

  void findPlannedSupportFraction(double t_plan, int support_index, double &last_support_fraction, double &next_support_fraction, double &transition_fraction) const;

  void updateZMPController(const double t_plan, const double last_support_fraction, const double next_support_fraction, const double transition_fraction);

  void updateZMPTrajectory(const double t_plan, const double last_support_fraction, const double next_support_fraction, const double transition_fraction);

  void updateS1Trajectory();

  void applyKneePD(Side side, drake::lcmt_qp_controller_input &qp_input);

  static const std::map<SupportLogicType, std::vector<bool> > createSupportLogicMaps();

  static const std::map<Side, int> createFootBodyIdMap(RigidBodyManipulator& robot, const std::map<Side, std::string>& foot_names);

  static const std::map<Side, int> createJointIndicesMap(RigidBodyManipulator& robot, const std::map<Side, std::string>& foot_body_ids);
};

#endif /* SYSTEMS_ROBOTINTERFACES_QPLOCOMOTIONPLAN_H_ */

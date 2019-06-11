#pragma once

#include <vector>

#include <Eigen/Dense>

#include "drake/common/drake_optional.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/common/trajectories/piecewise_quaternion.h"

namespace drake {
namespace manipulation {
namespace robot_plan_runner {
namespace robot_plans {

/*
 * KEmptyPlan has to be the first in the enumeration. This is because in
 * robot_plan_runner, integer values of the enums are used to index input
 * ports of the PortSwitch system. As port 0 is reserved for the port_selector
 * input port, indices of user-defined ports should start with 1.
 *
 * kLastElement is used to detect the end of this enum when it is looped
 * through.
 */
enum class PlanType { kEmptyPlan, kTaskSpacePlan, kJointSpacePlan,
    kLastElement };

struct PlanData {
  PlanType plan_type{PlanType::kEmptyPlan};
  // Plan signature should be different for every unique plan. It can be
  //  1,2,3, ..., as in the case of running simulations, or
  //  the integer timestamp of the robot_plan_t message received.
  long plan_signature{-1};
  optional<trajectories::PiecewisePolynomial<double>> joint_traj;
  struct EeData {
    // Coordinates of point Q expressed in frame T (task frame).
    Eigen::Vector3d p_ToQ_T;
    // Reference Cartesian coordinates of point Q in world frame relative to
    // the position of Q at the beginning of the plan:
    // p_WoQ_W_ref = ee_xyz_traj.value(t) + p_WoQ_W_t0
    trajectories::PiecewisePolynomial<double> ee_xyz_traj;
    // Reference orientation (\in SO(3)) of frame T in world frame as a
    // quaternion: Q_WT_ref.
    trajectories::PiecewiseQuaternionSlerp<double> ee_quat_traj;
  };
  optional<EeData> ee_data;

  double get_duration() const {
    if (joint_traj.has_value()) {
      return joint_traj.value().end_time();
    } else if (ee_data.has_value()) {
      // TODO(pangtao22): throw if durations of ee_xyz_traj and ee_quat_traj
      //  are different.
      return ee_data.value().ee_xyz_traj.end_time();
    } else {
      throw "invalid PlanData.";
    }
  };
};

/*
 * Abstract base class for all concrete plans.
 */
class PlanBase {
 public:
  PlanBase(PlanType plan_type, int num_positions)
      : num_positions_(num_positions), plan_type_(plan_type){};
  virtual ~PlanBase() = default;

  /*
   * This function is called in controller subsystems of RobotPlanRunner. It
   * takes current robot state (q, v, tau_external), current time (t) and
   * current plan to be executed (plan_data). It writes q and tau_external
   * commands to the pointers provided.
   */
  virtual void Step(const Eigen::Ref<const Eigen::VectorXd>& q,
                    const Eigen::Ref<const Eigen::VectorXd>& v,
                    const Eigen::Ref<const Eigen::VectorXd>& tau_external,
                    double control_period, double t, const PlanData& plan_data,
                    EigenPtr<Eigen::VectorXd> q_commanded,
                    EigenPtr<Eigen::VectorXd> tau_commanded) const = 0;
  PlanType get_plan_type() const { return plan_type_; };

 protected:
  const int num_positions_;
  const PlanType plan_type_;
};

}  // namespace robot_plans
}  // namespace robot_plan_runner
}  // namespace manipulation
}  // namespace drake

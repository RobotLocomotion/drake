#pragma once

#include <vector>

#include <Eigen/Dense>

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/common/trajectories/piecewise_quaternion.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace manipulation {
namespace robot_plan_runner {

enum class PlanType { kJointSpacePlan, kTaskSpacePlan, kEmptyPlan };

struct PlanData {
  PlanType plan_type{PlanType::kEmptyPlan};
  // plan signature can be 1,2,3 (as in the case of running simulations)
  // or timestamp (as in the case of receiving LCM messages).
  long plan_signature{-1};
  optional<trajectories::PiecewisePolynomial<double>> joint_traj;
  struct EeData {
    trajectories::PiecewisePolynomial<double> ee_xyz_traj;
    trajectories::PiecewiseQuaternionSlerp<double> ee_quat_traj;
  };
  optional<EeData> ee_traj;
};

class PlanBase {
 public:
  PlanBase() = default;
  virtual ~PlanBase() = default;

  virtual void Step(const Eigen::Ref<const Eigen::VectorXd>& q,
                    const Eigen::Ref<const Eigen::VectorXd>& v,
                    const Eigen::Ref<const Eigen::VectorXd>& tau_external,
                    double t, const PlanData& plan_data,
                    Eigen::VectorXd* const q_commanded,
                    Eigen::VectorXd* const tau_commanded) const = 0;
  virtual PlanType get_plan_type() const = 0;
};

class JointSpacePlan : public PlanBase {
 public:
  JointSpacePlan() : num_positions_(7){};

  void Step(const Eigen::Ref<const Eigen::VectorXd>& q,
            const Eigen::Ref<const Eigen::VectorXd>& v,
            const Eigen::Ref<const Eigen::VectorXd>& tau_external, double t,
            const PlanData& plan_data, Eigen::VectorXd* const q_cmd,
            Eigen::VectorXd* const tau_cmd) const override;

  inline PlanType get_plan_type() const override {
    return PlanType::kJointSpacePlan;
  };

 private:
  const int num_positions_;
};


}  // namespace robot_plan_runner
}  // namespace manipulation
}  // namespace drake

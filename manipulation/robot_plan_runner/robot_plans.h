#pragma once

#include <vector>

#include <Eigen/Dense>

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/common/trajectories/piecewise_quaternion.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace manipulation {
namespace robot_plan_runner {

enum class PlanType { kJointSpaceController, kTaskSpaceController };

struct PlanData {
  PlanType plan_type;
  int plan_index;
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
  virtual ~PlanBase(){};

  virtual void Step(const Eigen::Ref<const Eigen::VectorXd>& q,
                    const Eigen::Ref<const Eigen::VectorXd>& v,
                    const Eigen::Ref<const Eigen::VectorXd>& tau_external,
                    double t, Eigen::VectorXd* const q_commanded,
                    Eigen::VectorXd* const tau_commanded) const = 0;
  virtual void UpdatePlan(const PlanData& plan_data) = 0;

};

class JointSpacePlan : public PlanBase {
 public:
  JointSpacePlan() : num_positions_(7){};

  void UpdatePlan(const PlanData& plan_data) override;

  void Step(const Eigen::Ref<const Eigen::VectorXd>& q,
            const Eigen::Ref<const Eigen::VectorXd>& v,
            const Eigen::Ref<const Eigen::VectorXd>& tau_external, double t,
            Eigen::VectorXd* const q_cmd,
            Eigen::VectorXd* const tau_cmd) const override;

 private:
  std::unique_ptr<const trajectories::PiecewisePolynomial<double>> q_traj_;
  const int num_positions_;
};

}  // namespace robot_plan_runner
}  // namespace manipulation
}  // namespace drake

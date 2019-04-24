#pragma once

#include <Eigen/Dense>

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/common/trajectories/piecewise_polynomial.h"

namespace drake {
namespace manipulation {
namespace robot_plan_runner {

class PlanBase {
 public:
  PlanBase() = default;
  virtual ~PlanBase() {};

  virtual void Step(const Eigen::Ref<const Eigen::VectorXd>& q,
                    const Eigen::Ref<const Eigen::VectorXd>& v,
                    const Eigen::Ref<const Eigen::VectorXd>& tau_external,
                    double t, Eigen::VectorXd* const q_commanded,
                    Eigen::VectorXd* const tau_commanded) const = 0;
};

class JointSpacePlan : public PlanBase {
 public:
  JointSpacePlan() : num_positions_(7){};

  void UpdatePlan(
      std::unique_ptr<trajectories::PiecewisePolynomial<double>> q_traj_new);

  void Step(const Eigen::Ref<const Eigen::VectorXd>& q,
            const Eigen::Ref<const Eigen::VectorXd>& v,
            const Eigen::Ref<const Eigen::VectorXd>& tau_external, double t,
            Eigen::VectorXd* const q_cmd,
            Eigen::VectorXd* const tau_cmd) const override;
 private:
  std::unique_ptr<trajectories::PiecewisePolynomial<double>> q_traj_;
  const int num_positions_;
};

}  // namespace robot_plan_runner
}  // namespace manipulation
}  // namespace drake

#pragma once

#include "drake/manipulation/robot_plan_runner/robot_plans/plan_base.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace manipulation {
namespace robot_plan_runner {
namespace robot_plans {

using Eigen::VectorXd;

class TaskSpacePlan : public PlanBase {
 public:
  TaskSpacePlan();

  void Step(const Eigen::Ref<const Eigen::VectorXd>& q,
            const Eigen::Ref<const Eigen::VectorXd>& v,
            const Eigen::Ref<const Eigen::VectorXd>& tau_external,
            double control_period, double t, const PlanData& plan_data,
            EigenPtr<Eigen::VectorXd> q_cmd,
            EigenPtr<Eigen::VectorXd> tau_cmd) const override;

 private:
  void UpdatePositionError(
      double t, const PlanData& plan_data,
      const Eigen::Ref<const Eigen::Vector3d>& p_WoQ_W) const;
  void UpdateOrientationError(double t, const PlanData& plan_data,
                              const Eigen::Quaterniond& Q_WT) const;

  std::unique_ptr<multibody::MultibodyPlant<double>> plant_;
  multibody::FrameIndex task_frame_idx_;
  multibody::ModelInstanceIndex robot_model_;
  mutable std::unique_ptr<systems::Context<double>> plant_context_;
  mutable Eigen::MatrixXd Jv_WTq_;
  mutable Eigen::Vector3d err_xyz_;
  mutable Eigen::Quaterniond Q_TTr_;
  mutable std::unique_ptr<Eigen::Vector3d> p_WoQ_W_t0_;

  // gains
  const Eigen::Array3d kp_translation;
  const Eigen::Array3d kp_rotation;
};

}  // namespace robot_plans
}  // namespace robot_plan_runner
}  // namespace manipulation
}  // namespace drake
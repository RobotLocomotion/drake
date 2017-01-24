#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "drake/common/drake_path.h"
#include "drake/common/trajectories/piecewise_polynomial_trajectory.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/system/joint_level_controller_system.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/system/kuka_plan_eval_system.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/system/qp_controller_system.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/system/robot_status_wrapper.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {

namespace examples {
namespace qp_inverse_dynamics {

const char kUrdfPath[] =
    "/examples/kuka_iiwa_arm/urdf/iiwa14_simplified_collision.urdf";

class KukaInverseDynamicsDemo : public systems::Diagram<double> {
 public:
  KukaInverseDynamicsDemo() {
    this->set_name("KukaInverseDynamicsDemo");

    // Instantiates an Multibody Dynamics (MBD) model of the world.
    auto tree = std::make_unique<RigidBodyTree<double>>();
    drake::parsers::urdf::AddModelInstanceFromUrdfFile(
        drake::GetDrakePath() + kUrdfPath, drake::multibody::joints::kFixed, nullptr, tree.get());

    drake::multibody::AddFlatTerrainToWorld(tree.get());

    systems::DiagramBuilder<double> builder;

    // Instantiates a RigidBodyPlant from an MBD model of the world.
    plant_ = builder.AddSystem<systems::RigidBodyPlant<double>>(move(tree));
    const RigidBodyTree<double>& robot = plant_->get_rigid_body_tree();

    rs_wrapper_ = builder.AddSystem(std::make_unique<RobotStatusWrapper>(robot));
    joint_level_controller_ = builder.AddSystem(std::make_unique<JointLevelControllerSystem>(robot));

    std::string alias_groups_config = drake::GetDrakePath() +
            "/examples/QPInverseDynamicsForHumanoids/"
            "config/kuka_alias_groups.yaml";
    std::string controller_config = drake::GetDrakePath() +
            "/examples/QPInverseDynamicsForHumanoids/"
            "config/kuka_controller.yaml";
    plan_eval_ = builder.AddSystem(std::make_unique<KukaPlanEvalSystem>(robot, alias_groups_config, controller_config));
    id_controller_ = builder.AddSystem(std::make_unique<QPControllerSystem>(robot));

    viz_publisher_ = builder.template AddSystem<systems::DrakeVisualizer>(robot, &lcm_);

    // plant -> rs
    builder.Connect(plant_->state_output_port(), rs_wrapper_->get_input_port_state());

    // rs -> qp_input
    builder.Connect(rs_wrapper_->get_output_port_humanoid_status(),
        plan_eval_->get_input_port_humanoid_status());
    // rs + qp_input -> qp_output
    builder.Connect(rs_wrapper_->get_output_port_humanoid_status(),
        id_controller_->get_input_port_humanoid_status());
    builder.Connect(plan_eval_->get_output_port_qp_input(),
        id_controller_->get_input_port_qp_input());
    // qp_output -> plant
    builder.Connect(id_controller_->get_output_port_qp_output(),
        joint_level_controller_->get_input_port_qp_output());
    builder.Connect(joint_level_controller_->get_output_port_torque(),
        plant_->get_input_port(0));

    builder.Connect(plant_->state_output_port(), viz_publisher_->get_input_port(0));

    builder.ExportOutput(plant_->state_output_port());
    builder.ExportOutput(joint_level_controller_->get_output_port_torque());

    builder.BuildInto(this);
  }

  const systems::RigidBodyPlant<double>& get_kuka_plant() const { return *plant_; }

  void SetDesiredTrajectory(std::unique_ptr<PiecewisePolynomialTrajectory> poly_trajectory, systems::Context<double>* context) {
    poly_trajectory_ = std::move(poly_trajectory);
    systems::Context<double>* plan_eval_context = GetMutableSubsystemContext(context, plan_eval_);
    systems::State<double>* plan_eval_state = plan_eval_context->get_mutable_state();
    plan_eval_->SetDesiredTrajectory(*poly_trajectory_, plan_eval_state);
  }

  systems::Context<double>* get_kuka_context(systems::Context<double>* context) const {
    return GetMutableSubsystemContext(context, plant_);
  }

 private:
  systems::RigidBodyPlant<double>* plant_{nullptr};
  RobotStatusWrapper* rs_wrapper_{nullptr};
  JointLevelControllerSystem* joint_level_controller_{nullptr};
  KukaPlanEvalSystem* plan_eval_{nullptr};
  QPControllerSystem* id_controller_{nullptr};

  std::unique_ptr<PiecewisePolynomialTrajectory> poly_trajectory_;
  systems::DrakeVisualizer* viz_publisher_{nullptr};
  drake::lcm::DrakeLcm lcm_;
};

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

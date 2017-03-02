#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "drake/common/drake_path.h"
#include "drake/common/trajectories/piecewise_polynomial_trajectory.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/systems/controllers/inverse_dynamics_controller.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/demultiplexer.h"
#include "drake/systems/primitives/multiplexer.h"
#include "drake/systems/primitives/trajectory_source.h"

namespace drake {

namespace examples {
namespace kuka_iiwa_arm {

const char kUrdfPath[] =
    "/examples/kuka_iiwa_arm/models/iiwa14/iiwa14_simplified_collision.urdf";

/// A systems::Diagram of a Kuka iiwa arm with position controller using
/// gravity compensation and a PID controller. A plan may be passed in as
/// a unique_ptr to a PiecewisePolynomialTrajectory.
template <typename T>
class KukaDemo : public systems::Diagram<T> {
 public:
  explicit KukaDemo(
      std::unique_ptr<PiecewisePolynomialTrajectory> poly_trajectory)
      : poly_trajectory_(std::move(poly_trajectory)) {
    this->set_name("KukaDemo");

    // Instantiates an Multibody Dynamics (MBD) model of the world.
    auto tree = std::make_unique<RigidBodyTree<T>>();
    drake::parsers::urdf::AddModelInstanceFromUrdfFile(
        drake::GetDrakePath() + kUrdfPath, drake::multibody::joints::kFixed,
        nullptr /* weld to frame */, tree.get());

    drake::multibody::AddFlatTerrainToWorld(tree.get());
    VerifyIiwaTree(*tree);

    systems::DiagramBuilder<T> builder;

    // Instantiates a RigidBodyPlant from an MBD model of the world.
    plant_ = builder.template AddSystem<systems::RigidBodyPlant<double>>(
        std::move(tree));

    DRAKE_ASSERT(plant_->get_input_port(0).size() ==
                 plant_->get_num_positions());

    // Create and add PID controller.
    Eigen::VectorXd kp;
    Eigen::VectorXd ki;
    Eigen::VectorXd kd;
    SetPositionControlledIiwaGains(&kp, &ki, &kd);

    controller_ =
        builder.template AddSystem<systems::InverseDynamicsController<T>>(
            GetDrakePath() + kUrdfPath, nullptr, kp, ki, kd,
            false /* no feedforward acceleration */);

    // TODO(siyuan): should be able to easily compute derivatives once
    // #5215 is in.
    auto zero_source =
        builder.template AddSystem<systems::ConstantVectorSource<T>>(
            Eigen::VectorXd::Zero(plant_->get_num_velocities()));
    auto input_mux =
        builder.template AddSystem<systems::Multiplexer<T>>(std::vector<int>{
            plant_->get_num_positions(), plant_->get_num_velocities()});
    builder.Connect(zero_source->get_output_port(),
                    input_mux->get_input_port(1));

    // Connects desired state to the controller.
    builder.Connect(input_mux->get_output_port(0),
                    controller_->get_input_port_desired_state());

    // Connects RBP state to the controller.
    builder.Connect(plant_->state_output_port(),
                    controller_->get_input_port_estimated_state());

    // Connects controller's output to RBP.
    builder.Connect(controller_->get_output_port_control(),
                    plant_->actuator_command_input_port());

    // Creates a plan and wraps it into a source system.
    desired_plan_ = builder.template AddSystem<systems::TrajectorySource<T>>(
        *poly_trajectory_);

    // Creates and adds LCM publisher for visualization.
    viz_publisher_ = builder.template AddSystem<systems::DrakeVisualizer>(
        plant_->get_rigid_body_tree(), &lcm_);

    builder.Connect(desired_plan_->get_output_port(),
                    input_mux->get_input_port(0));

    // Connects to publisher for visualization.
    builder.Connect(plant_->state_output_port(),
                    viz_publisher_->get_input_port(0));

    builder.ExportOutput(plant_->state_output_port());

    drake::log()->debug("Demo Kuka Plant Diagram built...");

    builder.BuildInto(this);
  }

  const systems::RigidBodyPlant<T>& get_kuka_plant() const { return *plant_; }

  systems::Context<T>* get_kuka_context(systems::Context<T>* context) const {
    return this->GetMutableSubsystemContext(context, plant_);
  }

 private:
  systems::RigidBodyPlant<T>* plant_{nullptr};
  systems::InverseDynamicsController<T>* controller_{nullptr};
  systems::Demultiplexer<T>* rbp_state_demux_{nullptr};
  systems::TrajectorySource<T>* desired_plan_{nullptr};
  std::unique_ptr<PiecewisePolynomialTrajectory> poly_trajectory_;
  systems::DrakeVisualizer* viz_publisher_{nullptr};
  drake::lcm::DrakeLcm lcm_;
};

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

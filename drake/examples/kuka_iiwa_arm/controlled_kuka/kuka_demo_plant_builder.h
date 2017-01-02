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
#include "drake/systems/controllers/gravity_compensator.h"
#include "drake/systems/controllers/pid_controlled_system.h"
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
    "/examples/kuka_iiwa_arm/urdf/iiwa14_simplified_collision.urdf";

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

    const int kNumActuators = tree->get_num_actuators();

    systems::DiagramBuilder<T> builder;

    // Instantiates a RigidBodyPlant from an MBD model of the world.
    auto plant =
        std::make_unique<systems::RigidBodyPlant<T>>(move(tree));
    plant_ = plant.get();

    DRAKE_ASSERT(plant_->get_input_port(0).size() ==
                 plant_->get_num_positions());

    // Create and add PID controller.
    // Constants are chosen by trial and error to qualitatively match an
    // experimental run with the same initial conditions and planner.
    // Quantitative comparisons would require torque control and a more careful
    // estimation of the model constants such as friction in the joints.

    // proportional constant.
    Eigen::VectorXd kp = Eigen::VectorXd::Zero(kNumActuators);
    kp << 100, 100, 100, 20, 10, 20, 1;

    // integral constant.
    Eigen::VectorXd ki = Eigen::VectorXd::Zero(kNumActuators);

    // derivative constant.
    Eigen::VectorXd kd = Eigen::VectorXd::Zero(kNumActuators);
    for (int i = 0; i < kp.rows(); ++i) {
      kd[i] = std::sqrt(kp[i]);
    }

    controller_ = builder.template AddSystem<systems::PidControlledSystem<T>>(
        std::move(plant), kp, ki, kd);

    // The iiwa's control protocol doesn't have any way to express the
    // desired velocity for the arm, so this simulation doesn't take
    // target velocities as an input.  The PidControlledSystem does
    // want target velocities to calculate the D term.  Since we don't
    // have any logic to calculate the desired target velocity (yet!)
    // set the D term (to stabilize the arm near the commanded
    // position) and feed a desired velocity vector of zero.
    auto zero_source =
        builder.template AddSystem<systems::ConstantVectorSource<T>>(
            Eigen::VectorXd::Zero(plant_->get_num_velocities()));
    auto input_mux =
        builder.template AddSystem<systems::Multiplexer<T>>(std::vector<int>{
            plant_->get_num_positions(), plant_->get_num_velocities()});
    builder.Connect(zero_source->get_output_port(),
                    input_mux->get_input_port(1));
    builder.Connect(input_mux->get_output_port(0),
                    controller_->get_input_port(1));

    gravity_compensator_ =
        builder.template AddSystem<systems::GravityCompensator<T>>(
            plant_->get_rigid_body_tree());

    // Split the input state into two signals one with the positions and one
    // with the velocities.
    // For Kuka:
    // -  get_num_states() = 14
    // -  get_num_positions() = 7
    rbp_state_demux_ = builder.template AddSystem<systems::Demultiplexer<T>>(
        plant_->get_num_states(), plant_->get_num_positions());

    // Creates a plan and wraps it into a source system.
    desired_plan_ = builder.template AddSystem<systems::TrajectorySource<T>>(
        *poly_trajectory_);

    // Creates and adds LCM publisher for visualization.
    viz_publisher_ = builder.template AddSystem<systems::DrakeVisualizer>(
        plant_->get_rigid_body_tree(), &lcm_);

    builder.Connect(desired_plan_->get_output_port(0),
                    input_mux->get_input_port(0));

    // Splits the RBP output into positions (q) and velocities (v).
    builder.Connect(controller_->get_output_port(0),
                    rbp_state_demux_->get_input_port(0));

    // Connects the gravity compensator to the output generalized positions.
    builder.Connect(rbp_state_demux_->get_output_port(0),
                    gravity_compensator_->get_input_port(0));
    builder.Connect(gravity_compensator_->get_output_port(0),
                    controller_->get_input_port(0));

    // Connects to publisher for visualization.
    builder.Connect(controller_->get_output_port(0),
                    viz_publisher_->get_input_port(0));

    builder.ExportOutput(controller_->get_output_port(0));

    drake::log()->debug("Demo Kuka Plant Diagram built...");

    builder.BuildInto(this);
  }

  const systems::RigidBodyPlant<T>& get_kuka_plant() const { return *plant_; }

  systems::Context<T>* get_kuka_context(systems::Context<T>* context) const {
    systems::Context<T>* controller_context =
        this->GetMutableSubsystemContext(context, controller_);
    return controller_->GetMutableSubsystemContext(controller_context, plant_);
  }

 private:
  systems::RigidBodyPlant<T>* plant_{nullptr};
  systems::PidControlledSystem<T>* controller_{nullptr};
  systems::Demultiplexer<T>* rbp_state_demux_{nullptr};
  systems::GravityCompensator<T>* gravity_compensator_{nullptr};
  systems::TrajectorySource<T>* desired_plan_{nullptr};
  std::unique_ptr<PiecewisePolynomialTrajectory> poly_trajectory_;
  systems::DrakeVisualizer* viz_publisher_{nullptr};
  drake::lcm::DrakeLcm lcm_;
};

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

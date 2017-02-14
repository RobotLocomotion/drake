/// @file
///
/// This demo sets up a position controlled and gravity compensated KUKA iiwa
/// robot within a simulation to follow an arbitrarily designed plan. The
/// generated plan takes the arm from the zero configuration to reach to a
/// position in space and then repeat this reaching task with a different joint
/// configuration constraint.

#include <iostream>
#include <memory>
#include <string>

#include <gflags/gflags.h>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_path.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/system/kuka_inverse_dynamics_servo.h"
#include "drake/examples/kuka_iiwa_arm/controlled_kuka/make_demo_plan.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/single_output_vector_source.h"

DEFINE_double(simulation_sec, 0.5, "Number of seconds to simulate.");

namespace drake {

using systems::Context;
using systems::Simulator;

namespace examples {

using qp_inverse_dynamics::KukaInverseDynamicsServo;

namespace kuka_iiwa_arm {
namespace {

template <typename T>
class PiecewisePolynomialSource : public systems::SingleOutputVectorSource<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PiecewisePolynomialSource)

  /**
   * Constructs a PiecewisePolynomialSource that interpolates a given
   * PiecewisePolynomial and its derivatives up to the specified order.
   * @param trajectory Trajectory to be interpolated, and it must have only
   * one column.
   * @param output_derivative_order Highest derivative order, needs to be
   * bigger than or equal to 0.
   * @param set_derivatives_to_zero_when_time_is_past_limits All derivatives
   * will be zero for interpolating time before the start time or after the
   * end time of @p trajectory.
   */
  PiecewisePolynomialSource(
      const PiecewisePolynomial<T>& trajectory, int output_derivative_order,
      bool set_derivatives_to_zero_when_time_is_past_limits)
      : systems::SingleOutputVectorSource<T>(
            trajectory.rows() * (1 + output_derivative_order)),
        trajectory_(trajectory),
        clamp_derivatives_(set_derivatives_to_zero_when_time_is_past_limits) {
    DRAKE_DEMAND(trajectory.cols() == 1);
    DRAKE_DEMAND(output_derivative_order >= 0);

    for (int i = 0; i < output_derivative_order; i++) {
      if (i == 0)
        derivatives_.push_back(trajectory_.derivative());
      else
        derivatives_.push_back(derivatives_[i - 1].derivative());
    }
  }

 protected:
  /**
   * Outputs a vector of values evaluated at the context time of the trajectory
   * and up to its Nth derivatives, where the trajectory and N are passed to the
   * constructor. The size of the vector is
   * (1 + output_derivative_order) * rows of the trajectory passed to the
   * constructor.
   */
  void DoCalcVectorOutput(
      const Context<T>& context,
      Eigen::VectorBlock<VectorX<T>>* output) const override {
    int len = trajectory_.rows();
    output->head(len) = trajectory_.value(context.get_time());

    double time = context.get_time();
    bool set_zero = clamp_derivatives_ && (time > trajectory_.getEndTime() ||
        time < trajectory_.getStartTime());

    for (size_t i = 0; i < derivatives_.size(); ++i) {
      if (set_zero) {
        output->segment(len * (i + 1), len).setZero();
      } else {
        output->segment(len * (i + 1), len) =
          derivatives_[i].value(context.get_time());
      }
    }
  }

 private:
  const PiecewisePolynomial<T> trajectory_;
  const bool clamp_derivatives_;
  std::vector<PiecewisePolynomial<T>> derivatives_;
};

int DoMain() {
  DRAKE_DEMAND(FLAGS_simulation_sec > 0);
  std::string model_path =
      GetDrakePath() +
      "/examples/kuka_iiwa_arm/urdf/iiwa14_simplified_collision.urdf";
  std::string alias_group_path = GetDrakePath() +
                                 "/examples/kuka_iiwa_arm/controlled_kuka/"
                                 "inverse_dynamics_controller_config/"
                                 "iiwa.alias_groups";
  std::string controller_config_path =
      GetDrakePath() +
      "/examples/kuka_iiwa_arm/controlled_kuka/"
      "inverse_dynamics_controller_config/iiwa.id_controller_config";

  lcm::DrakeLcm lcm;

  // Makes a RBT.
  std::unique_ptr<RigidBodyTree<double>> tree =
      std::make_unique<RigidBodyTree<double>>();
  drake::parsers::urdf::AddModelInstanceFromUrdfFile(model_path,
      drake::multibody::joints::kFixed, nullptr /* weld to frame */,
      tree.get());

  drake::multibody::AddFlatTerrainToWorld(tree.get());

  int iiwa_instance_id = RigidBodyTreeConstants::kFirstNonWorldModelInstanceId;

  // Builds Diagram of the closed loop simulation.
  systems::DiagramBuilder<double> builder;
  systems::RigidBodyPlant<double>* plant =
      builder.AddSystem<systems::RigidBodyPlant<double>>(std::move(tree));
  KukaInverseDynamicsServo* controller =
      builder.AddSystem<KukaInverseDynamicsServo>(model_path, alias_group_path,
                                                  controller_config_path);
  PiecewisePolynomialSource<double>* trajectory =
      builder.AddSystem<PiecewisePolynomialSource<double>>(
          MakeKukaDemoTrajectory(model_path)->get_piecewise_polynomial(),
          2 /* up to second derivative */,
          true /* clip velocity and acceleration to zero for out of bound t */);

  systems::DrakeVisualizer* visualizer =
      builder.AddSystem<systems::DrakeVisualizer>(plant->get_rigid_body_tree(),
                                                  &lcm);

  // plant -> controller
  builder.Connect(plant->model_instance_state_output_port(iiwa_instance_id),
                  controller->get_input_port_measured_state());

  // traj -> controller
  builder.Connect(trajectory->get_output_port(),
                  controller->get_input_port_desired_state_and_acceleration());

  // controller -> plant
  builder.Connect(
      controller->get_output_port_torque(),
      plant->model_instance_actuator_command_input_port(iiwa_instance_id));

  // plant -> viz
  builder.Connect(plant->state_output_port(), visualizer->get_input_port(0));

  std::unique_ptr<systems::System<double>> demo = builder.Build();

  Simulator<double> simulator(*demo);
  Context<double>* context = simulator.get_mutable_context();

  // Initliazations.
  controller->Initialize(dynamic_cast<systems::Diagram<double>*>(demo.get())
                             ->GetMutableSubsystemContext(context, controller));

  simulator.Initialize();
  simulator.set_target_realtime_rate(1.0);

  simulator.StepTo(FLAGS_simulation_sec);

  return 0;
}

}  // namespace
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::kuka_iiwa_arm::DoMain();
}

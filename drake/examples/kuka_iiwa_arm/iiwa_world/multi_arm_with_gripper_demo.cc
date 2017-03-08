#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_world/world_sim_tree_builder.h"
#include "drake/examples/kuka_iiwa_arm/sim_diagram_builder.h"
#include "drake/examples/schunk_wsg/schunk_wsg_constants.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/inverse_dynamics_controller.h"
#include "drake/systems/primitives/trajectory_source.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace {

std::unique_ptr<RigidBodyTree<double>> build_tree(
    int num_pairs, std::vector<ModelInstanceInfo<double>>* iiwa,
    std::vector<ModelInstanceInfo<double>>* wsg) {
  auto tree_builder = std::make_unique<WorldSimTreeBuilder<double>>();

  // Adds models to the simulation builder. Instances of these models can be
  // subsequently added to the world.
  tree_builder->StoreModel(
      "iiwa",
      "/examples/kuka_iiwa_arm/models/iiwa14/iiwa14_simplified_collision.urdf");
  tree_builder->StoreModel("wsg",
                           "/examples/schunk_wsg/models/schunk_wsg_50.sdf");

  iiwa->clear();
  wsg->clear();

  for (int i = 0; i < num_pairs; ++i) {
    // Adds an iiwa arm
    int id = tree_builder->AddFixedModelInstance(
        "iiwa", Vector3<double>(i, 0, 0));
    iiwa->push_back(tree_builder->get_model_info_for_instance(id));

    // Adds a wsg gripper
    id = tree_builder->AddModelInstanceToFrame(
        "wsg", Vector3<double>::Zero(), Vector3<double>::Zero(),
        tree_builder->tree().findFrame("iiwa_frame_ee", id),
        drake::multibody::joints::kFixed);
    wsg->push_back(tree_builder->get_model_info_for_instance(id));
  }

  return tree_builder->Build();
}

// Creates a demo with 3 iiwa arms and 3 wsg grippers.
void main() {
  drake::lcm::DrakeLcm lcm;
  std::vector<ModelInstanceInfo<double>> iiwa_info, wsg_info;
  SimDiagramBuilder<double> builder;
  systems::DiagramBuilder<double>* diagram_builder =
      builder.get_mutable_builder();

  const int kNumArms = 3;
  auto plant = builder.AddPlant(build_tree(kNumArms, &iiwa_info, &wsg_info));
  builder.AddVisualizer(&lcm);

  systems::TrajectorySource<double>* iiwa_traj_src{nullptr};
  systems::TrajectorySource<double>* wsg_traj_src{nullptr};

  // Generates a desired plan for all iiwa arms.
  {
    std::vector<double> times = {0, 2, 4};
    std::vector<MatrixX<double>> knots(times.size(),
                                       MatrixX<double>::Zero(7, 1));
    knots[1] << M_PI, 0, 0, M_PI / 2., 0, 0, 0;
    PiecewisePolynomial<double> poly = PiecewisePolynomial<double>::Cubic(
        times, knots, MatrixX<double>::Zero(7, 1), MatrixX<double>::Zero(7, 1));

    // Adds a trajectory source for desired state and accelerations.
    iiwa_traj_src =
        diagram_builder->template AddSystem<systems::TrajectorySource<double>>(
            PiecewisePolynomialTrajectory(poly), 1);
  }

  // Generates a desired plan for all wsg grippers.
  {
    std::vector<double> times = {0, 2, 4};
    std::vector<MatrixX<double>> knots(times.size(),
                                       MatrixX<double>::Zero(1, 1));
    knots[1] << -0.055;  // Fully open the gripper.
    PiecewisePolynomial<double> poly =
        PiecewisePolynomial<double>::FirstOrderHold(times, knots);

    // Adds a trajectory source for desired state and accelerations.
    wsg_traj_src =
        diagram_builder->template AddSystem<systems::TrajectorySource<double>>(
            PiecewisePolynomialTrajectory(poly), 1);
  }

  // Adds controllers for all the iiwa arms.
  VectorX<double> iiwa_kp, iiwa_kd, iiwa_ki;
  SetPositionControlledIiwaGains(&iiwa_kp, &iiwa_ki, &iiwa_kd);
  int ctr = 0;
  for (const auto& info : iiwa_info) {
    auto controller =
        builder.AddController<systems::InverseDynamicsController<double>>(
            info.instance_id, info.model_path, info.world_offset, iiwa_kp,
            iiwa_ki, iiwa_kd, false /* no feedforward acceleration */);

    // Updates the controller's model's end effector's inertia to include
    // the added gripper.
    const std::string kEndEffectorLinkName = "iiwa_link_7";
    Matrix6<double> lumped_gripper_inertia_EE =
        ComputeLumpedGripperInertiaInEndEffectorFrame(
            plant->get_rigid_body_tree(), info.instance_id,
            kEndEffectorLinkName, wsg_info[ctr].instance_id);
    RigidBody<double>* controller_ee =
        controller->get_robot_for_control().FindBody(kEndEffectorLinkName);
    controller_ee->set_spatial_inertia(lumped_gripper_inertia_EE);

    diagram_builder->Connect(iiwa_traj_src->get_output_port(),
                             controller->get_input_port_desired_state());
    ctr++;
  }

  // Adds controllers for all the wsg grippers.
  const int kWsgActDim = schunk_wsg::kSchunkWsgNumActuators;
  const VectorX<double> wsg_kp = VectorX<double>::Constant(kWsgActDim, 300.0);
  const VectorX<double> wsg_ki = VectorX<double>::Constant(kWsgActDim, 0.0);
  const VectorX<double> wsg_kd = VectorX<double>::Constant(kWsgActDim, 5.0);
  for (const auto& info : wsg_info) {
    std::unique_ptr<systems::MatrixGain<double>> feedback_selector =
        std::make_unique<systems::MatrixGain<double>>(
            schunk_wsg::GetSchunkWsgFeedbackSelector<double>());
    auto controller =
        builder.template AddController<systems::PidController<double>>(
            info.instance_id, std::move(feedback_selector), wsg_kp, wsg_ki,
            wsg_kd);
    diagram_builder->Connect(wsg_traj_src->get_output_port(),
                             controller->get_input_port_desired_state());
  }

  // Simulates.
  std::unique_ptr<systems::Diagram<double>> diagram = builder.Build();
  systems::Simulator<double> simulator(*diagram);
  simulator.Initialize();
  simulator.set_target_realtime_rate(1.0);
  simulator.StepTo(5);
}

}  // namespace
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

int main() {
  drake::examples::kuka_iiwa_arm::main();

  return 0;
}

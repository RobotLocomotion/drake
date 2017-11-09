#include <gflags/gflags.h>

#include "drake/common/text_logging_gflags.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/manipulation/schunk_wsg/schunk_wsg_plain_controller.h"
#include "drake/manipulation/util/sim_diagram_builder.h"
#include "drake/manipulation/util/world_sim_tree_builder.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/inverse_dynamics_controller.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/trajectory_source.h"

DEFINE_double(simulation_sec, 5., "Number of seconds to simulate.");

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace {
using manipulation::util::ModelInstanceInfo;
using manipulation::util::WorldSimTreeBuilder;
using manipulation::util::SimDiagramBuilder;
using trajectories::PiecewisePolynomial;

std::unique_ptr<RigidBodyTree<double>> build_tree(
    int num_pairs, std::vector<ModelInstanceInfo<double>>* iiwa,
    std::vector<ModelInstanceInfo<double>>* wsg) {
  auto tree_builder = std::make_unique<WorldSimTreeBuilder<double>>();

  // Adds models to the simulation builder. Instances of these models can be
  // subsequently added to the world.
  tree_builder->StoreDrakeModel(
      "iiwa",
      "drake/manipulation/models/iiwa_description/urdf/"
      "iiwa14_polytope_collision.urdf");
  tree_builder->StoreDrakeModel(
      "wsg",
      "drake/manipulation/models/wsg_50_description/sdf/schunk_wsg_50.sdf");

  iiwa->clear();
  wsg->clear();

  for (int i = 0; i < num_pairs; ++i) {
    // Adds an iiwa arm
    int id =
        tree_builder->AddFixedModelInstance("iiwa", Vector3<double>(i, 0, 0));
    iiwa->push_back(tree_builder->get_model_info_for_instance(id));

    // Adds a wsg gripper
    id = tree_builder->AddModelInstanceToFrame(
        "wsg", tree_builder->tree().findFrame("iiwa_frame_ee", id),
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
            poly, 1);
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
            poly, 1);
  }

  // Adds controllers for all the iiwa arms.
  VectorX<double> iiwa_kp, iiwa_kd, iiwa_ki;
  SetPositionControlledIiwaGains(&iiwa_kp, &iiwa_ki, &iiwa_kd);
  int ctr = 0;
  for (const auto& info : iiwa_info) {
    auto single_arm = std::make_unique<RigidBodyTree<double>>();
    parsers::urdf::AddModelInstanceFromUrdfFile(
        info.absolute_model_path, multibody::joints::kFixed, info.world_offset,
        single_arm.get());

    auto controller = builder.AddController<
        systems::controllers::InverseDynamicsController<double>>(
        info.instance_id, std::move(single_arm), iiwa_kp, iiwa_ki, iiwa_kd,
        false /* no feedforward acceleration */);
    controller->set_name("controller" + std::to_string(info.instance_id));

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
  for (const auto& info : wsg_info) {
    auto controller = builder.template AddController<
        manipulation::schunk_wsg::SchunkWsgPlainController>(info.instance_id);
    diagram_builder->Connect(wsg_traj_src->get_output_port(),
                             controller->get_input_port_desired_state());
    Vector1<double> max_force{40};  // Max force, in Newtons.
    const auto max_force_source =
        diagram_builder->AddSystem<systems::ConstantVectorSource<double>>(
            max_force);
    diagram_builder->Connect(max_force_source->get_output_port(),
                             controller->get_input_port_max_force());
  }

  // Simulates.
  std::unique_ptr<systems::Diagram<double>> diagram = builder.Build();
  systems::Simulator<double> simulator(*diagram);
  simulator.Initialize();
  simulator.set_target_realtime_rate(1.0);
  simulator.StepTo(FLAGS_simulation_sec);
}

}  // namespace
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::logging::HandleSpdlogGflags();
  drake::examples::kuka_iiwa_arm::main();
  return 0;
}

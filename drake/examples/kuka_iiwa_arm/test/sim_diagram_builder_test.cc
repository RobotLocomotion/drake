#include "drake/examples/kuka_iiwa_arm/sim_diagram_builder.h"

#include <gtest/gtest.h>

#include "drake/common/drake_path.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_world/world_sim_tree_builder.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/inverse_dynamics_controller.h"
#include "drake/systems/controllers/pid_controller.h"
#include "drake/systems/primitives/constant_vector_source.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace {

// Builds a RigidBodyTree of @p num_iiwa iiwa arm and @p num_wsg grippers.
std::unique_ptr<RigidBodyTree<double>> build_tree(
    int num_iiwa, int num_wsg, std::vector<ModelInstanceInfo<double>>* iiwa,
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

  DRAKE_DEMAND(num_iiwa >= num_wsg);
  DRAKE_DEMAND(num_wsg >= 0);

  for (int i = 0; i < num_iiwa; ++i) {
    // Adds an iiwa arm
    int id = tree_builder->AddFixedModelInstance(
        "iiwa", Vector3<double>(i, 0, 0));
    iiwa->push_back(tree_builder->get_model_info_for_instance(id));
  }

  for (int i = 0; i < num_wsg; ++i) {
    // Adds a wsg gripper
    int id = tree_builder->AddModelInstanceToFrame(
        "wsg", Vector3<double>::Zero(), Vector3<double>::Zero(),
        tree_builder->tree().findFrame("iiwa_frame_ee",
                                       iiwa->at(i).instance_id),
        drake::multibody::joints::kFixed);
    wsg->push_back(tree_builder->get_model_info_for_instance(id));
  }

  return tree_builder->Build();
}

// Builds a diagram with 2 iiwa arm with inverse dynamics controllers. Start
// the simulation from a non zero configuration with the controller trying to
// maintain the initial configuration. Assert that after simulation, the robots
// remain stationary.
GTEST_TEST(SimDiagramBuilderTest, TestSimulation) {
  SimDiagramBuilder<double> builder;
  const int kNumIiwa = 2;
  std::vector<ModelInstanceInfo<double>> iiwa_info, wsg_info;
  std::unique_ptr<RigidBodyTree<double>> tree =
      build_tree(kNumIiwa, 0, &iiwa_info, &wsg_info);

  const int kNumPos = 7;

  // Adds a plant.
  auto plant = builder.AddPlant(std::move(tree));

  // Adds desired.
  systems::DiagramBuilder<double>* base_builder = builder.get_mutable_builder();
  VectorX<double> state_d = VectorX<double>::Zero(2 * kNumPos);
  state_d.head<kNumPos>() << 0, 1, -0.5, -1, -0.5, 0.5, 1;
  auto state_d_source =
      base_builder->template AddSystem<systems::ConstantVectorSource<double>>(
          state_d);

  // Adds a controller.
  VectorX<double> iiwa_kp, iiwa_kd, iiwa_ki;
  SetPositionControlledIiwaGains(&iiwa_kp, &iiwa_ki, &iiwa_kd);

  for (const auto& info : iiwa_info) {
    auto controller =
        builder
            .template AddController<systems::InverseDynamicsController<double>>(
                info.instance_id, info.model_path, info.world_offset, iiwa_kp,
                iiwa_ki, iiwa_kd, false /* no feedforward acceleration */);

    base_builder->Connect(state_d_source->get_output_port(),
                          controller->get_input_port_desired_state());
  }

  drake::lcm::DrakeLcm lcm;
  builder.AddVisualizer(&lcm);

  base_builder->ExportOutput(plant->get_output_port(0));
  std::unique_ptr<systems::Diagram<double>> diagram = builder.Build();

  // Simulates.
  systems::Simulator<double> simulator(*diagram);
  systems::Context<double>* context = simulator.get_mutable_context();
  systems::Context<double>* plant_context =
      diagram->GetMutableSubsystemContext(context, plant);
  VectorX<double> state0(2 * kNumPos * iiwa_info.size());
  for (size_t i = 0; i < iiwa_info.size(); ++i) {
    state0.segment<kNumPos>(i * kNumPos) = state_d.head<kNumPos>();
    state0.segment<kNumPos>(i * kNumPos + kNumPos * iiwa_info.size()) =
        state_d.tail<kNumPos>();
  }
  plant->set_state_vector(plant_context, state0);

  simulator.Initialize();
  simulator.StepTo(0.02);

  auto state_output = diagram->AllocateOutput(simulator.get_context());
  diagram->CalcOutput(simulator.get_context(), state_output.get());
  const auto final_output_data = state_output->get_vector_data(0)->get_value();

  EXPECT_TRUE(drake::CompareMatrices(final_output_data, state0, 1e-15,
                                     drake::MatrixCompareType::absolute));
}

// Tests that Build without adding a RigidBodyPlant crashes.
GTEST_TEST(SimDiagramBuilderTest, TestNoPlantBuild) {
  SimDiagramBuilder<double> builder;
  EXPECT_DEATH(builder.Build(), ".*");
}

// Tests that multiple calls to AddPlant() crashes.
GTEST_TEST(SimDiagramBuilderTest, TestMultiAddPlant) {
  SimDiagramBuilder<double> builder;
  std::vector<ModelInstanceInfo<double>> iiwa_info, wsg_info;
  {
    std::unique_ptr<RigidBodyTree<double>> tree =
        build_tree(1, 0, &iiwa_info, &wsg_info);
    builder.AddPlant(std::move(tree));
  }
  {
    std::unique_ptr<RigidBodyTree<double>> tree =
        build_tree(1, 0, &iiwa_info, &wsg_info);
    EXPECT_DEATH(builder.AddPlant(std::move(tree)), ".*");
  }
}

// Tests that multiple calls to AddController() for the same model instance
// crashes.
GTEST_TEST(SimDiagramBuilderTest, TestMultiAddController) {
  SimDiagramBuilder<double> builder;
  builder.template AddController<systems::PidController<double>>(
      0, VectorX<double>::Zero(1), VectorX<double>::Zero(1),
      VectorX<double>::Zero(1));

  EXPECT_DEATH(builder.template AddController<systems::PidController<double>>(
                   0, VectorX<double>::Zero(1), VectorX<double>::Zero(1),
                   VectorX<double>::Zero(1)),
               ".*");
}

// Tests that multiple calls to AddVisualizer() crashes.
GTEST_TEST(SimDiagramBuilderTest, TestMultiAddVisualizer) {
  SimDiagramBuilder<double> builder;

  std::vector<ModelInstanceInfo<double>> iiwa_info, wsg_info;
  std::unique_ptr<RigidBodyTree<double>> tree =
      build_tree(1, 0, &iiwa_info, &wsg_info);
  builder.AddPlant(std::move(tree));

  drake::lcm::DrakeLcm lcm;
  builder.AddVisualizer(&lcm);
  EXPECT_DEATH(builder.AddVisualizer(&lcm), ".*");
}

// Tests that calling AddVisualizer() without AddPlant() crashes.
GTEST_TEST(SimDiagramBuilderTest, TestAddVisualizerWithoutPlant) {
  SimDiagramBuilder<double> builder;
  drake::lcm::DrakeLcm lcm;
  EXPECT_DEATH(builder.AddVisualizer(&lcm), ".*");
}

}  // namespace
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

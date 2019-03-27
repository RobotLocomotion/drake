// This test covers both the SimDiagramBuilder and the WorldSimTreeBuilder.
#include "drake/manipulation/util/sim_diagram_builder.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/manipulation/util/world_sim_tree_builder.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/pid_controller.h"
#include "drake/systems/controllers/rbt_inverse_dynamics_controller.h"
#include "drake/systems/primitives/constant_vector_source.h"

namespace drake {
namespace manipulation {
namespace util {
namespace {

// Builds a RigidBodyTree of @p num_iiwa iiwa arm and @p num_wsg grippers.
std::unique_ptr<RigidBodyTree<double>> build_tree(
    int num_iiwa, int num_wsg, std::vector<ModelInstanceInfo<double>>* iiwa,
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

  DRAKE_DEMAND(num_iiwa >= num_wsg);
  DRAKE_DEMAND(num_wsg >= 0);

  for (int i = 0; i < num_iiwa; ++i) {
    // Adds an iiwa arm
    int id =
        tree_builder->AddFixedModelInstance("iiwa", Vector3<double>(i, 0, 0));
    iiwa->push_back(tree_builder->get_model_info_for_instance(id));
  }

  for (int i = 0; i < num_wsg; ++i) {
    // Adds a wsg gripper
    int id = tree_builder->AddModelInstanceToFrame(
        "wsg",
        tree_builder->tree().findFrame("iiwa_frame_ee",
                                       iiwa->at(i).instance_id),
        drake::multibody::joints::kFixed);
    wsg->push_back(tree_builder->get_model_info_for_instance(id));
  }

  return tree_builder->Build();
}

GTEST_TEST(WorldSimTreeBuilderTest, TestFindNotStoredModel) {
  auto tree_builder = std::make_unique<WorldSimTreeBuilder<double>>();
  // Should throw if we are trying to find a model that doesn't exist.
  EXPECT_THROW(
      tree_builder->AddFixedModelInstance("233", Vector3<double>::Zero()),
      std::exception);
}

GTEST_TEST(WorldSimTreeBuilderTest, TestBuildWithInitialRigidBodyTree) {
  // First make a RBT with 1 iiwa.
  std::vector<ModelInstanceInfo<double>> iiwas, grippers;
  std::unique_ptr<RigidBodyTree<double>> tree =
      build_tree(1, 0, &iiwas, &grippers);

  EXPECT_EQ(tree->get_num_positions(), 7);

  // Get a WorldSimTreeBuilder starting from tree.
  auto tree_builder =
      std::make_unique<WorldSimTreeBuilder<double>>(true, std::move(tree));

  tree_builder->StoreDrakeModel(
      "second_iiwa",
      "drake/manipulation/models/iiwa_description/urdf/"
      "iiwa14_polytope_collision.urdf");

  // Add a second arm
  Isometry3<double> expected_pose = Isometry3<double>::Identity();
  expected_pose.translation()[0] = 233;
  int id = tree_builder->AddFixedModelInstance("second_iiwa",
      expected_pose.translation());
  ModelInstanceInfo<double> second_iiwa_info =
      tree_builder->get_model_info_for_instance(id);

  tree = tree_builder->Build();

  // Check kinematics.
  VectorX<double> q = tree->getZeroConfiguration();
  KinematicsCache<double> cache = tree->doKinematics(q);

  RigidBody<double>* second_base =
      tree->FindBody("base", "iiwa14", second_iiwa_info.instance_id);
  EXPECT_TRUE(second_base != nullptr);
  Isometry3<double> second_base_pose = tree->CalcBodyPoseInWorldFrame(
      cache, *second_base);

  EXPECT_TRUE(CompareMatrices(second_base_pose.matrix(), expected_pose.matrix(),
                              1e-15, MatrixCompareType::absolute));
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
  state_d_source->set_name("state_d_source");

  // Adds a controller.
  VectorX<double> iiwa_kp, iiwa_kd, iiwa_ki;

  iiwa_kp = VectorX<double>::Constant(7, 100);
  iiwa_kd = VectorX<double>::Constant(7, 0.1);
  iiwa_ki = VectorX<double>::Zero(7);

  for (const auto& info : iiwa_info) {
    auto single_arm = std::make_unique<RigidBodyTree<double>>();
    parsers::urdf::AddModelInstanceFromUrdfFile(
        info.absolute_model_path, multibody::joints::kFixed, info.world_offset,
        single_arm.get());

    auto controller = builder.template AddController<
        systems::controllers::rbt::InverseDynamicsController<double>>(
        info.instance_id, std::move(single_arm), iiwa_kp, iiwa_ki, iiwa_kd,
        false /* no feedforward acceleration */);
    controller->set_name("controller_" + std::to_string(info.instance_id));

    base_builder->Connect(state_d_source->get_output_port(),
                          controller->get_input_port_desired_state());
  }

  drake::lcm::DrakeLcm lcm;
  builder.AddVisualizer(&lcm);

  base_builder->ExportOutput(plant->get_output_port(0));
  std::unique_ptr<systems::Diagram<double>> diagram = builder.Build();

  // Simulates.
  systems::Simulator<double> simulator(*diagram);
  systems::Context<double>& context = simulator.get_mutable_context();
  systems::Context<double>& plant_context =
      diagram->GetMutableSubsystemContext(*plant, &context);
  VectorX<double> state0(2 * kNumPos * iiwa_info.size());
  for (size_t i = 0; i < iiwa_info.size(); ++i) {
    state0.segment<kNumPos>(i * kNumPos) = state_d.head<kNumPos>();
    state0.segment<kNumPos>(i * kNumPos + kNumPos * iiwa_info.size()) =
        state_d.tail<kNumPos>();
  }
  plant->set_state_vector(&plant_context, state0);

  simulator.Initialize();
  simulator.AdvanceTo(0.02);

  auto state_output = diagram->AllocateOutput();
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
  builder.template AddController<systems::controllers::PidController<double>>(
      0, VectorX<double>::Zero(1), VectorX<double>::Zero(1),
      VectorX<double>::Zero(1));

  EXPECT_DEATH(
      builder
          .template AddController<systems::controllers::PidController<double>>(
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

// Tests that if we have two IIWA arms in the diagram, and we add a schunk
// gripper to one of the arm, then the other arm should not have the gripper.
GTEST_TEST(SimDiagramBuilderTest, TestAddingOneSchunkToTwoArms) {
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

  std::vector<ModelInstanceInfo<double>> iiwa;
  std::vector<ModelInstanceInfo<double>> wsg;

  const int num_iiwa = 2;
  const int num_wsg = 1;
  DRAKE_ASSERT(num_iiwa >= num_wsg);

  for (int i = 0; i < num_iiwa; ++i) {
    // Adds an iiwa arm
    int id =
        tree_builder->AddFixedModelInstance("iiwa", Vector3<double>(i, 0, 0));
    iiwa.push_back(tree_builder->get_model_info_for_instance(id));
  }

  // The pose of the schunk frame `S` in the IIWA end effector body frame `E`.
  const Eigen::Isometry3d X_ES =
      Eigen::Translation3d(Eigen::Vector3d(0.09, 0, 0)) *
      Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(22.0 / 180 * M_PI, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());
  for (int i = 0; i < num_wsg; ++i) {
    // Adds a wsg gripper
    // The transformation from schunk to iiwa ee link has roll-pitch-yaw angle
    // [180, 22, 90], with translation[0.09, 0, 0].
    int id = tree_builder->AddModelInstanceToFrame(
        "wsg", "iiwa_link_ee", iiwa.at(i).instance_id, "iiwa_link_ee_S", X_ES,
        drake::multibody::joints::kFixed);
    wsg.push_back(tree_builder->get_model_info_for_instance(id));
  }

  // iiwa_link_ee_S is a frame rigidly attached to the link iiwa_link_ee, and
  // provides the location and orientation at which a schunk gripper should be
  // mounted.
  for (int i = 0; i < num_wsg; ++i) {
    auto schunk_frame = tree_builder->tree().findFrame("iiwa_link_ee_S",
                                                       iiwa.at(i).instance_id);
    EXPECT_TRUE(CompareMatrices(schunk_frame->get_transform_to_body().matrix(),
                                X_ES.matrix()));
  }
  for (int i = num_wsg; i < num_iiwa; ++i) {
    EXPECT_THROW(tree_builder->tree().findFrame("frame_schunk_to_iiwa_link_ee",
                                                iiwa.at(i).instance_id),
                 std::logic_error);
  }
}

}  // namespace
}  // namespace util
}  // namespace manipulation
}  // namespace drake

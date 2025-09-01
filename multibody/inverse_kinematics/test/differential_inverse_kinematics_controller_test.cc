#include "drake/multibody/inverse_kinematics/differential_inverse_kinematics_controller.h"

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/pointer_cast.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/planning/robot_diagram_builder.h"
#include "drake/planning/scene_graph_collision_checker.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/bus_value.h"

namespace drake {
namespace multibody {
namespace {

using Eigen::VectorXd;
using math::RigidTransformd;
using planning::CollisionCheckerParams;
using planning::DofMask;
using planning::RobotDiagramBuilder;
using planning::SceneGraphCollisionChecker;
using systems::BusValue;
using systems::Context;
using systems::InputPortIndex;
using systems::OutputPortIndex;
using systems::Simulator;

class DifferentialInverseKinematicsControllerTest : public ::testing::Test {
 protected:
  ModelInstanceIndex ArrangeArmModel(Parser* parser) {
    const char arm_url[] =
        "package://drake_models/iiwa_description/sdf/iiwa14_no_collision.sdf";
    const auto arm_instance = parser->AddModelsFromUrl(arm_url).at(0);
    auto& plant = parser->plant();
    plant.WeldFrames(plant.world_frame(),
                     plant.GetFrameByName("iiwa_link_0", arm_instance));
    return arm_instance;
  }

  void SetUp() {
    RobotDiagramBuilder<double> builder;
    auto arm_instance = ArrangeArmModel(&builder.parser());
    builder.plant().Finalize();
    auto model = builder.Build();
    CollisionCheckerParams checker_params{
        .model = std::move(model),
        .robot_model_instances = {arm_instance},
        .edge_step_size = 0.05};
    auto collision_checker =
        std::make_shared<SceneGraphCollisionChecker>(checker_params);

    // Pretend we have a mix of active and passive dofs, to exercise the
    // passive-dof handling.
    // TODO(rpoyner-tri): Figure out a test for passive dof handling during
    // selector configuration.
    DofMask mixed{true, true, true, false, true, true, true};

    using DiffIkSys = DifferentialInverseKinematicsSystem;
    auto recipe = std::make_shared<DiffIkSys::Recipe>();
    recipe->AddIngredient(std::make_shared<DiffIkSys::LeastSquaresCost>(
        DiffIkSys::LeastSquaresCost::Config{}));
    // Because we're not evaluating the controller, the values of these
    // parameters aren't particularly important.
    auto diff_ik = std::make_unique<DiffIkSys>(
        std::move(recipe), "world", std::move(collision_checker), mixed, 0.1,
        1.0, SpatialVelocity<double>::Zero());

    // Pretend we have planar rotation joints that need wrapping.
    // TODO(rpoyner-tri): Figure out a test to confirm that wrapping works.
    std::vector<int> rotation_indices({3});
    dut_ = std::make_unique<DifferentialInverseKinematicsController>(
        std::move(diff_ik), rotation_indices);
  }

  void FixTrivialInputs(Context<double>* context) {
    // Access input ports.
    auto& estimated_state_port = dut_->GetInputPort("estimated_state");
    auto& nominal_posture_port = dut_->GetInputPort("nominal_posture");
    auto& desired_poses_port = dut_->GetInputPort("desired_poses");

    // Fix inputs.
    VectorXd state = VectorXd::Zero(14);
    VectorXd positions = VectorXd::Zero(7);
    RigidTransformd desired_pose;
    BusValue desired_poses;
    desired_poses.Set("iiwa14::iiwa_link_7", Value{desired_pose});
    estimated_state_port.FixValue(context, state);
    nominal_posture_port.FixValue(context, positions);
    desired_poses_port.FixValue(context, desired_poses);
  }

  std::unique_ptr<DifferentialInverseKinematicsController> dut_;
};

/* Enable this test locally to emit a graphviz diagram as log output.

To extract the graphs from the log stream:

   # Extract sections of log into files `xx{N}`
   $ csplit --suppress-matched logfile '/GRAPHVIZ/' '{*}'
   # Rename the graphviz file(s) to {graph name}.dot.
   $ grep digraph xx* \
      | sed 's#\([^:]*\):digraph \([^ ]*\) .*#mv \1 \2.dot#g' | sh
   # Discard the other section files.
   $ rm xx*

*/
TEST_F(DifferentialInverseKinematicsControllerTest, DISABLED_Graphviz) {
  fmt::print("\n===BEGIN GRAPHVIZ===\n\n{}\n===END GRAPHVIZ===\n\n",
             dut_->GetGraphvizString());
}

TEST_F(DifferentialInverseKinematicsControllerTest, Trivia) {
  // Test trivial public methods.
  EXPECT_EQ(&dut_->differential_inverse_kinematics(),
            &dut_->get_mutable_differential_inverse_kinematics());
}

TEST_F(DifferentialInverseKinematicsControllerTest, Feedthrough) {
  // Every input feeds through to velocity output. No input feeds through to
  // position output, since output position is integrated.
  auto get_input = [this](const std::string& name) {
    return dut_->GetInputPort(name).get_index();
  };
  auto get_output = [this](const std::string& name) {
    return dut_->GetOutputPort(name).get_index();
  };
  const std::multimap<int, int> expected_feedthroughs{
      {get_input("estimated_state"), get_output("commanded_velocity")},
      {get_input("desired_poses"), get_output("commanded_velocity")},
      {get_input("nominal_posture"), get_output("commanded_velocity")}};
  EXPECT_EQ(dut_->GetDirectFeedthroughs(), expected_feedthroughs);
}

TEST_F(DifferentialInverseKinematicsControllerTest, Initialization) {
  auto context = dut_->CreateDefaultContext();
  FixTrivialInputs(context.get());
  Simulator<double> simulator(*dut_, std::move(context));
  // Advancing without setting initial position will throw, owing to NaN
  // poisoning by the controller.
  DRAKE_EXPECT_THROWS_MESSAGE(simulator.AdvanceTo(0.001),
                              ".*allFinite.*failed.*");

  // Once initialized, advancing works.
  dut_->set_initial_position(&simulator.get_mutable_context(),
                             VectorXd::Zero(7));
  simulator.AdvanceTo(0.001);
}

TEST_F(DifferentialInverseKinematicsControllerTest, Random) {
  auto context = dut_->CreateDefaultContext();
  FixTrivialInputs(context.get());
  Simulator<double> simulator(*dut_, std::move(context));
  VectorXd positions = VectorXd::Zero(7);
  dut_->set_initial_position(&simulator.get_mutable_context(), positions);
  RandomGenerator generator;
  dut_->SetRandomContext(&simulator.get_mutable_context(), &generator);
  // Setting random state also performs NaN poisoning.
  DRAKE_EXPECT_THROWS_MESSAGE(simulator.AdvanceTo(0.001),
                              ".*allFinite.*failed.*");
  dut_->set_initial_position(&simulator.get_mutable_context(), positions);
  simulator.AdvanceTo(0.001);
}

// TODO(rpoyner-tri): add a test for position feedback.

}  // namespace
}  // namespace multibody
}  // namespace drake

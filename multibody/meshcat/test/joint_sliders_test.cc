#include "drake/multibody/meshcat/joint_sliders.h"

#include <memory>
#include <string>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/meshcat_visualizer.h"
#include "drake/geometry/test_utilities/meshcat_environment.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/test_utilities/initialization_test_system.h"

namespace drake {
namespace multibody {
namespace meshcat {
namespace {

using Eigen::Vector2d;
using Eigen::VectorXd;
using geometry::Meshcat;
using geometry::MeshcatVisualizer;
using geometry::SceneGraph;

class JointSlidersTest : public ::testing::Test {
 public:
  JointSlidersTest()
      : meshcat_{geometry::GetTestEnvironmentMeshcat()},
        builder_{},
        plant_and_scene_graph_{AddMultibodyPlantSceneGraph(&builder_, 0.0)},
        plant_{plant_and_scene_graph_.plant},
        scene_graph_{plant_and_scene_graph_.scene_graph} {}

  void Add(const std::string& url, const std::string& model_name = {}) {
    Parser parser(&plant_, model_name);
    parser.AddModelsFromUrl(url);
  }

  void AddAcrobot() {
    Add("package://drake/multibody/benchmarks/acrobot/acrobot.urdf");
    plant_.Finalize();
  }

  static constexpr char kAcrobotJoint1[] = "ShoulderJoint";
  static constexpr char kAcrobotJoint2[] = "ElbowJoint";

 protected:
  std::shared_ptr<Meshcat> meshcat_;
  systems::DiagramBuilder<double> builder_;
  AddMultibodyPlantSceneGraphResult<double> plant_and_scene_graph_;
  MultibodyPlant<double>& plant_;
  SceneGraph<double>& scene_graph_;
};

// Test the narrowest constructor, where all optionals are null.
TEST_F(JointSlidersTest, NarrowConstructor) {
  // Parse the acrobot model.
  AddAcrobot();
  auto& joint_1 = plant_.GetMutableJointByName(kAcrobotJoint1);
  auto& joint_2 = plant_.GetMutableJointByName(kAcrobotJoint2);

  // Set default positions.
  const double default_angle_1 = 0.1;
  const double default_angle_2 = 0.2;
  joint_1.set_default_positions(Vector1d(default_angle_1));
  joint_2.set_default_positions(Vector1d(default_angle_2));

  // Set joint limits.
  const double min_angle_1 = -1.0;
  const double max_angle_1 = 1.0;
  const double min_angle_2 = -200.0;
  const double max_angle_2 = 200.0;
  joint_1.set_position_limits(Vector1d(min_angle_1), Vector1d(max_angle_1));
  joint_2.set_position_limits(Vector1d(min_angle_2), Vector1d(max_angle_2));

  // Add the sliders.
  const JointSliders<double> dut(meshcat_, &plant_);
  auto context = dut.CreateDefaultContext();

  // Sliders start at their default context value.
  EXPECT_EQ(meshcat_->GetSliderValue(kAcrobotJoint1), default_angle_1);
  EXPECT_EQ(meshcat_->GetSliderValue(kAcrobotJoint2), default_angle_2);
  EXPECT_EQ(dut.get_output_port().Eval(*context),
            Vector2d(default_angle_1, default_angle_2));

  // Slider limits obey the joint positions limits.
  meshcat_->SetSliderValue(kAcrobotJoint1, -9999);
  EXPECT_EQ(meshcat_->GetSliderValue(kAcrobotJoint1), min_angle_1);
  meshcat_->SetSliderValue(kAcrobotJoint1, 9999);
  EXPECT_EQ(meshcat_->GetSliderValue(kAcrobotJoint1), max_angle_1);
  EXPECT_EQ(dut.get_output_port().Eval(*context),
            Vector2d(max_angle_1, default_angle_2));

  // When the joint position limits are huge, the slider uses a narrower range.
  meshcat_->SetSliderValue(kAcrobotJoint1, -9999);
  EXPECT_GT(meshcat_->GetSliderValue(kAcrobotJoint2), min_angle_2);
  meshcat_->SetSliderValue(kAcrobotJoint1, 9999);
  EXPECT_LT(meshcat_->GetSliderValue(kAcrobotJoint2), max_angle_2);
}

// Test the widest constructor, where all optionals are set to vectors.
TEST_F(JointSlidersTest, WideConstructorWithVectors) {
  // Parse the acrobot model.
  AddAcrobot();

  const double default_angle_1 = 0.1;
  const double default_angle_2 = 0.2;
  const double min_angle_1 = -1.0;
  const double max_angle_1 = 1.0;
  const double min_angle_2 = -200.0;
  const double max_angle_2 = 200.0;
  const double step_1 = 0.1;   // Round to one decimal.
  const double step_2 = 0.01;  // Round to two decimals.

  // Add some very specifically configured sliders.
  const Eigen::Vector2d initial_value(default_angle_1, default_angle_2);
  const Eigen::Vector2d lower_limit(min_angle_1, min_angle_2);
  const Eigen::Vector2d upper_limit(max_angle_1, max_angle_2);
  const Eigen::Vector2d step(step_1, step_2);
  const std::vector<std::string> decrement_keycodes{"ArrowLeft", "ArrowDown"};
  const std::vector<std::string> increment_keycodes{"ArrowRight", "ArrowUp"};
  const JointSliders dut(meshcat_, &plant_, initial_value, lower_limit,
                         upper_limit, step, decrement_keycodes,
                         increment_keycodes);
  auto context = dut.CreateDefaultContext();

  // Sliders start at their initial value.
  EXPECT_EQ(meshcat_->GetSliderValue(kAcrobotJoint1), default_angle_1);
  EXPECT_EQ(meshcat_->GetSliderValue(kAcrobotJoint2), default_angle_2);
  EXPECT_EQ(dut.get_output_port().Eval(*context),
            Vector2d(default_angle_1, default_angle_2));

  // Slider obey their configured limits.
  meshcat_->SetSliderValue(kAcrobotJoint1, -9999);
  EXPECT_EQ(meshcat_->GetSliderValue(kAcrobotJoint1), min_angle_1);
  meshcat_->SetSliderValue(kAcrobotJoint1, 9999);
  EXPECT_EQ(meshcat_->GetSliderValue(kAcrobotJoint1), max_angle_1);
  EXPECT_EQ(dut.get_output_port().Eval(*context),
            Vector2d(max_angle_1, default_angle_2));

  // When the limits are huge, the slider uses a narrower range.
  meshcat_->SetSliderValue(kAcrobotJoint1, -9999);
  EXPECT_GT(meshcat_->GetSliderValue(kAcrobotJoint2), min_angle_2);
  meshcat_->SetSliderValue(kAcrobotJoint1, 9999);
  EXPECT_LT(meshcat_->GetSliderValue(kAcrobotJoint2), max_angle_2);

  // Each slider has a different roundoff step.
  meshcat_->SetSliderValue(kAcrobotJoint1, 0.1111111111);
  meshcat_->SetSliderValue(kAcrobotJoint2, 0.2222222222);
  EXPECT_EQ(meshcat_->GetSliderValue(kAcrobotJoint1), 0.1);
  EXPECT_EQ(meshcat_->GetSliderValue(kAcrobotJoint2), 0.22);
}

// Test the widest constructor, with the variants set to single scalars.
TEST_F(JointSlidersTest, WideConstructorWithScalars) {
  // Parse the acrobot model.
  AddAcrobot();

  // Add some very specifically configured sliders.
  const double min_angle = -1.0;
  const double max_angle = 1.0;
  const double step = 0.1;
  const JointSliders dut(meshcat_, &plant_, {}, min_angle, max_angle, step);
  auto context = dut.CreateDefaultContext();

  // Slider obey their configured limits.
  meshcat_->SetSliderValue(kAcrobotJoint1, -9999);
  meshcat_->SetSliderValue(kAcrobotJoint2, -9999);
  EXPECT_EQ(meshcat_->GetSliderValue(kAcrobotJoint1), min_angle);
  EXPECT_EQ(meshcat_->GetSliderValue(kAcrobotJoint2), min_angle);
  meshcat_->SetSliderValue(kAcrobotJoint1, 9999);
  meshcat_->SetSliderValue(kAcrobotJoint2, 9999);
  EXPECT_EQ(meshcat_->GetSliderValue(kAcrobotJoint1), max_angle);
  EXPECT_EQ(meshcat_->GetSliderValue(kAcrobotJoint2), max_angle);
  EXPECT_EQ(dut.get_output_port().Eval(*context),
            Vector2d::Constant(max_angle));

  // Each slider uses the given step.
  meshcat_->SetSliderValue(kAcrobotJoint1, 0.1111111111);
  meshcat_->SetSliderValue(kAcrobotJoint2, 0.2222222222);
  EXPECT_EQ(meshcat_->GetSliderValue(kAcrobotJoint1), 0.1);
  EXPECT_EQ(meshcat_->GetSliderValue(kAcrobotJoint2), 0.2);
}

// Test that the constructor diagnoses num_positions mismatches.
TEST_F(JointSlidersTest, WrongNumPositions) {
  AddAcrobot();
  DRAKE_EXPECT_THROWS_MESSAGE(
      JointSliders(meshcat_, &plant_, Vector1d::Zero(), {}, {}, {}),
      "Expected initial_value of size 2, but got size 1 instead");
  DRAKE_EXPECT_THROWS_MESSAGE(
      JointSliders(meshcat_, &plant_, {}, Vector1d::Zero(), {}, {}),
      "Expected lower_limit of size 2, but got size 1 instead");
  DRAKE_EXPECT_THROWS_MESSAGE(
      JointSliders(meshcat_, &plant_, {}, {}, Vector1d::Zero(), {}),
      "Expected upper_limit of size 2, but got size 1 instead");
  DRAKE_EXPECT_THROWS_MESSAGE(
      JointSliders(meshcat_, &plant_, {}, {}, {}, Vector1d::Zero()),
      "Expected step of size 2, but got size 1 instead");
}

// Test robots with overlapping joint names, in which case the model name must
// be used to disambiguate them.
TEST_F(JointSlidersTest, DuplicatedJointNames) {
  Add("package://drake/multibody/benchmarks/acrobot/acrobot.urdf", "alpha");
  Add("package://drake/multibody/benchmarks/acrobot/acrobot.urdf", "bravo");
  plant_.Finalize();

  // Add the sliders.
  const JointSliders<double> dut(meshcat_, &plant_);

  // TODO(rpoyner-tri): We would probably prefer slashes for names nesting on
  // sliders labels.
  // Confirm that the names are unique.
  const std::string alpha = "/alpha::acrobot";
  const std::string bravo = "/bravo::acrobot";
  EXPECT_EQ(meshcat_->GetSliderValue(kAcrobotJoint1 + alpha), 0.0);
  EXPECT_EQ(meshcat_->GetSliderValue(kAcrobotJoint2 + alpha), 0.0);
  EXPECT_EQ(meshcat_->GetSliderValue(kAcrobotJoint1 + bravo), 0.0);
  EXPECT_EQ(meshcat_->GetSliderValue(kAcrobotJoint2 + bravo), 0.0);
}

// Test a multi-dof joint.
TEST_F(JointSlidersTest, MultiDofJoint) {
  Add("package://drake/multibody/meshcat/test/universal_joint.sdf");
  plant_.Finalize();
  const JointSliders<double> dut(meshcat_, &plant_);

  // Confirm the names hasve the per-dof suffix.
  EXPECT_EQ(meshcat_->GetSliderValue("charlie_qx"), 0.0);
  EXPECT_EQ(meshcat_->GetSliderValue("charlie_qy"), 0.0);
}

// Tests that a free body gets no sliders. In the future we might add support
// for this, which is fine (and thus we'd change this test), but at least for
// now we should confirm that nothing crashes in this case.
TEST_F(JointSlidersTest, FreeBody) {
  Add("package://drake/multibody/models/box.urdf");
  plant_.Finalize();
  const JointSliders<double> dut(meshcat_, &plant_);
  EXPECT_EQ(dut.get_output_port().size(), 7);
}

// Tests that the Delete function removes the sliders.
TEST_F(JointSlidersTest, DeleteFunction) {
  // Add the sliders.
  AddAcrobot();
  auto dut = std::make_unique<JointSliders<double>>(meshcat_, &plant_);
  EXPECT_EQ(meshcat_->GetSliderValue(kAcrobotJoint1), 0.0);

  // Remove them; confirm that they are gone.
  dut->Delete();
  EXPECT_THROW(meshcat_->GetSliderValue(kAcrobotJoint1), std::exception);

  // A second call is harmless.
  dut->Delete();

  // The destructor does not crash.
  EXPECT_NO_THROW(dut.reset());
}

// Tests that the destructor removes the sliders.
TEST_F(JointSlidersTest, Destructor) {
  // Add the sliders.
  AddAcrobot();
  auto dut = std::make_unique<JointSliders<double>>(meshcat_, &plant_);
  EXPECT_EQ(meshcat_->GetSliderValue(kAcrobotJoint1), 0.0);

  // Delete the entire object; confirm that the sliders are gone.
  EXPECT_NO_THROW(dut.reset());
  EXPECT_THROW(meshcat_->GetSliderValue(kAcrobotJoint1), std::exception);
}

// Tests the "Run" sugar function.
TEST_F(JointSlidersTest, Run) {
  // Add the acrobot visualizer and sliders.
  AddAcrobot();

  Vector2d initial_value{0.12, 0.34};
  MeshcatVisualizer<double>::AddToBuilder(&builder_, scene_graph_, meshcat_);
  auto* dut = builder_.AddSystem<JointSliders<double>>(meshcat_, &plant_,
                                                       initial_value);

  auto init_system = builder_.AddSystem<systems::InitializationTestSystem>();

  auto diagram = builder_.Build();

  // Run for a while.
  const double timeout = 1.0;
  Eigen::VectorXd q = dut->Run(*diagram, timeout);
  EXPECT_TRUE(CompareMatrices(q, initial_value));

  // Confirm that initialization events were triggered.
  EXPECT_TRUE(init_system->get_pub_init());
  EXPECT_TRUE(init_system->get_dis_update_init());
  EXPECT_TRUE(init_system->get_unres_update_init());

  // Note: the stop button is deleted on timeout, so we cannot easily check
  // that it was created correctly here.

  // Obtain the current pose of one shape.
  const std::string geometry_path = "visualizer/acrobot/Link1";
  const std::string original = meshcat_->GetPackedTransform(geometry_path);
  ASSERT_FALSE(original.empty());

  // Set a non-default slider position.
  meshcat_->SetSliderValue(kAcrobotJoint1, 0.25);

  // Run for a while (with a non-default stop_button_keycode).
  q = dut->Run(*diagram, timeout, "KeyP");
  EXPECT_TRUE(CompareMatrices(q, Vector2d{0.25, initial_value[1]}));

  // Check that the slider's transform had any effect, i.e., that the
  // MeshcatVisualizer::Publish was called.
  const std::string updated = meshcat_->GetPackedTransform(geometry_path);
  ASSERT_FALSE(updated.empty());
  EXPECT_NE(updated, original);
}

// Tests that SetPositions diagnoses num_positions mismatches.
TEST_F(JointSlidersTest, SetPositionsWrongNumPositions) {
  AddAcrobot();
  JointSliders<double> dut(meshcat_, &plant_);
  DRAKE_EXPECT_THROWS_MESSAGE(dut.SetPositions(Vector1d::Zero()),
                              "Expected q of size 2, but got size 1 instead");
}

/* Tests the "SetPositions" function with the Acrobot model (number of positions
 on the MultibodyPlant equal to the number of joints). */
TEST_F(JointSlidersTest, SetPositionsAcrobot) {
  // Acrobot has two positions, both of them joints.
  AddAcrobot();
  JointSliders<double> dut(meshcat_, &plant_);
  auto context = dut.CreateDefaultContext();
  ASSERT_EQ(dut.get_output_port().size(), 2);

  // The initial configuration should set both joints to 0.
  auto positions = dut.get_output_port().Eval(*context);
  EXPECT_EQ(positions[0], 0);
  EXPECT_EQ(positions[1], 0);
  EXPECT_EQ(meshcat_->GetSliderValue(kAcrobotJoint1), 0);
  EXPECT_EQ(meshcat_->GetSliderValue(kAcrobotJoint2), 0);

  /* Setting the positions should update both the initial value and sliders.
   Do not initialize q to something outside of (lower_limit, upper_limit), which
   defaults to (-10, 10). */
  const Vector2d q{-4, 7};
  dut.SetPositions(q);
  positions = dut.get_output_port().Eval(*context);
  EXPECT_EQ(positions[0], q[0]);
  EXPECT_EQ(positions[1], q[1]);
  EXPECT_EQ(meshcat_->GetSliderValue(kAcrobotJoint1), q[0]);
  EXPECT_EQ(meshcat_->GetSliderValue(kAcrobotJoint2), q[1]);

  // Deleting should remove the sliders, but the positions should remain.
  dut.Delete();
  positions = dut.get_output_port().Eval(*context);
  EXPECT_EQ(positions[0], q[0]);
  EXPECT_EQ(positions[1], q[1]);
  EXPECT_THROW(meshcat_->GetSliderValue(kAcrobotJoint1), std::exception);
  EXPECT_THROW(meshcat_->GetSliderValue(kAcrobotJoint2), std::exception);
}

/* Tests the "SetPositions" function with the Kuka IIWA Robot model (number of
 positions on the MultibodyPlant not equal to the number of joints). */
TEST_F(JointSlidersTest, SetPositionsKukaIiwaRobot) {
  // Kuka IIWA has 14 positions, 7 of them are joints.
  static constexpr char kKukaIiwaJoint1[] = "iiwa_joint_1";  // Index: 7
  static constexpr char kKukaIiwaJoint2[] = "iiwa_joint_2";  // Index: 8
  static constexpr char kKukaIiwaJoint3[] = "iiwa_joint_3";  // Index: 9
  static constexpr char kKukaIiwaJoint4[] = "iiwa_joint_4";  // Index: 10
  static constexpr char kKukaIiwaJoint5[] = "iiwa_joint_5";  // Index: 11
  static constexpr char kKukaIiwaJoint6[] = "iiwa_joint_6";  // Index: 12
  static constexpr char kKukaIiwaJoint7[] = "iiwa_joint_7";  // Index: 13
  Add("package://drake_models/iiwa_description/urdf/"
      "iiwa14_primitive_collision.urdf");
  plant_.Finalize();
  JointSliders<double> dut(meshcat_, &plant_);
  auto context = dut.CreateDefaultContext();
  EXPECT_EQ(dut.get_output_port().size(), 14);

  /* The initial configuration should should be [1, 0, ..., 0] -- the first
   index should be 1, everything else 0.  Therefore, every joint slider should
   have a value of 0 as well. */
  VectorXd initial = VectorXd::Zero(14);
  initial[0] = 1;
  auto positions = dut.get_output_port().Eval(*context);
  EXPECT_TRUE(CompareMatrices(positions, initial));
  VectorXd slider_values = VectorXd(7);
  slider_values << meshcat_->GetSliderValue(kKukaIiwaJoint1),
      meshcat_->GetSliderValue(kKukaIiwaJoint2),
      meshcat_->GetSliderValue(kKukaIiwaJoint3),
      meshcat_->GetSliderValue(kKukaIiwaJoint4),
      meshcat_->GetSliderValue(kKukaIiwaJoint5),
      meshcat_->GetSliderValue(kKukaIiwaJoint6),
      meshcat_->GetSliderValue(kKukaIiwaJoint7);
  EXPECT_TRUE(CompareMatrices(slider_values, VectorXd::Zero(7)));

  // Setting the positions should update both the initial value and sliders.
  VectorXd q(14);
  q << 0, 0, 0, 1,                               // floating base quaternion
      -3, -2, -1,                                // floating base position
      0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07;  // iiwa joints
  dut.SetPositions(q);
  positions = dut.get_output_port().Eval(*context);
  EXPECT_TRUE(CompareMatrices(positions, q));
  slider_values << meshcat_->GetSliderValue(kKukaIiwaJoint1),
      meshcat_->GetSliderValue(kKukaIiwaJoint2),
      meshcat_->GetSliderValue(kKukaIiwaJoint3),
      meshcat_->GetSliderValue(kKukaIiwaJoint4),
      meshcat_->GetSliderValue(kKukaIiwaJoint5),
      meshcat_->GetSliderValue(kKukaIiwaJoint6),
      meshcat_->GetSliderValue(kKukaIiwaJoint7);
  EXPECT_TRUE(CompareMatrices(slider_values, q.tail<7>()));

  // Deleting should remove the sliders, but the positions should remain.
  dut.Delete();
  positions = dut.get_output_port().Eval(*context);
  EXPECT_TRUE(CompareMatrices(positions, q));
  EXPECT_THROW(meshcat_->GetSliderValue(kKukaIiwaJoint1), std::exception);
  EXPECT_THROW(meshcat_->GetSliderValue(kKukaIiwaJoint2), std::exception);
  EXPECT_THROW(meshcat_->GetSliderValue(kKukaIiwaJoint3), std::exception);
  EXPECT_THROW(meshcat_->GetSliderValue(kKukaIiwaJoint4), std::exception);
  EXPECT_THROW(meshcat_->GetSliderValue(kKukaIiwaJoint5), std::exception);
  EXPECT_THROW(meshcat_->GetSliderValue(kKukaIiwaJoint6), std::exception);
  EXPECT_THROW(meshcat_->GetSliderValue(kKukaIiwaJoint7), std::exception);
}

TEST_F(JointSlidersTest, Graphviz) {
  AddAcrobot();
  const JointSliders<double> dut(meshcat_, &plant_);
  EXPECT_THAT(dut.GetGraphvizString(), testing::HasSubstr("meshcat_out ->"));
}

}  // namespace
}  // namespace meshcat
}  // namespace multibody
}  // namespace drake

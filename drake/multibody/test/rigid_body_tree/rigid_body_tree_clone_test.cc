#include "drake/multibody/rigid_body_tree.h"

#include <cmath>
#include <iostream>
#include <memory>

#include <gtest/gtest.h>

#include "drake/common/drake_path.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/sdf_parser.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/test/rigid_body_tree/rigid_body_tree_compare_to_clone.h"
#include "drake/systems/analysis/explicit_euler_integrator.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/signal_logger.h"

using std::unique_ptr;

namespace drake {

using multibody::test::rigid_body_tree::CompareToClone;
using parsers::ModelInstanceIdTable;
using parsers::sdf::AddModelInstancesFromSdfFileToWorld;
using parsers::urdf::AddModelInstanceFromUrdfFileWithRpyJointToWorld;
using parsers::urdf::AddModelInstanceFromUrdfFileToWorld;

namespace systems {
namespace plants {
namespace test {
namespace {

class RigidBodyTreeCloneTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    tree_ = std::make_unique<RigidBodyTree<double>>();
  }
  unique_ptr<RigidBodyTree<double>> tree_;
};

// Tests RigidBodyTree::Clone() using a simple two DOF robot.
TEST_F(RigidBodyTreeCloneTest, CloneTwoDofRobot) {
  std::string filename = drake::GetDrakePath() +
      "/multibody/test/rigid_body_tree/two_dof_robot.urdf";
  AddModelInstanceFromUrdfFileWithRpyJointToWorld(filename, tree_.get());
  EXPECT_TRUE(CompareToClone(*tree_));
}

// Tests RigidBodyTree::Clone() using Atlas.
TEST_F(RigidBodyTreeCloneTest, CloneAtlas) {
  std::string filename = drake::GetDrakePath() +
      "/examples/Atlas/urdf/atlas_convex_hull.urdf";
  AddModelInstanceFromUrdfFileToWorld(filename, multibody::joints::kQuaternion,
      tree_.get());
  EXPECT_TRUE(CompareToClone(*tree_));
}

// Tests RigidBodyTree::Clone() using a Prius with LIDAR sensors.
TEST_F(RigidBodyTreeCloneTest, ClonePrius) {
  std::string filename = drake::GetDrakePath() +
     "/automotive/models/prius/prius_with_lidar.sdf";
  AddModelInstancesFromSdfFileToWorld(filename, multibody::joints::kQuaternion,
      tree_.get());
  EXPECT_TRUE(CompareToClone(*tree_));
}

// Tests RigidBodyTree::Clone() using Valkyrie.
TEST_F(RigidBodyTreeCloneTest, CloneValkyrie) {
  std::string filename = drake::GetDrakePath() +
      "/examples/Valkyrie/urdf/urdf/"
      "valkyrie_A_sim_drake_one_neck_dof_wide_ankle_rom.urdf";
  // While it may seem odd to use a fixed floating joint with Valkyrie, it is
  // used in this case just to confirm that RigidBodyTree::Clone() works with
  // this type of joint. Previous unit tests already cover the quaternion
  // floating joint type.
  AddModelInstanceFromUrdfFileToWorld(filename, multibody::joints::kFixed,
      tree_.get());
  EXPECT_TRUE(CompareToClone(*tree_));
}

class TestRbtCloneDiagram : public Diagram<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TestRbtCloneDiagram);

  explicit TestRbtCloneDiagram(unique_ptr<RigidBodyTree<double>> tree) {
    tree_ = tree.get();
    DiagramBuilder<double> builder;

    plant_ =
        builder.template AddSystem<RigidBodyPlant<double>>(move(tree));

    const int num_actuators = plant_->actuator_command_input_port().size();
    auto constant_zero_source =
        builder.template AddSystem<ConstantVectorSource<double>>(
            VectorX<double>::Zero(num_actuators));

    logger_ = builder.template AddSystem<SignalLogger<double>>(
        plant_->get_num_states());

    builder.Connect(constant_zero_source->get_output_port(),
        plant_->actuator_command_input_port());
    builder.Connect(plant_->state_output_port(),
        logger_->get_input_port(0));
    builder.BuildInto(this);
  }

  /// Sets the initial state of the pendulum.
  ///
  /// @param[in] context This Diagram's context.
  /// @param[in] q The initial position of the pendulum's pivot joint.
  /// @param[in] v The initial velocity of the pendulum's pivot joint.
  void SetInitialState(Context<double>* context, double q, double v) {
    Context<double>* plant_context =
        GetMutableSubsystemContext(context, plant_);
    DRAKE_DEMAND(plant_context != nullptr);
    ContinuousState<double>* plant_state =
        plant_context->get_mutable_continuous_state();
    DRAKE_DEMAND(plant_state != nullptr);
    DRAKE_DEMAND(plant_state->size() == 2);
    (*plant_state)[0] = q;
    (*plant_state)[1] = v;
  }

  const SignalLogger<double>& get_logger() const { return *logger_; }

 private:
  RigidBodyTree<double>* tree_{nullptr};
  RigidBodyPlant<double>* plant_{nullptr};
  SignalLogger<double>* logger_{nullptr};
};

// Tests RigidBodyTree::Clone() by running two simulation, one using the
// original and another using a clone and verifying that the two behave
// identically.
TEST_F(RigidBodyTreeCloneTest, PendulumDynamicsTest) {
  const std::string model_file_name =
      GetDrakePath() + "/examples/Pendulum/Pendulum.urdf";
  const std::string model_name = "Pendulum";
  auto tree = std::make_unique<RigidBodyTree<double>>();
  AddModelInstanceFromUrdfFileToWorld(
      model_file_name, drake::multibody::joints::kFixed, tree.get());
  TestRbtCloneDiagram cloned_diagram(tree->Clone());
  TestRbtCloneDiagram original_diagram(std::move(tree));

  // Instantiates the contexts.
  unique_ptr<Context<double>> original_context =
      original_diagram.AllocateContext();
  original_diagram.SetDefaultState(
      *original_context, original_context->get_mutable_state());
  original_diagram.SetInitialState(original_context.get(), 1.57, 0);

  unique_ptr<Context<double>> cloned_context =
      cloned_diagram.AllocateContext();
  cloned_diagram.SetDefaultState(
      *cloned_context, cloned_context->get_mutable_state());
  cloned_diagram.SetInitialState(cloned_context.get(), 1.57, 0);

  // Instantiates the simulators.
  Simulator<double> original_simulator(
      original_diagram, std::move(original_context));
  Simulator<double> cloned_simulator(
      cloned_diagram, std::move(cloned_context));

  // TODO(liang.fok) Consider switching to a variable step size integrator so
  // the integrator step size need not be computed in the following manner.
  //
  // Computes a reasonable integrator step size. To do this, we use estimate the
  // swing period of the pendulum, which is approximated by the equation below:
  //
  //     swing_period = 2 * pi * sqrt(L / g)
  //
  // where L is the pendulum's center of mass, and g is the acceleration due to
  // gravity. In this case, L = 0.5 m and g = 9.81 m/s^2.
  //
  const double swing_period = 2 * M_PI * std::sqrt(0.5 / 9.81);

  // We arbitrarily compute 100 steps per pendulum swing period. This should be
  // sufficient to test the original RigidBodyTree against its clone.
  const double integrator_step_size = swing_period / 100;
  original_simulator.reset_integrator<ExplicitEulerIntegrator<double>>(
      original_diagram, integrator_step_size,
      original_simulator.get_mutable_context());
  cloned_simulator.reset_integrator<ExplicitEulerIntegrator<double>>(
      cloned_diagram, integrator_step_size,
      cloned_simulator.get_mutable_context());

  original_simulator.Initialize();
  cloned_simulator.Initialize();

  const SignalLogger<double>& original_logger = original_diagram.get_logger();
  const SignalLogger<double>& cloned_logger = cloned_diagram.get_logger();

  original_simulator.StepTo(swing_period);
  cloned_simulator.StepTo(swing_period);
  ASSERT_TRUE(CompareMatrices(original_logger.data(), cloned_logger.data()));
}

}  // namespace
}  // namespace test
}  // namespace plants
}  // namespace systems
}  // namespace drake

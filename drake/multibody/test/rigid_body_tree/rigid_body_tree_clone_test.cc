#include "drake/multibody/rigid_body_tree.h"

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
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/primitives/constant_vector_source.h"

using std::unique_ptr;

namespace drake {

using multibody::CompareToClone;
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

/// A sink block that logs the most recent plant state.
class TestRbtCloneLogger : public LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TestRbtCloneLogger);

  /// Constructs the accelerometer test logger system.
  ///
  /// @param plant_state_size The size of the plant's state.
  explicit TestRbtCloneLogger(int plant_state_size) {
    plant_state_port_index_ =
      this->DeclareInputPort(kVectorValued, plant_state_size).get_index();
  }

  // void enable_log_to_console() { log_to_console_ = true; }

  Eigen::VectorXd get_plant_state(const Context<double>& context) const {
    DRAKE_ASSERT_VOID(System<double>::CheckValidContext(context));
    return this->EvalVectorInput(context, plant_state_port_index_)->get_value();
  }
  // Eigen::VectorXd get_plant_state_derivative(const Context<double>& context)
  //     const;
  // Eigen::VectorXd get_acceleration(const Context<double>& context) const;

  const InputPortDescriptor<double>& get_plant_state_input_port() const {
    return System<double>::get_input_port(plant_state_port_index_);
  }
  // const InputPortDescriptor<double>& get_plant_state_derivative_input_port()
      // const;
  // const InputPortDescriptor<double>& get_acceleration_input_port() const;

 private:
  // No output.
  void DoCalcOutput(const Context<double>& context,
                    SystemOutput<double>* output) const override {}

  // No publish.
  void DoPublish(const Context<double>& context) const override {}

  // bool log_to_console_{false};
  // int plant_state_derivative_port_index_{};
  int plant_state_port_index_{};
  // int acceleration_port_index_{};
};

class TestRbtCloneDiagram : public Diagram<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TestRbtCloneDiagram);

  TestRbtCloneDiagram(unique_ptr<RigidBodyTree<double>> tree) {
    tree_ = tree.get();

    // Adds a pendulum to the RigidBodyTree and obtains its model instance ID.
    // const parsers::ModelInstanceIdTable table =

    // model_instance_id_ = table.at(model_name);

    // Specifies the location of the accelerometer sensor.
    // Eigen::Isometry3d sensor_frame_transform = Eigen::Isometry3d::Identity();
    // sensor_frame_transform.translation() << 0, 0, -0.5;

    // Adds a frame to the RigidBodyTree called "sensor frame" that is coincident
    // with the "arm" body within the RigidBodyTree.
    // sensor_frame_ = std::allocate_shared<RigidBodyFrame<double>>(
    //     Eigen::aligned_allocator<RigidBodyFrame<double>>(), "sensor frame",
    //    tree->FindBody("arm"), sensor_frame_transform);
    // tree->addFrame(sensor_frame_);
    DiagramBuilder<double> builder;

    plant_ =
        builder.template AddSystem<RigidBodyPlant<double>>(move(tree));

    const int num_actuators = plant_->actuator_command_input_port().size();
    auto constant_zero_source =
        builder.template AddSystem<ConstantVectorSource<double>>(
            VectorX<double>::Zero(num_actuators));

    logger_ = builder.template AddSystem<TestRbtCloneLogger>(
        plant_->get_num_states());

  // builder.Connect(lcm_subscriber_->get_output_port(0),
  //                  xdot_hack_->get_input_port());
  // builder.Connect(xdot_hack_->get_output_port(),
  //                  accelerometer_->get_plant_state_derivative_input_port());
    builder.Connect(constant_zero_source->get_output_port(),
        plant_->actuator_command_input_port());
    builder.Connect(plant_->state_output_port(),
        logger_->get_plant_state_input_port());
    builder.BuildInto(this);
  // builder.Connect(plant_->state_output_port(),
  //                  logger_->get_plant_state_input_port());
  // builder.Connect(xdot_hack_->get_output_port(),
  //                  logger_->get_plant_state_derivative_input_port());
  // builder.Connect(accelerometer_->get_output_port(),
  //                  logger_->get_acceleration_input_port());
  }

  /// Initializes this diagram.
  ///
  /// @param[in] visualizer The visualizer to include with the diagram. This can
  /// be nullptr.
  // void Initialize(std::unique_ptr<DrakeVisualizer> visualizer = nullptr);

  /// @name Mutators
  //@{

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
  //@}

  /// @name Accessors
  //@{
  // const Accelerometer& get_accelerometer() const { return *accelerometer_; }

  // const RigidBodyTree<double>& get_tree() const { return *tree_; }

  // RigidBodyPlantThatPublishesXdot<double>* get_plant() const {
  //     return plant_;
  // }

  // int get_model_instance_id() const { return model_instance_id_; }

  // const RigidBodyFrame<double>& get_sensor_frame() const {
  //   return *sensor_frame_.get();
  // }

  const TestRbtCloneLogger& get_logger() const { return *logger_; }
  // AccelerometerTestLogger* get_mutable_logger() { return logger_; }
  //@}

 private:
  // ::drake::lcm::DrakeLcmInterface* lcm_;
   // builder;
  RigidBodyTree<double>* tree_{nullptr};
  // int model_instance_id_{};
  // std::shared_ptr<RigidBodyFrame<double>> sensor_frame_;
  RigidBodyPlant<double>* plant_{nullptr};
  // AccelerometerXdotHack* xdot_hack_{nullptr};
  // std::unique_ptr<lcm::LcmtDrakeSignalTranslator> translator_;
  // lcm::LcmSubscriberSystem* lcm_subscriber_{nullptr};
  // Accelerometer* accelerometer_{nullptr};
  TestRbtCloneLogger* logger_{nullptr};
  // DrakeVisualizer* visualizer_{nullptr};
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

  // Decrease the step size for increased simulation accuracy.
  const double stepSize = 1e-4;
  original_simulator.get_mutable_integrator()->set_maximum_step_size(stepSize);
  original_simulator.Initialize();
  cloned_simulator.get_mutable_integrator()->set_maximum_step_size(stepSize);
  cloned_simulator.Initialize();

  const TestRbtCloneLogger& original_logger = original_diagram.get_logger();
  const TestRbtCloneLogger& cloned_logger = cloned_diagram.get_logger();

  const Context<double>& original_simulator_context =
      original_simulator.get_context();
  const Context<double>& original_logger_context =
      original_diagram.GetSubsystemContext(original_simulator_context,
          &original_logger);
  const Context<double>& cloned_simulator_context =
      cloned_simulator.get_context();
  const Context<double>& cloned_logger_context =
      cloned_diagram.GetSubsystemContext(cloned_simulator_context,
          &cloned_logger);

  // Simulate! Each time verifying that the states of the two simulations are
  // identical.
  const double kTimeStep = 0.01;
  const double kMaxTime = 1;
  for (double time = 0; time < kMaxTime; time += kTimeStep) {
    original_simulator.StepTo(time);
    cloned_simulator.StepTo(time);
    const Eigen::VectorXd original_state =
        original_logger.get_plant_state(original_logger_context);
    const Eigen::VectorXd cloned_state =
        cloned_logger.get_plant_state(cloned_logger_context);
    EXPECT_TRUE(CompareMatrices(original_state, cloned_state));
  }
}

}  // namespace
}  // namespace test
}  // namespace plants
}  // namespace systems
}  // namespace drake

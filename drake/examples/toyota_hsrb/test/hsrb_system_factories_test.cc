#include "drake/examples/toyota_hsrb/hsrb_diagram_factories.h"

#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "drake/common/drake_assert.h"
#include "drake/common/drake_path.h"
#include "drake/common/eigen_types.h"
#include "drake/lcm/drake_mock_lcm.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/system.h"
#include "drake/systems/framework/system_input.h"
#include "drake/systems/primitives/constant_vector_source.h"

using Eigen::Isometry3d;
using Eigen::Quaterniond;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;
using std::endl;
using std::getline;
using std::ifstream;
using std::make_unique;
using std::move;
using std::string;
using std::stringstream;
using std::unique_ptr;

namespace drake {

using lcm::DrakeMockLcm;
using systems::BasicVector;
using systems::Context;
using systems::ConstantVectorSource;
using systems::ContinuousState;
using systems::Diagram;
using systems::DrakeVisualizer;
using systems::FreestandingInputPort;
using systems::KinematicsResults;
using systems::RigidBodyPlant;
using systems::System;
using systems::SystemOutput;

namespace examples {
namespace toyota_hsrb {
namespace {

string ReadTextFile(const string& file) {
  ifstream text_file(file);
  DRAKE_DEMAND(text_file.is_open());

  stringstream buffer;
  string line;
  while (getline(text_file, line)) {
    buffer << line << endl;
  }
  text_file.close();
  return buffer.str();
}

// TODO(liang.fok) Combine with near-identical code in rigid_body_plant_test.cc.
template <class T>
unique_ptr<FreestandingInputPort> MakeInput(unique_ptr<BasicVector<T>> data) {
  return make_unique<FreestandingInputPort>(move(data));
}

const char* kModelFileName = "/examples/toyota_hsrb/test/test_model.urdf";
const int kNumPositions{8};   // 7 floating DOFs and 1 regular DOF.
const int kNumVelocities{7};  // 6 floating DOFs and 1 regular DOF.
const int kNumActuators{1};   // Just the 1 regular DOF is actuated.
const int kNumStates{kNumPositions + kNumVelocities};

class ToyotaHsrbTests : public ::testing::Test {
 protected:
  void SetUp() override {
    // A test model is used in place of the Toyota HSRb since:
    //
    //   (1) The factory methods being tested are general and should work with
    //       all URDFs.
    //
    //   (2) HSRb URDFs need to be generated using `xacro`, the latest version
    //       of which is currently not supported by Drake.
    //
    // TODO(liang.fok): Get `xacro` to work with Drake and then update this unit
    // test to use the Toyota HSRb model. See #4326.
    const string urdf_string = ReadTextFile(GetDrakePath() + kModelFileName);
    const double penetration_stiffness = 4000;
    const double penetration_damping = 300;
    const double friction_coefficient = 10;

    plant_and_visualizer_ = BuildPlantAndVisualizerDiagram(
        urdf_string, penetration_stiffness, penetration_damping,
        friction_coefficient, &lcm_, &plant_);

    ASSERT_NE(plant_and_visualizer_, nullptr);
    ASSERT_NE(plant_, nullptr);

    // Verifies that `plant_and_visualizer_` contains two systems, a
    // `RigidBodyPlant` and a `DrakeVisualizer`, located at indices 0 and 1,
    // respectively, within the `Diagram`.
    std::vector<const System<double>*> systems =
        plant_and_visualizer_->GetSystems();
    EXPECT_EQ(systems.size(), 2u);
    const RigidBodyPlant<double>* rigid_body_plant =
        dynamic_cast<const RigidBodyPlant<double>*>(systems.at(0));
    visualizer_ =
        dynamic_cast<const DrakeVisualizer*>(systems.at(1));
    ASSERT_NE(rigid_body_plant, nullptr);
    ASSERT_NE(visualizer_, nullptr);

    lcm_.StartReceiveThread();
  }

  DrakeMockLcm lcm_;
  unique_ptr<Diagram<double>> plant_and_visualizer_{nullptr};
  RigidBodyPlant<double>* plant_{nullptr};
  const DrakeVisualizer* visualizer_{nullptr};
};

// TODO(liang.fok): Much of the code below is a duplicate of what's in
// rigid_body_plant_test.cc and drake_visualizer_test.cc. Consider unifying once
// rule-of-three is met.


// Verifies that @p message_bytes is correct. Only a few key fields are checked.
// The rest of the message is assumed to be correct.
// See drake_visualizer_test.cc for a more comprehensive test.
void VerifyLoadMessage(const std::vector<uint8_t>& message_bytes) {
  drake::lcmt_viewer_load_robot message;
  ASSERT_EQ(message.decode(message_bytes.data(), 0, message_bytes.size()),
            static_cast<int>(message_bytes.size()));
  ASSERT_EQ(message.num_links, 3);
  EXPECT_EQ(message.link.at(0).name, "world");
  EXPECT_EQ(message.link.at(1).name, "link1");
  EXPECT_EQ(message.link.at(2).name, "link2");
  EXPECT_EQ(message.link.at(0).robot_num, 0);
  EXPECT_EQ(message.link.at(1).robot_num, 0);
  EXPECT_EQ(message.link.at(2).robot_num, 0);
  EXPECT_EQ(message.link.at(0).num_geom, 1);
  EXPECT_EQ(message.link.at(1).num_geom, 1);
  EXPECT_EQ(message.link.at(2).num_geom, 1);
}

// Verifies that @p message_bytes is correct. Only a few key fields are checked.
// The rest of the message is assumed to be correct. See
// drake_visualizer_test.cc for a more comprehensive test.
void VerifyDrawMessage(const std::vector<uint8_t>& message_bytes) {
  drake::lcmt_viewer_draw expected_message;
  ASSERT_EQ(
      expected_message.decode(message_bytes.data(), 0, message_bytes.size()),
      static_cast<int>(message_bytes.size()));
  ASSERT_EQ(expected_message.num_links, 3);
  EXPECT_EQ(expected_message.timestamp, 0);
  EXPECT_EQ(expected_message.link_name.at(0), "world");
  EXPECT_EQ(expected_message.link_name.at(1), "link1");
  EXPECT_EQ(expected_message.link_name.at(2), "link2");
  EXPECT_EQ(expected_message.robot_num.at(0), 0);
  EXPECT_EQ(expected_message.robot_num.at(1), 0);
  EXPECT_EQ(expected_message.robot_num.at(2), 0);
}

void VerifyDiagram(const Diagram<double>& dut, const VectorXd& desired_state,
                   const RigidBodyPlant<double>& plant,
                   const DrakeVisualizer& visualizer, const DrakeMockLcm& lcm,
                   Context<double>* context) {
  std::unique_ptr<SystemOutput<double>> output = dut.AllocateOutput(*context);
  std::unique_ptr<ContinuousState<double>> derivatives =
      dut.AllocateTimeDerivatives();

  ASSERT_EQ(dut.get_num_output_ports(), plant.get_num_output_ports());
  ASSERT_EQ(output->get_num_ports(), dut.get_num_output_ports());

  VectorXd xc = context->get_continuous_state()->CopyToVector();
  ASSERT_EQ(xc, desired_state);

  const BasicVector<double>* output_state = output->get_vector_data(0);
  ASSERT_NE(nullptr, output_state);

  dut.CalcOutput(*context, output.get());
  dut.Publish(*context);

  // Asserts the output equals the state.
  EXPECT_EQ(desired_state, output_state->get_value());

  // Evaluates the correctness of the kinematics results port.
  auto& kinematics_results =
      output->get_data(1)->GetValue<KinematicsResults<double>>();
  ASSERT_EQ(kinematics_results.get_num_positions(), kNumPositions);
  ASSERT_EQ(kinematics_results.get_num_velocities(), kNumVelocities);

  VectorXd q = xc.topRows(kNumPositions);
  VectorXd v = xc.bottomRows(kNumVelocities);

  const auto& tree = plant.get_rigid_body_tree();
  auto cache = tree.doKinematics(q, v);

  for (int ibody = 0; ibody < plant.get_num_bodies(); ++ibody) {
    Isometry3d pose = tree.relativeTransform(cache, 0, ibody);
    Vector4d quat_vector = drake::math::rotmat2quat(pose.linear());
    // Note that Eigen quaternion elements are not laid out in memory in the
    // same way Drake currently aligns them. See issue #3470.
    // When solved we will not need to instantiate a temporary Quaternion below
    // just to perform a comparison.
    Quaterniond quat(quat_vector[0], quat_vector[1], quat_vector[2],
                     quat_vector[3]);
    Vector3d position = pose.translation();
    EXPECT_TRUE(quat.isApprox(kinematics_results.get_body_orientation(ibody)));
    EXPECT_TRUE(position.isApprox(kinematics_results.get_body_position(ibody)));
  }

  // // Verifies that the correct LCM messages were published
  VerifyLoadMessage(lcm.get_last_published_message("DRAKE_VIEWER_LOAD_ROBOT"));
  VerifyDrawMessage(lcm.get_last_published_message("DRAKE_VIEWER_DRAW"));
}

// Tests BuildPlantAndVisualizerDiagram().
TEST_F(ToyotaHsrbTests, TestBuildPlantAndVisualizerDiagram) {
  Diagram<double>* dut = plant_and_visualizer_.get();
  ASSERT_EQ(dut->get_num_input_ports(), plant_->get_num_input_ports());

  std::unique_ptr<Context<double>> context = dut->CreateDefaultContext();
  ASSERT_EQ(context->get_num_input_ports(), dut->get_num_input_ports());

  ASSERT_EQ(plant_->get_num_positions(), kNumPositions);
  ASSERT_EQ(plant_->get_num_velocities(), kNumVelocities);
  ASSERT_EQ(plant_->get_num_states(), kNumStates);
  ASSERT_EQ(plant_->get_num_actuators(), kNumActuators);
  ASSERT_EQ(dut->get_input_port(0).get_size(), kNumActuators);

  // Connect to a "fake" free standing input.
  // TODO(amcastro-tri): Connect to a ConstantVectorSource once Diagrams have
  // derivatives per #3218.
  context->SetInputPort(0, MakeInput(make_unique<BasicVector<double>>(
                                         plant_->get_num_actuators())));

  // Sets the state to a non-zero value.
  VectorXd desired_state(kNumStates);
  desired_state << 0.5, 0.1, -0.1, 0.2, 0.3, -0.2, 0.15, 0.3,
                    VectorXd::Zero(kNumVelocities);
  for (int i = 0; i < kNumPositions; ++i) {
    plant_->set_position(context.get(), i, desired_state[i]);
  }
  for (int i = 0; i < kNumVelocities; ++i) {
    plant_->set_velocity(context.get(), i, desired_state[kNumPositions + i]);
  }

  VerifyDiagram(*dut, desired_state, *plant_, *visualizer_, lcm_,
                context.get());
}

// Tests BuildConstantSourceToPlantDiagram().
TEST_F(ToyotaHsrbTests, TestBuildConstantSourceToPlantDiagram) {
  unique_ptr<Diagram<double>> dut = BuildConstantSourceToPlantDiagram(
                                        std::move(plant_and_visualizer_));
  ASSERT_NE(dut, nullptr);

  // The DUT should not have any input ports since the plant's input port is
  // connected to a constant vector source, which has no inputs. All of the
  // plant's output ports should be exported by the DUT.
  EXPECT_EQ(dut->get_num_input_ports(), 0);
  EXPECT_EQ(dut->get_num_output_ports(), plant_->get_num_output_ports());

  EXPECT_EQ(dut->get_output_port(0).get_size(), kNumStates);

  // Verifies all expected systems exist.
  std::vector<const System<double>*> systems = dut->GetSystems();
  EXPECT_EQ(systems.size(), 2u);
  const Diagram<double>* plant_and_visualizer =
      dynamic_cast<const Diagram<double>*>(systems.at(0));
  const ConstantVectorSource<double>* const_source =
      dynamic_cast<const ConstantVectorSource<double>*>(systems.at(1));
  ASSERT_NE(plant_and_visualizer, nullptr);
  ASSERT_NE(const_source, nullptr);

  std::unique_ptr<Context<double>> context = dut->CreateDefaultContext();

  VectorXd desired_state(kNumStates);
  desired_state << VectorXd::Zero(kNumStates);
  desired_state(3) = 1;  // This is the `w` in the quaternion floating DOF.

  VerifyDiagram(*dut, desired_state, *plant_, *visualizer_, lcm_,
                context.get());
}

}  // namespace
}  // namespace toyota_hsrb
}  // namespace examples
}  // namespace drake


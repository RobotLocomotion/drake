#include "drake/systems/controllers/pid_controlled_system.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/saturation.h"

namespace drake {
namespace systems {
namespace controllers {
namespace {

// This is the parent class of the two test plants defined below.
class TestPlant : public LeafSystem<double> {
 public:
  TestPlant() {}

  double GetInputValue(const Context<double>& context) {
    return get_input_port(0).Eval(context)[0];
  }
};

// A plant with an input port zero of size 1 and an output port zero of size 2.
// This is the minimum sized output port zero relative to the size of its input
// port zero, meaning all of the elements in its output port zero are fed back
// to the PID controller. A diagram of the resulting PidControlledSystem is
// given below, where X is the desired state of the plant and Q is the actual
// state of the plant.
//
//        2    +---------------+
//   X ---/--->|               |  1   +-----------+   2
//             | PidController |--/-->| TestPlant |---/---+---> Q
//         +-->|               |      +-----------+       |
//         |   +---------------+                          |
//         |                                              |
//         +----------------------------------------------+
//
class TestPlantWithMinOutputs : public TestPlant {
 public:
  TestPlantWithMinOutputs() {
    DeclareVectorInputPort(kUseDefaultName, 1);
    DeclareVectorOutputPort(kUseDefaultName, 2,
                            &TestPlantWithMinOutputs::CalcOutputVector,
                            {this->nothing_ticket()});
  }

  void CalcOutputVector(const Context<double>&,
                        BasicVector<double>* output) const {
    BasicVector<double>& output_vector = *output;
    output_vector[0] = 1.;
    output_vector[1] = 0.1;
  }
};

// A plant with an input port one of size 1 and an output port zero of size 6.
// The plant's output port zero has four elements that should not be fed back
// to the PID controller. A feedback selector system is used to determine which
// two elements of the plant's output port zero are fed back to the PID
// controller. A diagram of the resulting PidControlledSystem is given below,
// where X is the desired state of the plant and Q is the actual state of the
// plant.
//
//        2    +---------------+
//   X ---/--->|               |  1   +-----------+   6
//             | PidController |--/-->| TestPlant |---/---+---> Q
//         +-->|               |      +-----------+       |
//         |   +---------------+                          |
//         |                                              |
//         |          2           +------------------+    |
//         +----------/-----------| FeedbackSelector |----+
//                                +------------------+
//
class TestPlantWithMoreOutputs : public TestPlant {
 public:
  TestPlantWithMoreOutputs() {
    DeclareVectorInputPort(kUseDefaultName, 1);
    DeclareVectorOutputPort(kUseDefaultName, 6,
                            &TestPlantWithMoreOutputs::CalcOutputVector,
                            {this->nothing_ticket()});
  }

  void CalcOutputVector(const Context<double>&,
                        BasicVector<double>* output) const {
    BasicVector<double>& output_vector = *output;
    output_vector[0] = 1.;
    output_vector[1] = 0.1;
    output_vector[2] = 3.14;
    output_vector[3] = 6.89;
    output_vector[4] = 90.37;
    output_vector[5] = 498.9;
  }
};

class TestPlantWithMoreOutputPorts : public TestPlant {
 public:
  TestPlantWithMoreOutputPorts() {
    // Declare some empty plant input port.
    DeclareVectorInputPort(kUseDefaultName, 0);
    DeclareVectorInputPort(kUseDefaultName, 1);
    // Declare some non-state output port.
    DeclareVectorOutputPort(
        kUseDefaultName, 3,
        &TestPlantWithMoreOutputPorts::CalcNonStateOutputVector,
        {this->nothing_ticket()});
    DeclareVectorOutputPort(
        kUseDefaultName, 2,
        &TestPlantWithMoreOutputPorts::CalcStateOutputVector,
        {this->nothing_ticket()});
  }

  void CalcNonStateOutputVector(const Context<double>&,
                                BasicVector<double>* output) const {
    BasicVector<double>& output_vector = *output;
    output_vector[0] = 2.;
    output_vector[1] = 0.2;
    output_vector[2] = 0.02;
  }

  void CalcStateOutputVector(const Context<double>&,
                        BasicVector<double>* output) const {
    BasicVector<double>& output_vector = *output;
    output_vector[0] = 43.;
    output_vector[1] = 0.68;
  }
};

class PidControlledSystemTest : public ::testing::Test {
 protected:
  // Instantiates a PidControlledSystem based on the supplied plant and
  // feedback selector. Verifies that the output of the PID controller is
  // correct relative to the hard-coded input and gain values.
  void DoPidControlledSystemTest(
      std::unique_ptr<TestPlant> plant,
      const MatrixX<double>& feedback_selector) {
    DiagramBuilder<double> builder;
    const Vector1d input(1.);
    const Eigen::Vector2d state(1.1, 0.2);
    auto input_source = builder.AddSystem<ConstantVectorSource>(input);
    input_source->set_name("input");
    auto state_source = builder.AddSystem<ConstantVectorSource>(state);
    state_source->set_name("state");

    auto controller = builder.AddSystem<PidControlledSystem>(
        std::move(plant), feedback_selector, Kp_, Ki_, Kd_);

    builder.Connect(input_source->get_output_port(),
                    controller->get_input_port(0));
    builder.Connect(state_source->get_output_port(),
                    controller->get_input_port(1));
    builder.ExportOutput(controller->get_output_port(0));
    diagram_ = builder.Build();
    auto context = diagram_->CreateDefaultContext();
    auto output = diagram_->AllocateOutput();

    const systems::Context<double>& plant_context =
        diagram_->GetSubsystemContext(*controller->plant(),
                                      *context);

    diagram_->CalcOutput(*context, output.get());
    const BasicVector<double>* output_vec = output->get_vector_data(0);
    const double pid_input = dynamic_cast<TestPlant*>(controller->plant())
                                 ->GetInputValue(plant_context);
    EXPECT_EQ(pid_input, input[0] +
                             (state[0] - output_vec->get_value()[0]) * Kp_(0) +
                             (state[1] - output_vec->get_value()[1]) * Kd_(0));
  }

  const Vector1d Kp_{2};
  const Vector1d Ki_{0};
  const Vector1d Kd_{0.1};
  std::unique_ptr<Diagram<double>> diagram_;
};

// Tests that the PidController preserves the names of the plant.
TEST_F(PidControlledSystemTest, ExistingNamesRespected) {
  auto plant = std::make_unique<TestPlantWithMinOutputs>();
  plant->set_name("my awesome plant!");
  auto plant_ptr = plant.get();
  const int state_size = plant->get_output_port(0).size();

  PidControlledSystem<double> controller(
      std::move(plant), MatrixX<double>::Identity(state_size, state_size),
      Kp_, Ki_, Kd_);
  EXPECT_EQ("my awesome plant!", plant_ptr->get_name());
}

// Tests a plant where the size of output port zero is twice the size of input
// port zero.
TEST_F(PidControlledSystemTest, SimplePidControlledSystem) {
  // Our test plant is just a multiplexer which takes the input and outputs it
  // twice.
  auto plant = std::make_unique<TestPlantWithMinOutputs>();
  const int state_size = plant->get_output_port(0).size();
  DoPidControlledSystemTest(std::move(plant),
      MatrixX<double>::Identity(state_size, state_size));
}

// Tests a plant where the size of output port zero is more than twice the size
// of input port zero.
TEST_F(PidControlledSystemTest, PlantWithMoreOutputs) {
  auto plant = std::make_unique<TestPlantWithMoreOutputs>();
  const int plant_output_size = plant->get_output_port(0).size();
  const int controller_feedback_size = plant->get_input_port(0).size() * 2;

  // Selects the first two signals from the plant's output port zero as the
  // feedback for the controller.
  MatrixX<double> feedback_selector_matrix(controller_feedback_size,
                                           plant_output_size);
  EXPECT_EQ(feedback_selector_matrix.rows(), 2);
  EXPECT_EQ(feedback_selector_matrix.cols(), 6);
  feedback_selector_matrix << 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0;

  DoPidControlledSystemTest(std::move(plant), feedback_selector_matrix);
}

// Tests that additional output ports of the plant are exposed through the
// new diagram.
TEST_F(PidControlledSystemTest, PlantWithMoreOutputPorts) {
  auto plant = std::make_unique<TestPlantWithMoreOutputPorts>();
  const int state_output_port_index = 1;
  const int plant_input_port_index = 1;
  PidControlledSystem<double> system(std::move(plant), Kp_, Ki_, Kd_,
                                     state_output_port_index,
                                     plant_input_port_index);

  EXPECT_EQ(system.num_output_ports(), 2);
  EXPECT_EQ(system.get_output_port(0).size(), 3);
  EXPECT_EQ(system.get_output_port(1).size(), 2);

  auto context = system.CreateDefaultContext();
  auto output = system.AllocateOutput();

  system.CalcOutput(*context, output.get());

  EXPECT_EQ(output->get_vector_data(0)->GetAtIndex(1), 0.2);
  EXPECT_EQ(output->get_vector_data(1)->GetAtIndex(1), 0.68);
}

class ConnectControllerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    auto plant_ptr = std::make_unique<TestPlantWithMinOutputs>();
    feedback_selector_ = MatrixX<double>::Identity(
        plant_ptr->get_output_port(0).size(),
        plant_ptr->get_output_port(0).size());

    plant_ = builder_.AddSystem(std::move(plant_ptr));

    input_source_ = builder_.AddSystem<ConstantVectorSource>(plant_input_);
    state_source_ = builder_.AddSystem<ConstantVectorSource>(desired_state_);
    builder_.ExportOutput(plant_->get_output_port(0));
  }

  void ConnectPidPorts(
      PidControlledSystem<double>::ConnectResult plant_pid_ports) {
    builder_.Connect(input_source_->get_output_port(),
                     plant_pid_ports.control_input_port);
    builder_.Connect(state_source_->get_output_port(),
                     plant_pid_ports.state_input_port);
  }

  double ComputePidInput() {
    auto standard_diagram = builder_.Build();
    auto context = standard_diagram->CreateDefaultContext();

    auto plant_output =
        standard_diagram->GetSubsystemByName(plant_->get_name())
        .AllocateOutput();
    auto& plant_context =
        standard_diagram->GetSubsystemContext(*plant_, *context);

    plant_->CalcOutput(plant_context, plant_output.get());
    const BasicVector<double>* output_vec = plant_output->get_vector_data(0);
    const double pid_input =
        dynamic_cast<TestPlant*>(plant_)->GetInputValue(plant_context);

    output_position_ = output_vec->get_value()[0];
    output_velocity_ = output_vec->get_value()[1];
    return pid_input;
  }

  TestPlantWithMinOutputs* plant_;
  const Vector1d Kp{4};
  const Vector1d Ki{0};
  const Vector1d Kd{0.5};

  DiagramBuilder<double> builder_;
  ConstantVectorSource<double>* input_source_ = nullptr;
  ConstantVectorSource<double>* state_source_ = nullptr;
  MatrixX<double> feedback_selector_;

  const Vector1d plant_input_{1.0};
  const Eigen::Vector2d desired_state_{1.1, 0.2};
  double output_position_{0.0};
  double output_velocity_{0.0};
};

// Tests a plant where the controller is attached with the ConnectController
// method.
TEST_F(ConnectControllerTest, NonSaturatingController) {
  auto plant_pid_ports = PidControlledSystem<double>::ConnectController(
      plant_->get_input_port(0), plant_->get_output_port(0),
      feedback_selector_, Kp, Ki, Kd, &builder_);

  ConnectPidPorts(plant_pid_ports);

  const double pid_input = ComputePidInput();
  double calculated_input = plant_input_[0] +
                            (desired_state_[0] - output_position_) * Kp(0) +
                            (desired_state_[1] - output_velocity_) * Kd(0);

  EXPECT_EQ(pid_input, calculated_input);
}

// Tests a plant where the controller is attached with the ConnectController
// method and a Saturation in the input to the plant.
TEST_F(ConnectControllerTest, SaturatingController) {
  // Sets the max input in the case of saturation_test.
  const double saturation_max = 0.5;

  auto plant_pid_ports =
      PidControlledSystem<double>::ConnectControllerWithInputSaturation(
          plant_->get_input_port(0), plant_->get_output_port(0),
          feedback_selector_, Kp, Ki, Kd, Vector1d(0.0) /* u_min */,
          Vector1d(saturation_max), &builder_);

  ConnectPidPorts(plant_pid_ports);

  const double pid_input = ComputePidInput();
  double calculated_input = saturation_max;

  EXPECT_EQ(pid_input, calculated_input);
}

// Ensure that multiple plants can coexist in the same diagram (yes,
// this was broken at one point).
TEST_F(ConnectControllerTest, MultipleControllerTest) {
  auto plant_pid_ports = PidControlledSystem<double>::ConnectController(
      plant_->get_input_port(0), plant_->get_output_port(0),
      feedback_selector_, Kp, Ki, Kd, &builder_);

  ConnectPidPorts(plant_pid_ports);

  // Add another PidControlledSystem and confirm there's nothing that
  // prevents two plants from existing in the same diagram.
  SetUp();
  auto second_plant_pid_ports = PidControlledSystem<double>::ConnectController(
      plant_->get_input_port(0), plant_->get_output_port(0),
      feedback_selector_, Kp, Ki, Kd, &builder_);

  ConnectPidPorts(second_plant_pid_ports);
  DRAKE_EXPECT_NO_THROW(builder_.Build());
}

}  // namespace
}  // namespace controllers
}  // namespace systems
}  // namespace drake

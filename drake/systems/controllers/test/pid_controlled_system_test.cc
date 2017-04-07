#include "drake/systems/controllers/pid_controlled_system.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/gain.h"
#include "drake/systems/primitives/matrix_gain.h"
#include "drake/systems/primitives/saturation.h"

namespace drake {
namespace systems {
namespace {

// This is the parent class of the two test plants defined below.
class TestPlant : public LeafSystem<double> {
 public:
  TestPlant() {}

  double GetInputValue(const Context<double>& context) {
    return EvalEigenVectorInput(context, 0)(0);
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
    DeclareVectorInputPort(BasicVector<double>(1));
    DeclareVectorOutputPort(BasicVector<double>(2),
                            &TestPlantWithMinOutputs::CalcOutputVector);
  }

  void CalcOutputVector(const Context<double>& context,
                        BasicVector<double>* output) const {
    BasicVector<double>& output_vector = *output;
    output_vector[0] = 1.;
    output_vector[1] = 0.1;
  }

 protected:
  bool DoHasDirectFeedthrough(const SparsityMatrix* sparsity, int input_port,
                              int output_port) const override {
    return false;
  }
};

// A plant with an input port zero of size 1 and an output port zero of size 6.
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
    DeclareVectorInputPort(BasicVector<double>(1));
    DeclareVectorOutputPort(BasicVector<double>(6),
                            &TestPlantWithMoreOutputs::CalcOutputVector);
  }

  void CalcOutputVector(const Context<double>& context,
                        BasicVector<double>* output) const {
    BasicVector<double>& output_vector = *output;
    output_vector[0] = 1.;
    output_vector[1] = 0.1;
    output_vector[2] = 3.14;
    output_vector[3] = 6.89;
    output_vector[4] = 90.37;
    output_vector[5] = 498.9;
  }

 protected:
  bool DoHasDirectFeedthrough(const SparsityMatrix* sparsity, int input_port,
                              int output_port) const override {
    return false;
  }
};

class PidControlledSystemTest : public ::testing::Test {
 protected:
  // Instantiates a PidControlledSystem based on the supplied plant and
  // feedback selector. Verifies that the output of the PID controller is
  // correct relative to the hard-coded input and gain values.
  void DoPidControlledSystemTest(
      std::unique_ptr<TestPlant> plant,
      std::unique_ptr<MatrixGain<double>> feedback_selector) {
    DiagramBuilder<double> builder;
    const Vector1d input(1.);
    const Eigen::Vector2d state(1.1, 0.2);
    auto input_source = builder.AddSystem<ConstantVectorSource>(input);
    input_source->set_name("input");
    auto state_source = builder.AddSystem<ConstantVectorSource>(state);
    state_source->set_name("state");

    auto controller = builder.AddSystem<PidControlledSystem>(
        std::move(plant), std::move(feedback_selector), Kp_, Ki_, Kd_);
    // Check that the controller automatically assigns a name to the plant.
    controller->set_name("controller");

    builder.Connect(input_source->get_output_port(),
                    controller->get_input_port(0));
    builder.Connect(state_source->get_output_port(),
                    controller->get_input_port(1));
    builder.ExportOutput(controller->get_output_port(0));
    diagram_ = builder.Build();
    auto context = diagram_->CreateDefaultContext();
    auto output = diagram_->AllocateOutput(*context);

    systems::Context<double>* controller_context =
        diagram_->GetMutableSubsystemContext(context.get(), controller);
    systems::Context<double>* plant_context =
        controller->GetMutableSubsystemContext(controller_context,
                                               controller->plant());

    diagram_->CalcOutput(*context, output.get());
    const BasicVector<double>* output_vec = output->get_vector_data(0);
    const double pid_input = dynamic_cast<TestPlant*>(controller->plant())
                                 ->GetInputValue(*plant_context);
    EXPECT_EQ(pid_input, input[0] +
                             (state[0] - output_vec->get_value()[0]) * Kp_(0) +
                             (state[1] - output_vec->get_value()[1]) * Kd_(0));
  }

  const Vector1d Kp_{2};
  const Vector1d Ki_{0};
  const Vector1d Kd_{0.1};
  std::unique_ptr<Diagram<double>> diagram_;
};

// Tests that the PidController assigns default names to the plant and the
// feedback selector, if they have no name.
TEST_F(PidControlledSystemTest, DefaultNamesAssigned) {
  auto plant = std::make_unique<TestPlantWithMinOutputs>();
  auto plant_ptr = plant.get();
  auto feedback_selector =
    std::make_unique<MatrixGain<double>>(plant->get_output_port(0).size());
  auto feedback_selector_ptr = feedback_selector.get();

  PidControlledSystem<double> controller(
      std::move(plant), std::move(feedback_selector), Kp_, Ki_, Kd_);
  EXPECT_EQ("plant", plant_ptr->get_name());
  EXPECT_EQ("custom_feedback_selector", feedback_selector_ptr->get_name());
}

// Tests that the PidController preserves the names of the plant and the
// feedback selector.
TEST_F(PidControlledSystemTest, ExistingNamesRespected) {
  auto plant = std::make_unique<TestPlantWithMinOutputs>();
  plant->set_name("my awesome plant!");
  auto plant_ptr = plant.get();
  auto feedback_selector =
      std::make_unique<MatrixGain<double>>(plant->get_output_port(0).size());
  feedback_selector->set_name("my boring feedback selector");
  auto feedback_selector_ptr = feedback_selector.get();

  PidControlledSystem<double> controller(
      std::move(plant), std::move(feedback_selector), Kp_, Ki_, Kd_);
  EXPECT_EQ("my awesome plant!", plant_ptr->get_name());
  EXPECT_EQ("my boring feedback selector", feedback_selector_ptr->get_name());
}

// Tests a plant where the size of output port zero is twice the size of input
// port zero.
TEST_F(PidControlledSystemTest, SimplePidControlledSystem) {
  // Our test plant is just a multiplexer which takes the input and outputs it
  // twice.
  auto plant = std::make_unique<TestPlantWithMinOutputs>();
  auto feedback_selector =
      std::make_unique<MatrixGain<double>>(plant->get_output_port(0).size());
  DoPidControlledSystemTest(std::move(plant), std::move(feedback_selector));
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

  auto feedback_selector =
      std::make_unique<MatrixGain<double>>(feedback_selector_matrix);

  DoPidControlledSystemTest(std::move(plant), std::move(feedback_selector));
}

class ConnectControllerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    auto plant_ptr = std::make_unique<TestPlantWithMinOutputs>();
    feedback_selector_ =
        std::make_unique<MatrixGain<double>>(
            plant_ptr->get_output_port(0).size());

    plant_ = builder_.AddSystem(std::move(plant_ptr));
    plant_->set_name("plant");

    input_source_ = builder_.AddSystem<ConstantVectorSource>(plant_input_);
    input_source_->set_name("input");
    state_source_ = builder_.AddSystem<ConstantVectorSource>(desired_state_);
    state_source_->set_name("state");

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
    auto output = standard_diagram->AllocateOutput(*context);

    auto plant_context =
        standard_diagram->GetMutableSubsystemContext(context.get(), plant_);

    plant_->CalcOutput(*plant_context, output.get());
    const BasicVector<double>* output_vec = output->get_vector_data(0);
    const double pid_input =
        dynamic_cast<TestPlant*>(plant_)->GetInputValue(*plant_context);

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
  std::unique_ptr<MatrixGain<double>> feedback_selector_;

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
      std::move(feedback_selector_), Kp, Ki, Kd, &builder_);

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
          std::move(feedback_selector_), Kp, Ki, Kd, Vector1d(0.0) /* u_min */,
          Vector1d(saturation_max), &builder_);

  ConnectPidPorts(plant_pid_ports);

  const double pid_input = ComputePidInput();
  double calculated_input = saturation_max;

  EXPECT_EQ(pid_input, calculated_input);
}

}  // namespace
}  // namespace systems
}  // namespace drake

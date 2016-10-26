#include "drake/systems/controllers/pid_controlled_system.h"

#include <memory>

#include "gtest/gtest.h"

#include "drake/common/eigen_types.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/primitives/constant_vector_source.h"
#include "drake/systems/framework/primitives/gain.h"

namespace drake {
namespace systems {
namespace {

class TestSystem : public LeafSystem<double> {
 public:
  TestSystem() {
    DeclareInputPort(kVectorValued, 1, kContinuousSampling);
    DeclareOutputPort(kVectorValued, 2, kContinuousSampling);
  }

  double GetInputValue(const Context<double>& context) {
    return EvalEigenVectorInput(context, 0)(0);
  }

  void EvalOutput(const Context<double>& context,
                  SystemOutput<double>* output) const override {
    auto output_vector = GetMutableOutputVector(output, 0);
    output_vector[0] = 1.;
    output_vector[1] = 0.1;
  }

  bool has_any_direct_feedthrough() const override { return false; }
};

// Tests a plant where the size of output port zero is twice the size of input
// port zero.
GTEST_TEST(PidControlledSystemTest, SimplePidControlledSystem) {
  DiagramBuilder<double> builder;
  const Vector1d input(1.);
  const Eigen::Vector2d state(1.1, 0.2);
  auto input_source = builder.AddSystem<ConstantVectorSource>(input);
  auto state_source = builder.AddSystem<ConstantVectorSource>(state);

  // Our test "system" is just a multiplexer which takes the input and
  // outputs it twice.
  auto test_plant = std::make_unique<TestSystem>();
  TestSystem* plant_ptr = test_plant.get();
  auto feedback_selector =
      std::make_unique<MimoGain<double>>(
          test_plant->get_output_port(0).get_size());
  const double Kp = 2.;
  const double Kd = .1;
  auto controller = builder.AddSystem<PidControlledSystem>(
      std::move(test_plant), std::move(feedback_selector), Kp, 0., Kd);
  builder.Connect(input_source->get_output_port(),
                  controller->get_input_port(0));
  builder.Connect(state_source->get_output_port(),
                  controller->get_input_port(1));
  builder.ExportOutput(controller->get_output_port(0));
  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();
  auto output = diagram->AllocateOutput(*context);

  systems::Context<double>* controller_context =
      diagram->GetMutableSubsystemContext(context.get(), controller);
  controller->SetDefaultState(controller_context);
  systems::Context<double>* test_context =
      controller->GetMutableSubsystemContext(
          controller_context, plant_ptr);

  diagram->EvalOutput(*context, output.get());
  const BasicVector<double>* output_vec = output->get_vector_data(0);
  const double pid_input = plant_ptr->GetInputValue(*test_context);
  EXPECT_EQ(pid_input, input[0] + (state[0] - output_vec->get_value()[0]) * Kp +
            (state[1] - output_vec->get_value()[1]) * Kd);
}

class TestSystemWithMoreOutputs : public LeafSystem<double> {
 public:
  TestSystemWithMoreOutputs() {
    DeclareInputPort(kVectorValued, 1, kContinuousSampling);
    DeclareOutputPort(kVectorValued, 6, kContinuousSampling);
  }

  double GetInputValue(const Context<double>& context) {
    return EvalEigenVectorInput(context, 0)(0);
  }

  void EvalOutput(const Context<double>& context,
                  SystemOutput<double>* output) const override {
    auto output_vector = GetMutableOutputVector(output, 0);
    output_vector[0] = 1.;
    output_vector[1] = 0.1;
    output_vector[2] = 3.14;
    output_vector[3] = 6.89;
    output_vector[4] = 90.37;
    output_vector[5] = 498.9;
  }

  bool has_any_direct_feedthrough() const override { return false; }
};

// Tests a plant where the size of output port zero is more than twice the size
// of input port zero.
GTEST_TEST(PidControlledSystemTest, PlantWithMoreOutputs) {
  DiagramBuilder<double> builder;
  const Vector1d input(1.);
  const Eigen::Vector2d state(1.1, 0.2);
  auto input_source = builder.AddSystem<ConstantVectorSource>(input);
  auto state_source = builder.AddSystem<ConstantVectorSource>(state);

  // Our test "system" is just a multiplexer which takes the input and
  // outputs it twice.
  auto test_plant = std::make_unique<TestSystemWithMoreOutputs>();
  TestSystemWithMoreOutputs* plant_ptr = test_plant.get();

  const int plant_output_size = test_plant->get_output_port(0).get_size();
  const int controller_feedback_size
      = test_plant->get_input_port(0).get_size() * 2;

  // Selects the first two signals from the plant's output port zero as the
  // feedback for the controller.
  MatrixX<double> feedback_selector_matrix(controller_feedback_size,
                                           plant_output_size);
  EXPECT_EQ(feedback_selector_matrix.rows(), 2);
  EXPECT_EQ(feedback_selector_matrix.cols(), 6);
  feedback_selector_matrix
      << 1, 0, 0, 0, 0, 0,
         0, 1, 0, 0, 0, 0;

  auto feedback_selector =
      std::make_unique<MimoGain<double>>(feedback_selector_matrix);
  const double Kp = 2.;
  const double Kd = .1;
  auto controller = builder.AddSystem<PidControlledSystem>(
      std::move(test_plant), std::move(feedback_selector), Kp, 0., Kd);

  builder.Connect(input_source->get_output_port(),
                  controller->get_input_port(0));
  builder.Connect(state_source->get_output_port(),
                  controller->get_input_port(1));
  builder.ExportOutput(controller->get_output_port(0));
  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();
  auto output = diagram->AllocateOutput(*context);

  systems::Context<double>* controller_context =
      diagram->GetMutableSubsystemContext(context.get(), controller);
  controller->SetDefaultState(controller_context);
  systems::Context<double>* test_context =
      controller->GetMutableSubsystemContext(
          controller_context, plant_ptr);

  diagram->EvalOutput(*context, output.get());
  const BasicVector<double>* output_vec = output->get_vector_data(0);
  const double pid_input = plant_ptr->GetInputValue(*test_context);
  EXPECT_EQ(pid_input, input[0] + (state[0] - output_vec->get_value()[0]) * Kp +
            (state[1] - output_vec->get_value()[1]) * Kd);
}

}  // namespace
}  // namespace systems
}  // namespace drake

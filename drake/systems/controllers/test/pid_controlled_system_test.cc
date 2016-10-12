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

GTEST_TEST(PidControlledSystemTest, SimplePidControlledSystem) {
  DiagramBuilder<double> builder;
  const Vector1d input(1.);
  const Eigen::Vector2d state(1.1, 0.2);
  auto input_source = builder.AddSystem<ConstantVectorSource>(input);
  auto state_source = builder.AddSystem<ConstantVectorSource>(state);

  // Our test "system" is just a multiplexer which takes the input and
  // outputs it twice.
  auto test_sys = std::make_unique<TestSystem>();
  TestSystem* test_p = test_sys.get();
  const double Kp = 2.;
  const double Kd = .1;
  auto controller = builder.AddSystem<PidControlledSystem>(
      std::move(test_sys), Kp, 0., Kd);
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
          controller_context, test_p);

  diagram->EvalOutput(*context, output.get());
  const BasicVector<double>* output_vec = output->get_vector_data(0);
  const double pid_input = test_p->GetInputValue(*test_context);
  EXPECT_EQ(pid_input, input[0] + (state[0] - output_vec->get_value()[0]) * Kp +
            (state[1] - output_vec->get_value()[1]) * Kd);
}

}  // namespace
}  // namespace systems
}  // namespace drake

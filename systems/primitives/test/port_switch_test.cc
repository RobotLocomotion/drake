#include "drake/systems/primitives/port_switch.h"

#include <memory>
#include <utility>

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/fixed_input_port_value.h"
#include "drake/systems/framework/test_utilities/scalar_conversion.h"

using Eigen::VectorXd;
using std::make_unique;

namespace drake {
namespace systems {
namespace {

// Tests that the output of this system equals its input.
GTEST_TEST(PortSwitchTest, VectorTest) {
  PortSwitch<double> system(2);

  EXPECT_EQ(system.num_input_ports(), 1);
  EXPECT_EQ(system.num_output_ports(), 1);
  EXPECT_EQ(system.get_output_port().size(), 2);

  const InputPort<double>& port_A = system.DeclareInputPort("A");
  const InputPort<double>& port_B = system.DeclareInputPort("B");
  EXPECT_EQ(system.num_input_ports(), 3);
  EXPECT_EQ(port_A.get_data_type(), kVectorValued);
  EXPECT_EQ(port_B.get_data_type(), kVectorValued);
  EXPECT_EQ(port_A.size(), 2);
  EXPECT_EQ(port_B.size(), 2);
  EXPECT_EQ(port_A.get_name(), "A");
  EXPECT_EQ(port_B.get_name(), "B");

  auto context = system.CreateDefaultContext();

  // Should be stateless.
  EXPECT_EQ(context->num_total_states(), 0);

  const Eigen::Vector2d a(1., 2.);
  const Eigen::Vector2d b(3., 4.);
  port_A.FixValue(context.get(), a);
  port_B.FixValue(context.get(), b);

  system.get_port_selector_input_port().FixValue(context.get(),
                                                 port_A.get_index());
  EXPECT_TRUE(CompareMatrices(system.get_output_port().Eval(*context), a));

  system.get_port_selector_input_port().FixValue(context.get(),
                                                 port_B.get_index());
  EXPECT_TRUE(CompareMatrices(system.get_output_port().Eval(*context), b));
}

GTEST_TEST(PortSwitchTest, AbstactValueTest) {
  PortSwitch<double> system(std::string("test"));

  EXPECT_EQ(system.num_input_ports(), 1);
  EXPECT_EQ(system.num_output_ports(), 1);
  EXPECT_EQ(system.get_output_port().size(), 0);

  const InputPort<double>& port_A = system.DeclareInputPort("A");
  const InputPort<double>& port_B = system.DeclareInputPort("B");
  EXPECT_EQ(system.num_input_ports(), 3);
  EXPECT_EQ(port_A.get_data_type(), kAbstractValued);
  EXPECT_EQ(port_B.get_data_type(), kAbstractValued);
  EXPECT_EQ(port_A.size(), 0);
  EXPECT_EQ(port_B.size(), 0);
  EXPECT_EQ(port_A.get_name(), "A");
  EXPECT_EQ(port_B.get_name(), "B");

  auto context = system.CreateDefaultContext();

  // Should be stateless.
  EXPECT_EQ(context->num_total_states(), 0);

  port_A.FixValue(context.get(), "got A");
  port_B.FixValue(context.get(), "got B");

  system.get_port_selector_input_port().FixValue(context.get(),
                                                 port_A.get_index());
  EXPECT_EQ(system.get_output_port().Eval<std::string>(*context), "got A");

  system.get_port_selector_input_port().FixValue(context.get(),
                                                 port_B.get_index());
  EXPECT_EQ(system.get_output_port().Eval<std::string>(*context), "got B");
}

// Test that declared input ports (name + type/size) survive scalar conversion.
GTEST_TEST(PortSwitchTest, VectorScalarConversion) {
  PortSwitch<double> system(4);
  system.DeclareInputPort("A");
  system.DeclareInputPort("B");

  auto autodiff_system = system.ToAutoDiffXd();
  EXPECT_EQ(autodiff_system->num_input_ports(), 3);
  EXPECT_EQ(autodiff_system->GetInputPort("A").size(), 4);
  EXPECT_EQ(autodiff_system->GetInputPort("B").size(), 4);

  auto symbolic_system = system.ToSymbolic();
  EXPECT_EQ(symbolic_system->num_input_ports(), 3);
  EXPECT_EQ(symbolic_system->GetInputPort("A").size(), 4);
  EXPECT_EQ(symbolic_system->GetInputPort("B").size(), 4);
}

// Test that declared input ports (name + type/size) survive scalar conversion.
GTEST_TEST(PortSwitchTest, AbstractValueScalarConversion) {
  PortSwitch<double> system(std::string("test"));
  system.DeclareInputPort("A");
  system.DeclareInputPort("B");

  auto autodiff_system = system.ToAutoDiffXd();
  EXPECT_EQ(autodiff_system->num_input_ports(), 3);
  EXPECT_EQ(autodiff_system->GetInputPort("A").size(), 0);
  EXPECT_EQ(autodiff_system->GetInputPort("B").size(), 0);

  auto symbolic_system = system.ToSymbolic();
  EXPECT_EQ(symbolic_system->num_input_ports(), 3);
  EXPECT_EQ(symbolic_system->GetInputPort("A").size(), 0);
  EXPECT_EQ(symbolic_system->GetInputPort("B").size(), 0);
}

// Trivial class for testing scalar conversion of templated types.
template <typename T>
struct MySignal {
  MySignal() : value(0.0) {}
  explicit MySignal(const double& value_in) : value(value_in) {}
  T value;
};

// Test that scalar-typed abstract values port types survive scalar
// conversion (the port value's scalar type should match the system's scalar
// type).
GTEST_TEST(PortSwitchTest, TemplatedAbstractValue) {
  const MySignal<double> my_signal(4.0);
  PortSwitch<double> system(my_signal);
  system.DeclareInputPort("A");
  system.DeclareInputPort("B");

  auto autodiff_system = system.ToAutoDiffXd();
  EXPECT_EQ(autodiff_system->num_input_ports(), 3);
  const InputPort<AutoDiffXd>& port_A = autodiff_system->GetInputPort("A");
  EXPECT_EQ(port_A.size(), 0);
  EXPECT_EQ(autodiff_system->GetInputPort("B").size(), 0);
  auto autodiff_context = autodiff_system->CreateDefaultContext();
  port_A.FixValue(autodiff_context.get(), MySignal<AutoDiffXd>(2.0));
  autodiff_system->GetInputPort("port_selector")
      .FixValue(autodiff_context.get(), port_A.get_index());
  const MySignal<AutoDiffXd>& autodiff_output =
      autodiff_system->get_output_port(0).template Eval<MySignal<AutoDiffXd>>(
          *autodiff_context);
  EXPECT_EQ(autodiff_output.value.value(), 2.0);

  auto symbolic_system = system.ToSymbolic();
  EXPECT_EQ(symbolic_system->num_input_ports(), 3);
  EXPECT_EQ(symbolic_system->GetInputPort("A").size(), 0);
  const InputPort<symbolic::Expression>& port_B =
      symbolic_system->GetInputPort("B");
  EXPECT_EQ(port_B.size(), 0);
  auto symbolic_context = symbolic_system->CreateDefaultContext();
  port_B.FixValue(symbolic_context.get(), MySignal<symbolic::Expression>(3.0));
  symbolic_system->GetInputPort("port_selector")
      .FixValue(symbolic_context.get(), port_B.get_index());
  const MySignal<symbolic::Expression>& symbolic_output =
      symbolic_system->get_output_port(0)
          .template Eval<MySignal<symbolic::Expression>>(*symbolic_context);
  EXPECT_EQ(symbolic_output.value.Evaluate(), 3.0);
}

}  // namespace
}  // namespace systems
}  // namespace drake

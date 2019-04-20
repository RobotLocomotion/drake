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

  // Make sure it throws if I attempt to fix an AbstractValue.
  EXPECT_THROW(
      context->FixInputPort(port_A.get_index(), Value<std::string>("got A")),
      std::logic_error);
}

GTEST_TEST(PortSwitchTest, AbstactValueTest) {
  PortSwitch<double> system(std::string("test"));

  EXPECT_EQ(system.num_input_ports(), 1);
  EXPECT_EQ(system.num_output_ports(), 1);
  EXPECT_EQ(system.get_output_port().size(), 0);

  const InputPort<double>& port_A = system.DeclareInputPort("A");
  const InputPort<double>& port_B = system.DeclareInputPort("B");
  EXPECT_EQ(system.num_input_ports(), 3);
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

  // Make sure it throws if I attempt to fix a vector value.
  EXPECT_THROW(
      context->FixInputPort(port_A.get_index(), Eigen::Vector2d(1., 2.)),
      std::logic_error);
}

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

}  // namespace
}  // namespace systems
}  // namespace drake

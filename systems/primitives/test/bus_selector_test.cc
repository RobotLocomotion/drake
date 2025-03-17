#include "drake/systems/primitives/bus_selector.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/systems/framework/test_utilities/scalar_conversion.h"

namespace drake {
namespace systems {
namespace {

GTEST_TEST(BusSelectorTest, EmptyDefault) {
  const BusSelector<double> dut;
  EXPECT_EQ(dut.num_input_ports(), 1);
  EXPECT_EQ(dut.get_input_port().get_name(), "u0");
  EXPECT_EQ(dut.num_output_ports(), 0);
}

GTEST_TEST(BusSelectorTest, EmptyWithInputName) {
  const BusSelector<double> dut("bus");
  EXPECT_EQ(dut.num_input_ports(), 1);
  EXPECT_EQ(dut.get_input_port().get_name(), "bus");
  EXPECT_EQ(dut.num_output_ports(), 0);
}

GTEST_TEST(BusSelectorTest, VariousPorts) {
  BusSelector<double> dut("bus");
  dut.DeclareVectorOutputPort(kUseDefaultName, 2);
  dut.DeclareVectorOutputPort("out1", 3);
  dut.DeclareAbstractOutputPort(kUseDefaultName, Value<int>());
  dut.DeclareAbstractOutputPort("out3", Value<double>());

  EXPECT_EQ(dut.num_output_ports(), 4);
  EXPECT_EQ(dut.get_output_port(0).get_name(), "y0");
  EXPECT_EQ(dut.get_output_port(1).get_name(), "out1");
  EXPECT_EQ(dut.get_output_port(2).get_name(), "y2");
  EXPECT_EQ(dut.get_output_port(3).get_name(), "out3");

  // When input port(s) are missing, evaluating the output will throw.
  auto context = dut.CreateDefaultContext();
  BusValue bus;
  auto& fixed_input = dut.get_input_port().FixValue(context.get(), bus);
  DRAKE_EXPECT_THROWS_MESSAGE(dut.get_output_port(0).Eval(*context),
                              ".*Missing.*input.*y0.*");
  DRAKE_EXPECT_THROWS_MESSAGE(dut.get_output_port(2).Eval(*context),
                              ".*Missing.*input.*y2.*");

  // When the value for a vector output port is wrongly typed, we throw.
  bus.Set("y0", Value{0.5});
  fixed_input.GetMutableData()->set_value(bus);
  DRAKE_EXPECT_THROWS_MESSAGE(dut.get_output_port(0).Eval(*context),
                              ".*type.*non-vector.*y0.*");
  bus = BusValue{};

  // Prepare a context with the inputs set to a bus value with all data.
  // Check that it all makes it through to the output.
  const BasicVector<double> y0{12.0, 11.0};
  const BasicVector<double> out1{3.0, 2.0, 1.0};
  const int y2{11};
  const double out3{22.2};
  bus.Set("y0", Value{y0});
  bus.Set("out1", Value{out1});
  bus.Set("y2", Value{y2});
  bus.Set("out3", Value{out3});
  fixed_input.GetMutableData()->set_value(bus);
  EXPECT_EQ(dut.get_output_port(0).Eval(*context), y0.get_value());
  EXPECT_EQ(dut.get_output_port(1).Eval(*context), out1.get_value());
  EXPECT_EQ(dut.get_output_port(2).template Eval<int>(*context), y2);
  EXPECT_EQ(dut.get_output_port(3).template Eval<double>(*context), out3);

  // Change the input and make sure the output updates.
  const BasicVector<double> new_y0{1.0, 2.0};
  const int new_y2{33};
  bus.Set("y0", Value{new_y0});
  bus.Set("y2", Value{new_y2});
  fixed_input.GetMutableData()->set_value(bus);
  EXPECT_EQ(dut.get_output_port(0).Eval(*context), new_y0.get_value());
  EXPECT_EQ(dut.get_output_port(1).Eval(*context), out1.get_value());
  EXPECT_EQ(dut.get_output_port(2).template Eval<int>(*context), new_y2);
  EXPECT_EQ(dut.get_output_port(3).template Eval<double>(*context), out3);
}

GTEST_TEST(BusSelectorTest, ConvertScalarType) {
  BusSelector<double> dut("bus");
  dut.DeclareVectorOutputPort(kUseDefaultName, 2);
  dut.DeclareVectorOutputPort("out1", 3);
  dut.DeclareAbstractOutputPort(kUseDefaultName, Value<int>());
  dut.DeclareAbstractOutputPort("out3", Value<double>());
  EXPECT_TRUE(is_autodiffxd_convertible(dut, [&](const auto& converted) {
    EXPECT_EQ(converted.num_input_ports(), 1);
    EXPECT_EQ(converted.get_input_port().get_name(), "bus");
    EXPECT_EQ(converted.num_output_ports(), 4);
    EXPECT_EQ(converted.get_output_port(0).get_name(), "y0");
    EXPECT_EQ(converted.get_output_port(1).get_name(), "out1");
    EXPECT_EQ(converted.get_output_port(2).get_name(), "y2");
    EXPECT_EQ(converted.get_output_port(3).get_name(), "out3");
  }));
  EXPECT_TRUE(is_symbolic_convertible(dut, [&](const auto& converted) {
    EXPECT_EQ(converted.num_input_ports(), 1);
    EXPECT_EQ(converted.get_input_port().get_name(), "bus");
    EXPECT_EQ(converted.num_output_ports(), 4);
    EXPECT_EQ(converted.get_output_port(0).get_name(), "y0");
    EXPECT_EQ(converted.get_output_port(1).get_name(), "out1");
    EXPECT_EQ(converted.get_output_port(2).get_name(), "y2");
    EXPECT_EQ(converted.get_output_port(3).get_name(), "out3");
  }));
}

}  // namespace
}  // namespace systems
}  // namespace drake

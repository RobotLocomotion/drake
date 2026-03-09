#include "drake/systems/primitives/bus_creator.h"

#include <map>
#include <string>
#include <utility>

#include <gtest/gtest.h>

#include "drake/systems/framework/test_utilities/scalar_conversion.h"

namespace drake {
namespace systems {
namespace {

GTEST_TEST(BusCreatorTest, EmptyDefault) {
  const BusCreator<double> dut;
  EXPECT_EQ(dut.num_input_ports(), 0);
  EXPECT_EQ(dut.num_output_ports(), 1);
  EXPECT_EQ(dut.get_output_port().get_name(), "y0");
  auto context = dut.CreateDefaultContext();
  const auto& bus_value =
      dut.get_output_port().template Eval<BusValue>(*context);
  EXPECT_TRUE(bus_value.begin() == bus_value.end());
}

GTEST_TEST(BusCreatorTest, EmptyWithOutputName) {
  const BusCreator<double> dut("bus");
  EXPECT_EQ(dut.num_input_ports(), 0);
  EXPECT_EQ(dut.num_output_ports(), 1);
  EXPECT_EQ(dut.get_output_port().get_name(), "bus");
}

GTEST_TEST(BusCreatorTest, VariousPorts) {
  BusCreator<double> dut("bus");
  dut.DeclareVectorInputPort(kUseDefaultName, 2);
  dut.DeclareVectorInputPort("in1", 3);
  dut.DeclareAbstractInputPort(kUseDefaultName, Value<int>());
  dut.DeclareAbstractInputPort("in3", Value<double>());

  EXPECT_EQ(dut.num_input_ports(), 4);
  EXPECT_EQ(dut.get_input_port(0).get_name(), "u0");
  EXPECT_EQ(dut.get_input_port(1).get_name(), "in1");
  EXPECT_EQ(dut.get_input_port(2).get_name(), "u2");
  EXPECT_EQ(dut.get_input_port(3).get_name(), "in3");

  // Create some sugar to dump the output port to a map<string, string>.
  auto context = dut.CreateDefaultContext();
  const auto& output = dut.get_output_port();
  using Map = std::map<std::string, std::string>;
  auto eval_output = [&output, &context]() -> Map {
    Map result;
    const auto& bus_value = output.template Eval<BusValue>(*context);
    for (const auto&& [name, value] : bus_value) {
      std::string value_str;
      if ((name == "u0") || (name == "in1")) {
        const auto& datum = value.template get_value<BasicVector<double>>();
        value_str = fmt::to_string(fmt_eigen(datum.get_value()));
      } else if (name == "u2") {
        const auto& datum = value.template get_value<int>();
        value_str = fmt::to_string(datum);
      } else if (name == "in3") {
        const auto& datum = value.template get_value<double>();
        value_str = fmt::to_string(datum);
      } else {
        value_str = "error: unknown port name cannot be downcast";
      }
      result.emplace(name, std::move(value_str));
    }
    return result;
  };

  // The bus starts out empty.
  Map expected;
  EXPECT_EQ(eval_output(), expected);

  // Add inputs one by one and make sure they come through.
  const BasicVector<double> u0{12.0, 11.0};
  const BasicVector<double> in1{3.0, 2.0, 1.0};
  const int u2{11};
  const double in3{22.2};
  dut.get_input_port(3).FixValue(context.get(), Value{in3});
  expected.emplace("in3", "22.2");
  EXPECT_EQ(eval_output(), expected);
  dut.get_input_port(2).FixValue(context.get(), Value{u2});
  expected.emplace("u2", "11");
  EXPECT_EQ(eval_output(), expected);
  dut.get_input_port(1).FixValue(context.get(), Value{in1});
  expected.emplace("in1", "3\n2\n1");
  EXPECT_EQ(eval_output(), expected);
  dut.get_input_port(0).FixValue(context.get(), Value{u0});
  expected.emplace("u0", "12\n11");
  EXPECT_EQ(eval_output(), expected);

  // Change inputs and make sure the changes come through.
  dut.get_input_port(0).FixValue(context.get(),
                                 Value{BasicVector<double>{1.0, 2.0}});
  expected["u0"] = "1\n2";
  dut.get_input_port(2).FixValue(context.get(), Value{33});
  expected["u2"] = "33";
  EXPECT_EQ(eval_output(), expected);
}

GTEST_TEST(BusCreatorTest, ConvertScalarType) {
  BusCreator<double> dut("bus");
  dut.DeclareVectorInputPort(kUseDefaultName, 2);
  dut.DeclareVectorInputPort("in1", 3);
  dut.DeclareAbstractInputPort(kUseDefaultName, Value<int>());
  dut.DeclareAbstractInputPort("in3", Value<double>());
  EXPECT_TRUE(is_autodiffxd_convertible(dut, [&](const auto& converted) {
    EXPECT_EQ(converted.num_input_ports(), 4);
    EXPECT_EQ(converted.get_input_port(0).get_name(), "u0");
    EXPECT_EQ(converted.get_input_port(1).get_name(), "in1");
    EXPECT_EQ(converted.get_input_port(2).get_name(), "u2");
    EXPECT_EQ(converted.get_input_port(3).get_name(), "in3");
    EXPECT_EQ(converted.num_output_ports(), 1);
    EXPECT_EQ(converted.get_output_port().get_name(), "bus");
  }));
  EXPECT_TRUE(is_symbolic_convertible(dut, [&](const auto& converted) {
    EXPECT_EQ(converted.num_input_ports(), 4);
    EXPECT_EQ(converted.get_input_port(0).get_name(), "u0");
    EXPECT_EQ(converted.get_input_port(1).get_name(), "in1");
    EXPECT_EQ(converted.get_input_port(2).get_name(), "u2");
    EXPECT_EQ(converted.get_input_port(3).get_name(), "in3");
    EXPECT_EQ(converted.num_output_ports(), 1);
    EXPECT_EQ(converted.get_output_port().get_name(), "bus");
  }));
}

}  // namespace
}  // namespace systems
}  // namespace drake

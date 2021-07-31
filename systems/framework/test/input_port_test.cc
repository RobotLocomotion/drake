#include "drake/systems/framework/input_port.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/test_utilities/my_vector.h"

namespace drake {
namespace systems {
namespace {

class DummySystem final : public LeafSystem<double> {
 public:
  using SystemBase::get_system_id;
};

// The mocked-up return value for our DoEval stub, below.
const AbstractValue* g_do_eval_result = nullptr;

// Returns g_do_eval_result.
const AbstractValue* DoEval(const ContextBase&) {
  return g_do_eval_result;
}

GTEST_TEST(InputPortTest, VectorTest) {
  using T = double;

  DummySystem dummy_system;
  dummy_system.set_name("dummy");
  const auto context_ptr = dummy_system.CreateDefaultContext();
  const auto& context = *context_ptr;
  const System<T>* const system = &dummy_system;
  internal::SystemMessageInterface* const system_interface = &dummy_system;
  const std::string name{"port_name"};
  const InputPortIndex index{2};
  const DependencyTicket ticket;
  const PortDataType data_type = kVectorValued;
  const int size = 3;
  const std::optional<RandomDistribution> random_type = std::nullopt;

  auto dut = internal::FrameworkFactory::Make<InputPort<T>>(
      system, system_interface, dummy_system.get_system_id(), name, index,
      ticket, data_type, size, random_type, &DoEval);

  // Check basic getters.
  EXPECT_EQ(dut->get_name(), name);
  EXPECT_EQ(dut->get_data_type(), data_type);
  EXPECT_EQ(dut->size(), size);
  EXPECT_EQ(dut->GetFullDescription(),
            "InputPort[2] (port_name) of System ::dummy (DummySystem)");
  EXPECT_EQ(&dut->get_system_interface(), system_interface);
  EXPECT_EQ(&dut->get_system(), system);

  // Check HasValue.
  g_do_eval_result = nullptr;
  EXPECT_EQ(dut->HasValue(context), false);
  const Vector3<T> data(1.0, 2.0, 3.0);
  const Value<BasicVector<T>> new_value{MyVector3d(data)};
  g_do_eval_result = &new_value;
  EXPECT_EQ(dut->HasValue(context), true);

  // Check Eval with various ValueType possibilities.
  const auto& eval_eigen = dut->Eval(context);
  const BasicVector<T>& eval_basic = dut->Eval<BasicVector<T>>(context);
  const MyVector3d& eval_myvec3 = dut->Eval<MyVector3d>(context);
  const AbstractValue& eval_abs = dut->Eval<AbstractValue>(context);
  EXPECT_EQ(eval_eigen, data);
  EXPECT_EQ(eval_basic.CopyToVector(), data);
  EXPECT_EQ(eval_myvec3.CopyToVector(), data);
  EXPECT_EQ(eval_abs.get_value<BasicVector<T>>().CopyToVector(), data);

  // Check error messages.
  DRAKE_EXPECT_THROWS_MESSAGE(
      dut->Eval<std::string>(context),
      "InputPort::Eval..: wrong value type std::string specified; "
      "actual type was drake::systems::MyVector<double,3> "
      "for InputPort.*2.*of.*dummy.*DummySystem.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      dut->Eval<MyVector2d>(context),
      "InputPort::Eval..: wrong value type .*MyVector<double,2> specified; "
      "actual type was .*MyVector<double,3> "
      "for InputPort.*2.*of.*dummy.*DummySystem.*");
}

GTEST_TEST(InputPortTest, AbstractTest) {
  using T = double;

  DummySystem dummy_system;
  dummy_system.set_name("dummy");
  const auto context_ptr = dummy_system.CreateDefaultContext();
  const auto& context = *context_ptr;
  const System<T>* const system = &dummy_system;
  internal::SystemMessageInterface* const system_interface = &dummy_system;
  const std::string name{"port_name"};
  const InputPortIndex index{2};
  const DependencyTicket ticket;
  const PortDataType data_type = kAbstractValued;
  const int size = 0;
  const std::optional<RandomDistribution> random_type = std::nullopt;

  auto dut = internal::FrameworkFactory::Make<InputPort<T>>(
      system, system_interface, dummy_system.get_system_id(), name, index,
      ticket, data_type, size, random_type, &DoEval);

  // Check basic getters.
  EXPECT_EQ(dut->get_name(), name);
  EXPECT_EQ(dut->get_data_type(), data_type);
  EXPECT_EQ(dut->size(), size);
  EXPECT_EQ(dut->GetFullDescription(),
            "InputPort[2] (port_name) of System ::dummy (DummySystem)");
  EXPECT_EQ(&dut->get_system_interface(), system_interface);
  EXPECT_EQ(&dut->get_system(), system);

  // Check HasValue.
  g_do_eval_result = nullptr;
  EXPECT_EQ(dut->HasValue(context), false);
  const std::string data{"foo"};
  const Value<std::string> new_value{data};
  g_do_eval_result = &new_value;
  EXPECT_EQ(dut->HasValue(context), true);

  // Check Eval with various ValueType possibilities.
  const std::string& eval_str = dut->Eval<std::string>(context);
  const AbstractValue& eval_abs = dut->Eval<AbstractValue>(context);
  EXPECT_EQ(eval_str, data);
  EXPECT_EQ(eval_abs.get_value<std::string>(), data);

  DRAKE_EXPECT_THROWS_MESSAGE(
      dut->Eval(context),
      "InputPort::Eval..: wrong value type .*BasicVector<double> specified; "
      "actual type was std::string "
      "for InputPort.*2.*of.*dummy.*DummySystem.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      dut->Eval<BasicVector<T>>(context),
      "InputPort::Eval..: wrong value type .*BasicVector<double> specified; "
      "actual type was std::string "
      "for InputPort.*2.*of.*dummy.*DummySystem.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      dut->Eval<MyVector3d>(context),
      "InputPort::Eval..: wrong value type .*BasicVector<double> specified; "
      "actual type was std::string "
      "for InputPort.*2.*of.*dummy.*DummySystem.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      dut->Eval<int>(context),
      "InputPort::Eval..: wrong value type int specified; "
      "actual type was std::string "
      "for InputPort.*2.*of.*dummy.*DummySystem.*");
}

// This struct is for testing the FixValue() variants.
struct SystemWithInputPorts final : public LeafSystem<double> {
 public:
  SystemWithInputPorts()
      : basic_vec_port{DeclareVectorInputPort("basic_vec_port", 3)},
        derived_vec_port{DeclareVectorInputPort(
            "derived_vec_port", MyVector3d(Eigen::Vector3d(1., 2., 3.)))},
        int_port{DeclareAbstractInputPort("int_port", Value<int>(5))},
        double_port{
            DeclareAbstractInputPort("double_port", Value<double>(1.25))},
        string_port{DeclareAbstractInputPort("string_port",
                                             Value<std::string>("hello"))} {}
  InputPort<double>& basic_vec_port;
  InputPort<double>& derived_vec_port;
  InputPort<double>& int_port;
  InputPort<double>& double_port;
  InputPort<double>& string_port;
};

// Test the FixValue() method. Note that the conversion of its value argument
// to an AbstractValue is handled by internal::ValueToAbstractValue which has
// its own unit tests. Here we need just check the input-port specific
// behavior for vector and abstract intput ports.
// Also for sanity, make sure the returned FixedInputPortValue object works,
// although its API is so awful no one should use it.
GTEST_TEST(InputPortTest, FixValueTests) {
  SystemWithInputPorts dut;
  std::unique_ptr<Context<double>> context = dut.CreateDefaultContext();

  // None of the ports should have a value initially.
  for (int i = 0; i < dut.num_input_ports(); ++i) {
    EXPECT_FALSE(dut.get_input_port(i).HasValue(*context));
  }

  // First pound on the vector ports.

  // An Eigen vector value should be stored as a BasicVector.
  const Eigen::Vector3d expected_vec(10., 20., 30.);
  dut.basic_vec_port.FixValue(&*context, expected_vec);
  EXPECT_EQ(dut.basic_vec_port.Eval(*context), expected_vec);
  EXPECT_EQ(
      dut.basic_vec_port.Eval<BasicVector<double>>(*context).CopyToVector(),
      expected_vec);

  DRAKE_EXPECT_THROWS_MESSAGE(
      dut.basic_vec_port.Eval<std::string>(*context), std::logic_error,
      ".*wrong value type.*std::string.*actual type.*BasicVector.*");

  // TODO(sherm1) It would be nice to make this work rather than throw.
  DRAKE_EXPECT_THROWS_MESSAGE(
      dut.basic_vec_port.Eval<Eigen::Vector3d>(*context), std::logic_error,
      ".*wrong value type.*Eigen.*actual type.*BasicVector.*");

  // Pass a more complicated Eigen object; should still work.
  const Eigen::Vector4d long_vec(.25, .5, .75, 1.);
  dut.basic_vec_port.FixValue(&*context, 2. * long_vec.tail(3));
  EXPECT_EQ(dut.basic_vec_port.Eval(*context),
            2. * Eigen::Vector3d(.5, .75, 1.));

  // Should return a runtime error if the size is wrong.
  DRAKE_EXPECT_THROWS_MESSAGE(
      dut.basic_vec_port.FixValue(&*context, long_vec.segment<2>(1)),
      std::logic_error, ".*expected.*size=3.*actual.*size=2.*");

  // A BasicVector-derived type should be acceptable to vector ports with
  // either a BasicVector model or the derived-type model (sizes must match).
  const MyVector3d my_vector(expected_vec);
  dut.basic_vec_port.FixValue(&*context, my_vector);
  dut.derived_vec_port.FixValue(&*context, my_vector);

  // Either way the concrete type should be preserved.
  EXPECT_EQ(dut.basic_vec_port.Eval<MyVector3d>(*context).CopyToVector(),
            expected_vec);
  EXPECT_EQ(dut.derived_vec_port.Eval<MyVector3d>(*context).CopyToVector(),
            expected_vec);

  // A plain BasicVector should work in the BasicVector-modeled port but
  // NOT in the MyVector3-modeled port.
  const BasicVector<double> basic_vector3({7., 8., 9.});
  dut.basic_vec_port.FixValue(&*context, basic_vector3);  // (2)

  // TODO(sherm1) This shouldn't work, but does. See issue #9669.
  dut.derived_vec_port.FixValue(&*context, basic_vector3);

  // This is the right type, wrong size for the vector ports.
  const MyVector2d my_vector2(Eigen::Vector2d{19., 20.});
  DRAKE_EXPECT_THROWS_MESSAGE(
      dut.basic_vec_port.FixValue(&*context, my_vector2), std::logic_error,
      ".*expected.*size=3.*actual.*size=2.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      dut.derived_vec_port.FixValue(&*context, my_vector2), std::logic_error,
      ".*expected.*size=3.*actual.*size=2.*");

  // Now try the abstract ports.

  dut.int_port.FixValue(&*context, 17);
  EXPECT_EQ(dut.int_port.Eval<int>(*context), 17);

  DRAKE_EXPECT_THROWS_MESSAGE(
      dut.int_port.FixValue(&*context, 1.25), std::logic_error,
      ".*expected value of type int.*actual type was double.*");

  dut.double_port.FixValue(&*context, 1.25);
  EXPECT_EQ(dut.double_port.Eval<double>(*context), 1.25);

  // Without an explicit template argument, FixValue() won't do numerical
  // conversions.
  DRAKE_EXPECT_THROWS_MESSAGE(
      dut.double_port.FixValue(&*context, 4), std::logic_error,
      ".*expected value of type double.*actual type was int.*");

  // But an explicit template argument can serve as a workaround.
  dut.double_port.FixValue<double>(&*context, 4);
  EXPECT_EQ(dut.double_port.Eval<double>(*context), 4.0);

  // Use the string port for a variety of tests:
  // - the port value can be set as a string or char* constant
  // - the generic AbstractValue API works
  // - we can use the returned FixedInputPortValue object to change the value

  // Check the basics.
  dut.string_port.FixValue(&*context, std::string("dummy"));
  EXPECT_EQ(dut.string_port.Eval<std::string>(*context), "dummy");

  // Test special case API for C string constant, treated as an std::string.
  dut.string_port.FixValue(&*context, "a c string");
  EXPECT_EQ(dut.string_port.Eval<std::string>(*context), "a c string");

  // Test that we can take an AbstractValue or Value<T> object as input.
  const Value<int> int_value(42);
  const AbstractValue& int_value_as_abstract = Value<int>(43);
  dut.int_port.FixValue(&*context, int_value);
  EXPECT_EQ(dut.int_port.Eval<int>(*context), 42);
  dut.int_port.FixValue(&*context, int_value_as_abstract);
  EXPECT_EQ(dut.int_port.Eval<int>(*context), 43);

  // We should only accept the right kind of abstract value.
  DRAKE_EXPECT_THROWS_MESSAGE(
      dut.string_port.FixValue(&*context, int_value), std::logic_error,
      ".*expected.*type std::string.*actual type was int.*");

  auto& fixed_value =
      dut.string_port.FixValue(&*context, Value<std::string>("abstract"));
  EXPECT_EQ(dut.string_port.Eval<std::string>(*context), "abstract");

  // FixedInputPortValue has a very clunky interface, but let's make sure we
  // at least got the right object.
  fixed_value.GetMutableData()->get_mutable_value<std::string>() =
      "replacement string";
  EXPECT_EQ(dut.string_port.Eval<std::string>(*context), "replacement string");

  // All of the ports should have values by now.
  for (int i = 0; i < dut.num_input_ports(); ++i) {
    EXPECT_TRUE(dut.get_input_port(i).HasValue(*context));
  }
}

// For a subsystem embedded in a diagram, test that we can query, fix, and
// evaluate that subsystem's input ports using only a Context for that
// subsystem (rather than the whole Diagram context).
GTEST_TEST(InputPortTest, ContextForEmbeddedSystem) {
  DiagramBuilder<double> builder;
  auto* system = builder.AddSystem<SystemWithInputPorts>();
  auto diagram = builder.Build();

  // Create a context just for the System.
  auto context = system->CreateDefaultContext();

  // Verify that we can tell that ports have no values using this Context.
  EXPECT_FALSE(system->basic_vec_port.HasValue(*context));
  EXPECT_FALSE(system->int_port.HasValue(*context));
  EXPECT_FALSE(system->double_port.HasValue(*context));
  EXPECT_FALSE(system->string_port.HasValue(*context));

  // Set the values.
  const int kIntValue = 42;
  const double kDoubleValue = 13.25;
  const std::string kStringValue("abstract");
  const Eigen::Vector3d kVectorValue(10., 20., 30.);

  // Now fix the values.
  system->basic_vec_port.FixValue(context.get(), kVectorValue);
  system->int_port.FixValue(context.get(), kIntValue);
  system->double_port.FixValue(context.get(), kDoubleValue);
  system->string_port.FixValue(context.get(), kStringValue);

  // Now verify the values.
  EXPECT_EQ(kVectorValue, system->basic_vec_port.Eval(*context));
  EXPECT_EQ(kIntValue, system->int_port.Eval<int>(*context));
  EXPECT_EQ(kDoubleValue, system->double_port.Eval<double>(*context));
  EXPECT_EQ(kStringValue, system->string_port.Eval<std::string>(*context));

  // When given an inapproprate context, we fail-fast.
  auto diagram_context = diagram->CreateDefaultContext();
  DRAKE_EXPECT_THROWS_MESSAGE(
      system->int_port.HasValue(*diagram_context),
      ".*Context.*was not created for this InputPort.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      system->int_port.Eval<int>(*diagram_context),
      ".*Context.*was not created for this InputPort.*");
}

}  // namespace
}  // namespace systems
}  // namespace drake


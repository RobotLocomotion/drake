#include "drake/systems/framework/input_port.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/test_utilities/my_vector.h"

namespace drake {
namespace systems {
namespace {

class DummySystem final : public LeafSystem<double> {};

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
  internal::SystemMessageInterface* const system_base = &dummy_system;
  const std::string name{"port_name"};
  const InputPortIndex index{2};
  const DependencyTicket ticket;
  const PortDataType data_type = kVectorValued;
  const int size = 3;
  const optional<RandomDistribution> random_type = nullopt;

  auto dut = internal::FrameworkFactory::Make<InputPort<T>>(
      system, system_base, name, index, ticket, data_type, size, random_type,
      &DoEval);

  // Check basic getters.
  EXPECT_EQ(dut->get_name(), name);
  EXPECT_EQ(dut->get_data_type(), data_type);
  EXPECT_EQ(dut->size(), size);
  EXPECT_EQ(dut->GetFullDescription(),
            "InputPort[2] (port_name) of System ::dummy (DummySystem)");
  EXPECT_EQ(&dut->get_system_base(), system_base);
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
  EXPECT_EQ(eval_abs.GetValueOrThrow<BasicVector<T>>().CopyToVector(), data);

  // Check error messages.
  DRAKE_EXPECT_THROWS_MESSAGE(
      dut->Eval<std::string>(context), std::exception,
      "InputPort::Eval..: wrong value type std::string specified; "
      "actual type was drake::systems::MyVector<double,3> "
      "for InputPort.*2.*of.*dummy.*DummySystem.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      dut->Eval<MyVector2d>(context), std::exception,
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
  internal::SystemMessageInterface* const system_base = &dummy_system;
  const std::string name{"port_name"};
  const InputPortIndex index{2};
  const DependencyTicket ticket;
  const PortDataType data_type = kAbstractValued;
  const int size = 0;
  const optional<RandomDistribution> random_type = nullopt;

  auto dut = internal::FrameworkFactory::Make<InputPort<T>>(
      system, system_base, name, index, ticket, data_type, size, random_type,
      &DoEval);

  // Check basic getters.
  EXPECT_EQ(dut->get_name(), name);
  EXPECT_EQ(dut->get_data_type(), data_type);
  EXPECT_EQ(dut->size(), size);
  EXPECT_EQ(dut->GetFullDescription(),
            "InputPort[2] (port_name) of System ::dummy (DummySystem)");
  EXPECT_EQ(&dut->get_system_base(), system_base);
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
  EXPECT_EQ(eval_abs.GetValueOrThrow<std::string>(), data);

  DRAKE_EXPECT_THROWS_MESSAGE(
      dut->Eval(context), std::exception,
      "InputPort::Eval..: wrong value type .*BasicVector<double> specified; "
      "actual type was std::string "
      "for InputPort.*2.*of.*dummy.*DummySystem.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      dut->Eval<BasicVector<T>>(context), std::exception,
      "InputPort::Eval..: wrong value type .*BasicVector<double> specified; "
      "actual type was std::string "
      "for InputPort.*2.*of.*dummy.*DummySystem.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      dut->Eval<MyVector3d>(context), std::exception,
      "InputPort::Eval..: wrong value type .*BasicVector<double> specified; "
      "actual type was std::string "
      "for InputPort.*2.*of.*dummy.*DummySystem.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      dut->Eval<int>(context), std::exception,
      "InputPort::Eval..: wrong value type int specified; "
      "actual type was std::string "
      "for InputPort.*2.*of.*dummy.*DummySystem.*");
}

// This struct is for testing the FixValue() variants.
struct SystemWithInputPorts final : public LeafSystem<double> {
 public:
  SystemWithInputPorts()
      : basic_vec_port{DeclareVectorInputPort("basic_vec_port",
                                              BasicVector<double>(3))},
        derived_vec_port{DeclareVectorInputPort(
            "derived_vec_port",
            MyVector3d(Eigen::Vector3d(1., 2., 3.)))},
        int_port{DeclareAbstractInputPort("int_port", Value<int>(5))},
        double_port{
            DeclareAbstractInputPort("double_port", Value<double>(1.25))},
        string_port{DeclareAbstractInputPort("string_port",
                                             Value<std::string>("hello"))} {}
  const InputPort<double>& basic_vec_port;
  const InputPort<double>& derived_vec_port;
  const InputPort<double>& int_port;
  const InputPort<double>& double_port;
  const InputPort<double>& string_port;
};

// Test each of the five FixValue() methods. For reference:
// (1) FixValue(Eigen::Ref) should convert to BasicVector
// (2) FixValue(BasicVector) special-cased to make concrete vector types work
// (3) FixValue(AbstractValue) must match the port type
// (4) FixValue(const char*) special-cased shorthand for std::string
// (5) FixValue<V>(V) creates Value<V> for an abstract port
//
// Also for sanity, make sure the returned FixedInputPortValue object works,
// although its API is so awful no one should use it.
GTEST_TEST(InputPortTest, FixValueTests) {
  SystemWithInputPorts dut;
  std::unique_ptr<Context<double>> context = dut.CreateDefaultContext();

  // None of the ports should have a value initially.
  for (int i = 0; i < dut.get_num_input_ports(); ++i) {
    EXPECT_FALSE(dut.get_input_port(i).HasValue(*context));
  }

  // First check that the API that accepts an Eigen::Ref works. This is
  // supposed to copy the Eigen object to a BasicVector *not* create
  // a generic abstract object.
  const Eigen::Vector3d expected_vec(10., 20., 30.);
  dut.basic_vec_port.FixValue(&*context, expected_vec);  // (1)
  EXPECT_EQ(dut.basic_vec_port.Eval(*context), expected_vec);
  EXPECT_EQ(
      dut.basic_vec_port.Eval<BasicVector<double>>(*context).CopyToVector(),
      expected_vec);
  DRAKE_EXPECT_THROWS_MESSAGE(
      dut.basic_vec_port.Eval<Eigen::Vector3d>(*context), std::logic_error,
      ".*wrong value type.*Eigen.*actual type.*BasicVector.*");

  // Pass a more complicated Eigen object; should still work.
  const Eigen::Vector4d long_vec(.25, .5, .75, 1.);
  dut.basic_vec_port.FixValue(&*context, long_vec.tail(3));
  EXPECT_EQ(dut.basic_vec_port.Eval(*context), Eigen::Vector3d(.5, .75, 1.));

  // Should return a runtime error if the size is wrong.
  DRAKE_EXPECT_THROWS_MESSAGE(
      dut.basic_vec_port.FixValue(&*context, long_vec.segment<2>(1)),
      std::logic_error, ".*expected.*size=3.*actual.*size=2.*");

  const MyVector3d my_vector(expected_vec);
  dut.basic_vec_port.FixValue(&*context, my_vector);  // (2)
  dut.derived_vec_port.FixValue(&*context, my_vector);

  EXPECT_EQ(dut.basic_vec_port.Eval<MyVector3d>(*context).CopyToVector(),
            expected_vec);
  EXPECT_EQ(dut.derived_vec_port.Eval<MyVector3d>(*context).CopyToVector(),
            expected_vec);

  const BasicVector<double> basic_vector3({7., 8., 9.});
  dut.basic_vec_port.FixValue(&*context, basic_vector3);  // (2)
  dut.derived_vec_port.FixValue(&*context, basic_vector3);

  // This is the right type, wrong size for the vector ports.
  const BasicVector<double> basic_vector2({19., 20.});
  DRAKE_EXPECT_THROWS_MESSAGE(
      dut.basic_vec_port.FixValue(&*context, basic_vector2), std::logic_error,
      ".*expected.*size=3.*actual.*size=2.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      dut.derived_vec_port.FixValue(&*context, basic_vector2), std::logic_error,
      ".*expected.*size=3.*actual.*size=2.*");

  dut.int_port.FixValue(&*context, 17);  // (5)
  EXPECT_EQ(dut.int_port.Eval<int>(*context), 17);

  DRAKE_EXPECT_THROWS_MESSAGE(
      dut.int_port.FixValue(&*context, 1.25), std::logic_error,
      ".*expected value of type int.*actual type was double.*");

  dut.double_port.FixValue(&*context, 1.25);  // (5)
  EXPECT_EQ(dut.double_port.Eval<double>(*context), 1.25);

  // Without an explicit template argument, API (5) won't do numerical
  // conversions.
  DRAKE_EXPECT_THROWS_MESSAGE(
      dut.double_port.FixValue(&*context, 4),  // (5)
      std::logic_error,
      ".*expected value of type double.*actual type was int.*");

  // But an explicit template argument can serve as a workaround.
  dut.double_port.FixValue<double>(&*context, 4);  // (5)
  EXPECT_EQ(dut.double_port.Eval<double>(*context), 4.0);

  // Use the string port for a variety of tests:
  // - the port value can be set as a string or char* constant
  // - the generic AbstractValue API works
  // - we can use the returned FixedInputPortValue object to change the value

  // Check the basics.
  dut.string_port.FixValue(&*context, std::string("dummy"));  // (5)
  EXPECT_EQ(dut.string_port.Eval<std::string>(*context), "dummy");

  // Test special case API for C string constant, treated as an std::string.
  dut.string_port.FixValue(&*context, "a c string");  // (4)
  EXPECT_EQ(dut.string_port.Eval<std::string>(*context), "a c string");

  // Test API (3) that takes an AbstractValue directly.

  // We should only accept the right kind of abstract value.
  DRAKE_EXPECT_THROWS_MESSAGE(
      dut.string_port.FixValue(&*context,
                               *AbstractValue::Make<int>(42)),  // (3)
      std::logic_error, ".*expected.*type std::string.*actual type was int.*");

  auto& fixed_value = dut.string_port.FixValue(
      &*context, *AbstractValue::Make<std::string>("abstract"));  // (3)
  EXPECT_EQ(dut.string_port.Eval<std::string>(*context), "abstract");

  // FixedInputPortValue has a very clunky interface, but let's make sure we
  // at least got the right one.
  fixed_value.GetMutableData()->GetMutableValue<std::string>() =
      "replacement string";
  EXPECT_EQ(dut.string_port.Eval<std::string>(*context), "replacement string");

  // All of the ports should have values by now.
  for (int i = 0; i < dut.get_num_input_ports(); ++i) {
    EXPECT_TRUE(dut.get_input_port(i).HasValue(*context));
  }
}

}  // namespace
}  // namespace systems
}  // namespace drake


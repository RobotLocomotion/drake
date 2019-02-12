#include "drake/systems/framework/input_port.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
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
      "actual type was drake::systems::MyVector<3,double> "
      "for InputPort.*2.*of.*dummy.*DummySystem.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      dut->Eval<MyVector2d>(context), std::exception,
      "InputPort::Eval..: wrong value type .*MyVector<2,double> specified; "
      "actual type was .*MyVector<3,double> "
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

}  // namespace
}  // namespace systems
}  // namespace drake


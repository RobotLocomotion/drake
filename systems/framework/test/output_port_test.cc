#include "drake/common/test_utilities/expect_no_throw.h"
/* clang-format off to disable clang-format-includes */
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/leaf_output_port.h"
#include "drake/systems/framework/output_port.h"
/* clang-format on */

#include <memory>
#include <stdexcept>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/common/never_destroyed.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/system.h"
#include "drake/systems/framework/system_output.h"
#include "drake/systems/framework/test_utilities/my_vector.h"

namespace drake {
namespace systems {
namespace {

using Eigen::Vector2d;
using Eigen::Vector3d;
using std::string;
using std::unique_ptr;
using std::make_unique;

// A hollow shell of a System.
class DummySystem : public LeafSystem<double> {
 public:
  DummySystem() {}
  ~DummySystem() override {}
  using SystemBase::DeclareCacheEntry;
  using SystemBase::assign_next_dependency_ticket;
  using SystemBase::get_system_id;
};

// The only concrete output ports we expect to encounter are LeafOutputPort
// and DiagramOutputPort. Those are well behaved so don't trigger some of the
// base class error messages. Hence this feeble port. Derived classes will
// introduce errors.
class MyOutputPort : public OutputPort<double> {
 public:
  MyOutputPort(DummySystem* dummy, OutputPortIndex index,
               DependencyTicket ticket)
      : OutputPort<double>(dummy, dummy, dummy->get_system_id(),
                           "my_output", index, ticket, kVectorValued, 2) {}

  std::unique_ptr<AbstractValue> DoAllocate() const override {
    return AbstractValue::Make<BasicVector<double>>(
        MyVector2d(Vector2d(1., 2.)));
  }

  void DoCalc(const Context<double>&, AbstractValue* value) const override {
    EXPECT_NE(value, nullptr);
    DRAKE_EXPECT_NO_THROW(
        value->set_value<BasicVector<double>>(MyVector2d(Vector2d(3., 4.))));
  }

  const AbstractValue& DoEval(const Context<double>& context) const override {
    // This is a fake cache entry for Eval to "update".
    static never_destroyed<std::unique_ptr<AbstractValue>> temp(Allocate());
    Calc(context, temp.access().get());
    return *temp.access();
  }

  internal::OutputPortPrerequisite DoGetPrerequisite() const override {
    ADD_FAILURE() << "We won't call this.";
    return {};
  }

  void ThrowIfInvalidPortValueType(
      const Context<double>&,
      const AbstractValue& proposed_value) const final {
    // Note: this is a very expensive way to check -- fine for this test
    // case (which has no alternative) but don't copy into real code!
    auto good_value = Allocate();
    if (proposed_value.type_info() != good_value->type_info()) {
      throw std::logic_error(fmt::format(
          "OutputPort::Calc(): expected output type {} "
          "but got {} for {}.",
          good_value->GetNiceTypeName(), proposed_value.GetNiceTypeName(),
          PortBase::GetFullDescription()));
    }
  }
};

class MyStringAllocatorPort : public MyOutputPort {
 public:
  using MyOutputPort::MyOutputPort;

  // Allocator returns string but should have returned BasicVector.
  std::unique_ptr<AbstractValue> DoAllocate() const override {
    return AbstractValue::Make<std::string>("hello");
  }
};

class MyBadSizeAllocatorPort : public MyOutputPort {
 public:
  using MyOutputPort::MyOutputPort;

  // Allocator returns 3-element vector but should have returned 2-element.
  std::unique_ptr<AbstractValue> DoAllocate() const override {
    return AbstractValue::Make<BasicVector<double>>(
        MyVector3d(Vector3d(1., 2., 3.)));
  }
};

class MyNullAllocatorPort : public MyOutputPort {
 public:
  using MyOutputPort::MyOutputPort;

  // Allocator returns nullptr.
  std::unique_ptr<AbstractValue> DoAllocate() const override { return nullptr; }
};

// These "bad allocator" messages are likely to be caught earlier by
// LeafOutputPort when it delegates to cache entries. These base class messages
// may issue in case of (a) future addition of uncached concrete output ports,
// or (b) implementation changes to existing ports or error handling in caching.
GTEST_TEST(TestBaseClass, BadAllocators) {
  DummySystem dummy;
  MyStringAllocatorPort string_allocator{&dummy, OutputPortIndex(0),
                                         dummy.assign_next_dependency_ticket()};
  MyBadSizeAllocatorPort bad_size_allocator{
      &dummy, OutputPortIndex(1),
      dummy.assign_next_dependency_ticket()};
  MyNullAllocatorPort null_allocator{&dummy, OutputPortIndex(2),
                                     dummy.assign_next_dependency_ticket()};

  DRAKE_EXPECT_THROWS_MESSAGE_IF_ARMED(
      string_allocator.Allocate(), std::logic_error,
      "OutputPort::Allocate().*expected BasicVector.*but got.*std::string"
      ".*OutputPort\\[0\\].*");
  DRAKE_EXPECT_THROWS_MESSAGE_IF_ARMED(
      bad_size_allocator.Allocate(), std::logic_error,
      "OutputPort::Allocate().*expected vector.*size 2.*but got.*size 3"
      ".*OutputPort\\[1\\].*");

  // Nullptr check is unconditional.
  DRAKE_EXPECT_THROWS_MESSAGE(
      null_allocator.Allocate(), std::logic_error,
      "OutputPort::Allocate().*nullptr.*OutputPort\\[2\\].*");
}

// This message can be caused by user action.
GTEST_TEST(TestBaseClass, BadOutputType) {
  DummySystem dummy;
  MyOutputPort port{&dummy, OutputPortIndex(0),
                    dummy.assign_next_dependency_ticket()};
  auto context = dummy.AllocateContext();
  auto good_port_value = port.Allocate();
  auto bad_port_value = AbstractValue::Make<std::string>("hi there");

  DRAKE_EXPECT_NO_THROW(port.Calc(*context, good_port_value.get()));

  // This message is thrown in Debug. In Release some other error may trigger
  // but not from OutputPort, so we can't use
  // DRAKE_EXPECT_THROWS_MESSAGE_IF_ARMED() which would insist that if any
  // message is thrown in Release it must be the expected one.
  if (kDrakeAssertIsArmed) {
    DRAKE_EXPECT_THROWS_MESSAGE(
        port.Calc(*context, bad_port_value.get()), std::logic_error,
        "OutputPort::Calc().*expected.*MyVector.*but got.*std::string"
        ".*OutputPort\\[0\\].*");
  }
}

// These functions match the signatures required by LeafOutputPort.

// AllocCallback that returns a string in an AbstractValue.
unique_ptr<AbstractValue> alloc_string() {
  return AbstractValue::Make(string("from alloc_string"));
}

// AllocCallback that returns a MyVector3d(-1,-2,-3), wrapped in a
// Value<BasicVector>.
unique_ptr<AbstractValue> alloc_myvector3() {
  return Value<BasicVector<double>>(MyVector3d::Make(-1., -2., -3.)).Clone();
}

// CalcCallback that expects to have a string to write on.
void calc_string(const ContextBase&, AbstractValue* value) {
  ASSERT_NE(value, nullptr);
  string& str_value = value->get_mutable_value<string>();
  str_value = "from calc_string";
}

// CalcVectorCallback sets the 3-element output vector to 99,100,101.
void calc_vector3(const ContextBase&, AbstractValue* value) {
  ASSERT_NE(value, nullptr);
  auto& vec = value->template get_mutable_value<BasicVector<double>>();
  EXPECT_EQ(vec.size(), 3);
  vec.set_value(Vector3d(99., 100., 101.));
}

// This class creates some isolated ports we can play with. They are not
// actually part of a System. There are lots of tests of Systems that have
// output ports elsewhere; that's not what we're trying to test here.
class LeafOutputPortTest : public ::testing::Test {
 protected:
  // Create abstract- and vector-valued ports.
  DummySystem dummy_;
  // TODO(sherm1) Use implicit_cast when available (from abseil).
  std::unique_ptr<LeafOutputPort<double>> absport_general_ptr_ =
      internal::FrameworkFactory::Make<LeafOutputPort<double>>(
          &dummy_,  // implicit_cast<const System<T>*>(&dummy_)
          &dummy_,  // implicit_cast<SystemBase*>(&dummy_)
          dummy_.get_system_id(),
          "absport",
          OutputPortIndex(dummy_.num_output_ports()),
          dummy_.assign_next_dependency_ticket(), kAbstractValued, 0 /* size */,
          &dummy_.DeclareCacheEntry(
              "absport", ValueProducer(alloc_string, calc_string)));
  LeafOutputPort<double>& absport_general_ = *absport_general_ptr_;
  std::unique_ptr<LeafOutputPort<double>> vecport_general_ptr_ =
      internal::FrameworkFactory::Make<LeafOutputPort<double>>(
          &dummy_,  // implicit_cast<const System<T>*>(&dummy_)
          &dummy_,  // implicit_cast<SystemBase*>(&dummy_)
          dummy_.get_system_id(),
          "vecport",
          OutputPortIndex(dummy_.num_output_ports()),
          dummy_.assign_next_dependency_ticket(), kVectorValued, 3 /* size */,
          &dummy_.DeclareCacheEntry(
              "vecport", ValueProducer(alloc_myvector3, calc_vector3)));
  LeafOutputPort<double>& vecport_general_ = *vecport_general_ptr_;
  unique_ptr<Context<double>> context_{dummy_.CreateDefaultContext()};
};

// Helper function for testing an abstract-valued port.
void AbstractPortCheck(const Context<double>& context,
                       const LeafOutputPort<double>& port,
                       string alloc_string) {
  unique_ptr<AbstractValue> val = port.Allocate();
  EXPECT_EQ(val->get_value<string>(), alloc_string);
  port.Calc(context, val.get());
  const string new_value("from calc_string");
  EXPECT_EQ(val->get_value<string>(), new_value);
  EXPECT_EQ(port.Eval<string>(context), new_value);
  EXPECT_EQ(port.Eval<AbstractValue>(context).get_value<string>(),
            new_value);

  // Can't Eval into the wrong type.
  DRAKE_EXPECT_THROWS_MESSAGE(
      port.Eval<int>(context), std::logic_error,
      "OutputPort::Eval().*wrong value type int.*actual type.*std::string.*");
}

TEST_F(LeafOutputPortTest, DisableByDefaultWorks) {
  absport_general_.disable_caching_by_default();
  const CacheEntry& abs_entry = absport_general_.cache_entry();
  const CacheEntry& vec_entry = vecport_general_.cache_entry();
  EXPECT_TRUE(abs_entry.is_disabled_by_default());
  EXPECT_FALSE(vec_entry.is_disabled_by_default());

  // Find the cache entry values in a created context and verify that
  // the right one is disabled.
  auto my_context = dummy_.CreateDefaultContext();
  const CacheEntryValue& abs_value =
      my_context->get_cache().get_cache_entry_value(abs_entry.cache_index());
  const CacheEntryValue& vec_value =
      my_context->get_cache().get_cache_entry_value(vec_entry.cache_index());

  EXPECT_TRUE(abs_value.is_cache_entry_disabled());
  EXPECT_FALSE(vec_value.is_cache_entry_disabled());
}

// Check for proper construction and functioning of abstract LeafOutputPorts.
TEST_F(LeafOutputPortTest, AbstractPorts) {
  // Check abstract port with explicit function allocator.
  AbstractPortCheck(*context_, absport_general_, "from alloc_string");
}

// Helper function for testing a vector-valued port.
void VectorPortCheck(const Context<double>& context,
                     const LeafOutputPort<double>& port,
                     Vector3d alloc_value) {
  // Treat the vector-valued port as a BasicVector, which
  // should have MyVector3d as concrete type.
  unique_ptr<AbstractValue> val = port.Allocate();
  auto& basic = val->template get_mutable_value<BasicVector<double>>();
  MyVector3d& myvector3 = dynamic_cast<MyVector3d&>(basic);
  EXPECT_EQ(basic.get_value(), alloc_value);
  EXPECT_EQ(myvector3.get_value(), alloc_value);
  port.Calc(context, val.get());
  // Should have written into the underlying MyVector3d.
  const Vector3d new_value(99., 100., 101.);
  EXPECT_EQ(myvector3.get_value(), new_value);

  // Check that Eval is correct, for many ValueType choices.
  const auto& eval_eigen = port.Eval(context);
  const BasicVector<double>& eval_basic =
      port.Eval<BasicVector<double>>(context);
  const MyVector3d& eval_myvec3 = port.Eval<MyVector3d>(context);
  const AbstractValue& eval_abs = port.Eval<AbstractValue>(context);
  EXPECT_EQ(eval_eigen, new_value);
  EXPECT_EQ(eval_basic.CopyToVector(), new_value);
  EXPECT_EQ(eval_myvec3.CopyToVector(), new_value);
  EXPECT_EQ(
      eval_abs.get_value<BasicVector<double>>().CopyToVector(),
      new_value);
}

// Check for proper construction and functioning of vector-valued
// LeafOutputPorts.
TEST_F(LeafOutputPortTest, VectorPorts) {
  // Check vector port with explicit function allocator.
  VectorPortCheck(*context_, vecport_general_, Vector3d(-1., -2., -3.));
}

// AllocCallback that returns an illegal null value.
unique_ptr<AbstractValue> alloc_null() {
  return nullptr;
}

// The null check is done in all builds.
TEST_F(LeafOutputPortTest, ThrowIfNullAlloc) {
  // Create an abstract port with an allocator that returns null.
  // TODO(sherm1) Use implicit_cast when available (from abseil).
  auto null_port = internal::FrameworkFactory::Make<LeafOutputPort<double>>(
      &dummy_,  // implicit_cast<const System<T>*>(&dummy_)
      &dummy_,  // implicit_cast<SystemBase*>(&dummy_),
      dummy_.get_system_id(),
      "null_port",
      OutputPortIndex(dummy_.num_output_ports()),
      dummy_.assign_next_dependency_ticket(),
      kAbstractValued, 0 /* size */,
      &dummy_.DeclareCacheEntry(
          "null", ValueProducer(alloc_null, calc_string)));

  // Creating a context for this system should fail when it tries to allocate
  // a cache entry for null_port.
  EXPECT_THROW(dummy_.CreateDefaultContext(), std::logic_error);
}

// Check that Debug builds catch bad output types. We can't run these tests
// unchecked since the results would be indeterminate -- they may run to
// completion or segfault depending on memory contents.
TEST_F(LeafOutputPortTest, ThrowIfBadCalcOutput) {
  if (kDrakeAssertIsDisarmed) {
    return;
  }

  // The abstract port is a string; let's give it an int.
  auto good_out = absport_general_.Allocate();
  auto bad_out = AbstractValue::Make<int>(5);
  DRAKE_EXPECT_NO_THROW(absport_general_.Calc(*context_, good_out.get()));
  DRAKE_EXPECT_THROWS_MESSAGE(
      absport_general_.Calc(*context_, bad_out.get()), std::logic_error,
      "OutputPort::Calc().*expected.*std::string.*got.*int.*");

  // The vector port is a MyVector3d, we'll give it a BasicVector.
  auto good_vec = vecport_general_.Allocate();
  auto bad_vec = AbstractValue::Make(BasicVector<double>(2));
  DRAKE_EXPECT_NO_THROW(vecport_general_.Calc(*context_, good_vec.get()));
  DRAKE_EXPECT_THROWS_MESSAGE(
      vecport_general_.Calc(*context_, bad_vec.get()), std::logic_error,
      "OutputPort::Calc().*expected.*MyVector.*got.*BasicVector.*");
}

// For testing diagram output ports we need a couple of subsystems that have
// recognizably different Contexts so we can verify that (1) the diagram exports
// the correct ports, and (2) the diagram passes down the right subcontext. Here
// we just vary the number of continuous state variables which lets us check
// both things. We also need to test that nested diagrams manage to export
// already-exported ports correctly.
class SystemWithNStates : public LeafSystem<double> {
 public:
  explicit SystemWithNStates(int num_states) {
    DeclareContinuousState(num_states);
    DeclareAbstractOutputPort(
        kUseDefaultName, &SystemWithNStates::ReturnNumContinuous);
  }
  ~SystemWithNStates() override {}
 private:
  void ReturnNumContinuous(const Context<double>& context, int* nc) const {
    ASSERT_NE(nc, nullptr);
    *nc = context.num_continuous_states();
  }
};

// A one-level diagram.
class MyDiagram : public Diagram<double> {
 public:
  MyDiagram() {
    DiagramBuilder<double> builder;
    auto sys1 = builder.AddSystem<SystemWithNStates>(1);
    auto sys2 = builder.AddSystem<SystemWithNStates>(2);
    builder.ExportOutput(sys1->get_output_port(0));
    builder.ExportOutput(sys2->get_output_port(0));
    builder.BuildInto(this);
  }
};

// A two-level nested diagram.
class MyNestedDiagram : public Diagram<double> {
 public:
  MyNestedDiagram() {
    DiagramBuilder<double> builder;
    auto leaf = builder.AddSystem<SystemWithNStates>(3);
    auto diag = builder.AddSystem<MyDiagram>();

    // Order so that the nested ports have to change numbering.
    builder.ExportOutput(leaf->get_output_port(0));  // Should have 3 states.
    builder.ExportOutput(diag->get_output_port(0));  // 1 state.
    builder.ExportOutput(diag->get_output_port(1));  // 2 states.

    builder.BuildInto(this);
  }
};

GTEST_TEST(DiagramOutputPortTest, OneLevel) {
  MyDiagram diagram;
  auto context = diagram.CreateDefaultContext();
  auto& out0 = diagram.get_output_port(0);
  auto& out1 = diagram.get_output_port(1);
  auto value0 = out0.Allocate();  // unique_ptr<AbstractValue>
  auto value1 = out1.Allocate();
  const int* int0{};
  const int* int1{};
  DRAKE_EXPECT_NO_THROW(int0 = &value0->get_value<int>());
  DRAKE_EXPECT_NO_THROW(int1 = &value1->get_value<int>());
  EXPECT_EQ(*int0, 0);  // Default value initialized.
  EXPECT_EQ(*int1, 0);
  out0.Calc(*context, value0.get());
  out1.Calc(*context, value1.get());
  EXPECT_EQ(*int0, 1);  // Make sure we got the right Context.
  EXPECT_EQ(*int1, 2);

  // When given an inapproprate context, we fail-fast.
  const auto& diagram_context = dynamic_cast<DiagramContext<double>&>(*context);
  const auto& sys1_context = diagram_context.GetSubsystemContext(
      SubsystemIndex{0});
  DRAKE_EXPECT_THROWS_MESSAGE(
      out0.Eval<int>(sys1_context),
      ".*Context.*was not created for this OutputPort.*");
}

GTEST_TEST(DiagramOutputPortTest, Nested) {
  MyNestedDiagram diagram;
  auto context = diagram.CreateDefaultContext();
  auto& out0 = diagram.get_output_port(0);
  auto& out1 = diagram.get_output_port(1);
  auto& out2 = diagram.get_output_port(2);
  auto value0 = out0.Allocate();  // unique_ptr<AbstractValue>
  auto value1 = out1.Allocate();
  auto value2 = out2.Allocate();
  const int* int0{};
  const int* int1{};
  const int* int2{};
  DRAKE_EXPECT_NO_THROW(int0 = &value0->get_value<int>());
  DRAKE_EXPECT_NO_THROW(int1 = &value1->get_value<int>());
  DRAKE_EXPECT_NO_THROW(int2 = &value2->get_value<int>());
  EXPECT_EQ(*int0, 0);  // Default value initialized.
  EXPECT_EQ(*int1, 0);
  EXPECT_EQ(*int2, 0);
  out0.Calc(*context, value0.get());
  out1.Calc(*context, value1.get());
  out2.Calc(*context, value2.get());
  EXPECT_EQ(*int0, 3);  // Make sure we got the right Context.
  EXPECT_EQ(*int1, 1);
  EXPECT_EQ(*int2, 2);
}

}  // namespace
}  // namespace systems
}  // namespace drake

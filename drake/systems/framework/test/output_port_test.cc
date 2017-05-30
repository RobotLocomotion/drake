#include "drake/systems/framework/output_port.h"
#include "drake/systems/framework/leaf_output_port.h"

#include <cctype>
#include <memory>
#include <stdexcept>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_context.h"
#include "drake/systems/framework/output_port_value.h"
#include "drake/systems/framework/system.h"
#include "drake/systems/framework/test_utilities/my_vector.h"
#include "drake/systems/framework/value.h"
#include "drake/systems/primitives/constant_vector_source.h"

namespace drake {
namespace systems {
namespace {

using Eigen::Vector3d;
using std::string;
using std::unique_ptr;
using std::make_unique;

// A hollow shell of a System.
class DummySystem : public LeafSystem<double> {
 public:
  DummySystem() {}
  ~DummySystem() override {}
};

// These functions match the signatures required by LeafOutputPort.

// AllocCallback returns a string in an AbstractValue.
unique_ptr<AbstractValue> alloc_string(const Context<double>&) {
  return AbstractValue::Make(string("from alloc_string"));
}

// AllocVectorCallback returns a MyVector3d(-1,-2,-3).
unique_ptr<BasicVector<double>> alloc_myvector3(const Context<double>&) {
  return MyVector3d::Make(-1., -2., -3.);
}

// CalcCallback that expects to have a string to write on.
void calc_string(const Context<double>& context, AbstractValue* value) {
  ASSERT_NE(value, nullptr);
  string& str_value = value->GetMutableValueOrThrow<string>();
  str_value = "from calc_string";
}

// CalcVectorCallback sets the 3-element output vector to 99,100,101.
void calc_vector3(const Context<double>& context, BasicVector<double>* value) {
  ASSERT_NE(value, nullptr);
  EXPECT_EQ(value->size(), 3);
  value->set_value(Vector3d(99., 100., 101.));
}

// EvalCallback returning a string.
const AbstractValue& eval_string(const Context<double>& context) {
  static Value<string> fake_cache;
  fake_cache.set_value("from eval_string");
  return fake_cache;
}

// EvalCallback returning a MyVector3d(3,1,4).
const AbstractValue& eval_vector3(const Context<double>& context) {
  static Value<BasicVector<double>> fake_cache(*MyVector3d::Make(0., 0., 0.));
  fake_cache.get_mutable_value().set_value(Vector3d(3., 1., 4.));
  return fake_cache;
}

// This class creates some isolated ports we can play with. They are not
// actually part of a System. There are lots of tests of Systems that have
// output ports elsewhere; that's not what we're trying to test here.
class LeafOutputPortTest : public ::testing::Test {
 public:
  void SetUp() override {
    // TODO(sherm1) Eval methods should be generated automatically if not set.
    // This is just testing the basic wiring.
    absport_general_.set_evaluation_function(eval_string);
    vecport_general_.set_evaluation_function(eval_vector3);
  }

 protected:
  systems::ConstantVectorSource<double> source_{Vector3d(3., -3., 0.)};
  unique_ptr<Context<double>> context_{source_.CreateDefaultContext()};

  // Create abstract- and vector-valued ports.
  DummySystem dummy_;
  LeafOutputPort<double> absport_general_{dummy_, alloc_string, calc_string};
  LeafOutputPort<double> vecport_general_{dummy_, alloc_myvector3, 3,
                                          calc_vector3};
};

// Helper function for testing an abstract-valued port.
void AbstractPortCheck(const Context<double>& context,
                       const LeafOutputPort<double>& port,
                       string alloc_string) {
  unique_ptr<AbstractValue> val = port.Allocate(context);
  EXPECT_EQ(val->GetValueOrThrow<string>(), alloc_string);
  port.Calc(context, val.get());
  EXPECT_EQ(val->GetValueOrThrow<string>(), string("from calc_string"));
  const AbstractValue& val_cached = port.Eval(context);
  EXPECT_EQ(val_cached.GetValueOrThrow<string>(), string("from eval_string"));
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
  unique_ptr<AbstractValue> val = port.Allocate(context);
  auto& basic = val->template GetMutableValueOrThrow<BasicVector<double>>();
  MyVector3d& myvector3 = dynamic_cast<MyVector3d&>(basic);
  EXPECT_EQ(basic.get_value(), alloc_value);
  EXPECT_EQ(myvector3.get_value(), alloc_value);
  port.Calc(context, val.get());
  // Should have written into the underlying MyVector3d.
  EXPECT_EQ(myvector3.get_value(), Vector3d(99., 100., 101.));
}

// Check for proper construction and functioning of vector-valued
// LeafOutputPorts.
TEST_F(LeafOutputPortTest, VectorPorts) {
  // Check vector port with explicit function allocator.
  VectorPortCheck(*context_, vecport_general_, Vector3d(-1., -2., -3.));
}

// AllocCallback that returns an illegal null value.
unique_ptr<AbstractValue> alloc_null(const Context<double>&) {
  return nullptr;
}

// The null check is done in all builds.
TEST_F(LeafOutputPortTest, ThrowIfNullAlloc) {
  // Create an abstract port with an allocator that returns null.
  LeafOutputPort<double> null_port{dummy_, alloc_null, calc_string};
  EXPECT_THROW(null_port.Allocate(*context_), std::logic_error);
}

// Check that Debug builds catch bad output types. We can't
// run these tests unchecked since they may execute or segfault depending on
// memory contents.
#ifndef DRAKE_ASSERT_IS_DISARMED
TEST_F(LeafOutputPortTest, ThrowIfBadCalcOutput) {
  // The abstract port is a string; let's give it an int.
  auto good_out = absport_general_.Allocate(*context_);
  auto bad_out = AbstractValue::Make<int>(5);
  EXPECT_NO_THROW(absport_general_.Calc(*context_, good_out.get()));
  EXPECT_THROW(absport_general_.Calc(*context_, bad_out.get()),
               std::logic_error);

  // The vector port is a MyVector<3,double>, we'll give it a shorter one.
  auto good_vec = vecport_general_.Allocate(*context_);
  auto bad_vec = AbstractValue::Make<BasicVector<double>>(
      *MyVector<2, double>::Make(2., 3.));
  EXPECT_NO_THROW(vecport_general_.Calc(*context_, good_vec.get()));
  EXPECT_THROW(vecport_general_.Calc(*context_, bad_vec.get()),
               std::logic_error);
}
#endif

// For testing diagram output ports we need a couple of subsystems that have
// recognizably different Contexts so we can verify that the diagram output
// port passes down the right subcontext.
class SystemWithOneState : public LeafSystem<double> {
 public:
  SystemWithOneState() {
    DeclareContinuousState(1);
    DeclareAbstractOutputPort(alloc_string, calc_string);
  }
  ~SystemWithOneState() override {}
};

class SystemWithTwoStates : public LeafSystem<double> {
 public:
  SystemWithTwoStates() {
    DeclareContinuousState(2);
    DeclareVectorOutputPort(alloc_myvector3, 3, calc_vector3);
  }
  ~SystemWithTwoStates() override {}
};

GTEST_TEST(DiagramOutputPortTest, Basics) {
  SystemWithOneState sys1;
  SystemWithTwoStates sys2;
  auto context1 = sys1.CreateDefaultContext();
  auto context2 = sys2.CreateDefaultContext();

  // TODO(sherm1) MORE TESTS COMING.
}


}  // namespace
}  // namespace systems
}  // namespace drake

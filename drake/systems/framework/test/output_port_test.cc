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

// These functions match the signatures required by LeafOutputPort.

// AllocCallback returns a string in an AbstractValue.
unique_ptr<AbstractValue> alloc_string(const Context<double>*) {
  return AbstractValue::Make(string("from alloc_string"));
}

// AllocVectorCallback returns a MyVector3d(-1,-2,-3).
unique_ptr<BasicVector<double>> alloc_myvector3(
    const Context<double>*) {
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
  static VectorValue<double> fake_cache(MyVector3d::Make(0., 0., 0.));
  fake_cache.get_mutable_vector().set_value(Vector3d(3., 1., 4.));
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
    absport_model_.set_evaluation_function(eval_string);
    vecport_general_.set_evaluation_function(eval_vector3);
    vecport_model_.set_evaluation_function(eval_vector3);

    // We're not using this system but need the port to look like it belongs
    // to one so that validation checks will succeed.
    absport_general_.set_system_and_index(&source_, OutputPortIndex(0));
    absport_model_.set_system_and_index(&source_, OutputPortIndex(1));
    vecport_general_.set_system_and_index(&source_, OutputPortIndex(2));
    vecport_model_.set_system_and_index(&source_, OutputPortIndex(3));
  }

 protected:
  systems::ConstantVectorSource<double> source_{Vector3d(3., -3., 0.)};
  unique_ptr<Context<double>> context_{source_.CreateDefaultContext()};
  unique_ptr<MyVector3d> vec3_model_{MyVector3d::Make(1., 2., 3.)};
  unique_ptr<AbstractValue> abs_model_{
      AbstractValue::Make(string("model string"))};

  // Create abstract- and vector-valued ports specified with either an explicit
  // allocator function, or with a model value of the appropriate type.
  LeafOutputPort<double> absport_general_{alloc_string, calc_string};
  LeafOutputPort<double> absport_model_{*abs_model_, calc_string};
  LeafOutputPort<double> vecport_general_{alloc_myvector3, 3, calc_vector3};
  LeafOutputPort<double> vecport_model_{*vec3_model_, calc_vector3};
};

// Helper function for testing an abstract-valued port.
void AbstractPortCheck(const Context<double>& context,
                       const LeafOutputPort<double>& port,
                       string alloc_string) {
  EXPECT_THROW(port.AllocateVector(), std::logic_error);
  unique_ptr<AbstractValue> val = port.Allocate();
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

  // Same checks for abstract port with model-value based allocation.
  AbstractPortCheck(*context_, absport_model_, "model string");
}

// Helper function for testing a vector-valued port.
// Since all output ports are ultimately AbstractValues, the Allocate() method
// should work even for vector-valued ports whose supplied allocators return
// BasicVectors. (AllocateVector() just does the downcast to extract the
// BasicVector that is contained in the AbstractValue.) Similarly, although
// a CalcVector() function is provided for writing to a BasicVector, Calc()
// should still work and write to an AbstractValue containing a BasicVector.
void VectorPortCheck(const Context<double>& context,
                     const LeafOutputPort<double>& port,
                     Vector3d alloc_value) {
  // First treat the vector-valued port dealing only in BasicVector, which
  // should have MyVector3d as concrete type.
  unique_ptr<BasicVector<double>> val = port.AllocateVector();
  MyVector3d& val_myvector3 = *dynamic_cast<MyVector3d*>(val.get());
  EXPECT_EQ(val->get_value(), alloc_value);
  EXPECT_EQ(val_myvector3.get_value(), alloc_value);
  port.CalcVector(context, val.get());
  // Should have written into the underlying MyVector3d.
  EXPECT_EQ(val_myvector3.get_value(), Vector3d(99., 100., 101.));

  // Now treat the same port as a more-general abstract valued port, but verify
  // that there is still a MyVector3d lurking underneath.
  unique_ptr<AbstractValue> val_abs = port.Allocate();
  auto vvptr = dynamic_cast<VectorValue<double>*>(val_abs.get());
  MyVector3d& val_abs_myvector3 =
      dynamic_cast<MyVector3d&>(vvptr->get_mutable_vector());
  EXPECT_EQ(val_abs_myvector3.get_value(), alloc_value);
  port.Calc(context, val_abs.get());
  // Should have written into the underlying MyVector3d, ultimately making
  // use of the calc_vector3() function with which the port was defined.
  EXPECT_EQ(val_abs_myvector3.get_value(), Vector3d(99., 100., 101.));
}

// Check for proper construction and functioning of vector-valued
// LeafOutputPorts.
TEST_F(LeafOutputPortTest, VectorPorts) {
  // Check vector port with explicit function allocator.
  VectorPortCheck(*context_, vecport_general_, Vector3d(-1., -2., -3.));

  // Check vector port with model-value based allocation.
  VectorPortCheck(*context_, vecport_model_, Vector3d(1., 2., 3.));
}

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

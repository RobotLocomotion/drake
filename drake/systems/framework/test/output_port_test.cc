#include "drake/systems/framework/output_port.h"

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

// These functions match the signatures required by LeafOutputPort.

// AllocCallback returns a string in an AbstractValue.
unique_ptr<AbstractValue> alloc_string(const Context<double>*) {
  return AbstractValue::Make(string("from alloc_string"));
}

// AllocVectorCallback returns a MyVector<3,double> initialized to -1,-2,-3.
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

// EvalCallback returning a MyVector3d.
const AbstractValue& eval_vector3(const Context<double>& context) {
  static VectorValue<double> fake_cache(MyVector3d::Make(0., 0., 0.));
  fake_cache.get_mutable_vector().set_value(Vector3d(3., 1., 4.));
  return fake_cache;
}

// Check for proper functioning of the four LeafOutputPort constructors.
GTEST_TEST(LeafOutputPortTest, Construction) {
  systems::ConstantVectorSource<double> source(Vector3d(3., -3., 0.));
  auto context = source.CreateDefaultContext();
  auto vec3model = MyVector3d::Make(1., 2., 3.);
  auto absmodel = AbstractValue::Make(string("model string"));

  LeafOutputPort<double> abs_general(alloc_string, calc_string);
  LeafOutputPort<double> abs_model(*absmodel, calc_string);
  // TODO(sherm1) Eval methods should be generated automatically if not set.
  // This is just testing the basic wiring.
  abs_general.set_evaluation_function(eval_string);
  abs_model.set_evaluation_function(eval_string);
  abs_general.set_system_and_index(&source, OutputPortIndex(0));
  abs_model.set_system_and_index(&source, OutputPortIndex(1));

  LeafOutputPort<double> vec_general(alloc_myvector3, 3, calc_vector3);
  LeafOutputPort<double> vec_model(*vec3model, calc_vector3);
  vec_general.set_evaluation_function(eval_vector3);
  vec_model.set_evaluation_function(eval_vector3);
  vec_general.set_system_and_index(&source, OutputPortIndex(2));
  vec_model.set_system_and_index(&source, OutputPortIndex(3));

  unique_ptr<AbstractValue> ag_val = abs_general.Allocate();
  EXPECT_EQ(ag_val->GetValueOrThrow<string>(), "from alloc_string");
  abs_general.Calc(*context, ag_val.get());
  EXPECT_EQ(ag_val->GetValueOrThrow<string>(), "from calc_string");
  const AbstractValue& ag_cached = abs_general.Eval(*context);
  EXPECT_EQ(ag_cached.GetValueOrThrow<string>(), "from eval_string");

  unique_ptr<AbstractValue> am_val = abs_model.Allocate();
  EXPECT_EQ(am_val->GetValueOrThrow<string>(), "model string");
  abs_model.Calc(*context, am_val.get());
  EXPECT_EQ(am_val->GetValueOrThrow<string>(), "from calc_string");
  const AbstractValue& am_cached = abs_model.Eval(*context);
  EXPECT_EQ(am_cached.GetValueOrThrow<string>(), "from eval_string");

  // Allocate methods return BasicVector and AbstractValue but should have
  // MyVector3d as concrete type.
  unique_ptr<BasicVector<double>> vg_val = vec_general.AllocateVector();
  MyVector3d& vg_val_myvector3 = *dynamic_cast<MyVector3d*>(vg_val.get());
  EXPECT_EQ(vg_val->get_value(), Vector3d(-1., -2., -3.));
  EXPECT_EQ(vg_val_myvector3.get_value(), Vector3d(-1., -2., -3.));
  vec_general.CalcVector(*context, vg_val.get());
  // Should have written into the underlying MyVector3d.
  EXPECT_EQ(vg_val_myvector3.get_value(), Vector3d(99., 100., 101.));

  unique_ptr<AbstractValue> vg_val_abs = vec_general.Allocate();
  auto vvptr = dynamic_cast<VectorValue<double>*>(vg_val_abs.get());
  MyVector3d& vg_val_abs_myvector3 =
      dynamic_cast<MyVector3d&>(vvptr->get_mutable_vector());
  EXPECT_EQ(vg_val_abs_myvector3.get_value(), Vector3d(-1., -2., -3.));
}

// TODO(sherm1) MORE TESTS COMING.

}  // namespace
}  // namespace systems
}  // namespace drake

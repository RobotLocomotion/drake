#include "drake/systems/framework/value.h"

#include <memory>
#include <sstream>
#include <string>
#include <type_traits>
#include <vector>

#include "gtest/gtest.h"

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/test_utilities/my_vector.h"

namespace drake {
namespace systems {
namespace {

using MyVector2i = MyVector<2, int>;

struct NoDefaultCtor {
  explicit NoDefaultCtor(int i) : data{i} {}
  int data;
};

struct BareStruct {
  int data;
};

GTEST_TEST(ValueTest, DefaultConstructor) {
  // Value<int>() should work because int is default-constructible.
  const AbstractValue& abstract_value = Value<int>();
  EXPECT_EQ(0, abstract_value.GetValue<int>());

  // Value<NoDefaultCtor>() should not work because NoDefaultCtor is not.
  static_assert(!std::is_default_constructible<Value<NoDefaultCtor>>::value,
                "Value<NoDefaultCtor>() should not work.");
}

GTEST_TEST(ValueTest, ForwardingConstructor) {
  // Value<NoDefaultCtor>(int) should work using forwarding.
  const AbstractValue& abstract_value = Value<NoDefaultCtor>(22);
  EXPECT_EQ(22, abstract_value.GetValue<NoDefaultCtor>().data);

  // Value<BareStruct>(BareStruct&&) should use the `(const T&)` constructor,
  // not the forwarding constructor.
  const Value<BareStruct> xvalue_bare(BareStruct{});

  // Value<BareStruct>(BareStruct&) should use the `(const T&)` constructor,
  // not the forwarding constructor.
  BareStruct bare_struct{};
  const Value<BareStruct> lvalue_bare(bare_struct);

  // Value<BareStruct>(const BareStruct&) should use the `(const T&)`
  // constructor, not the forwarding constructor.
  const BareStruct const_bare_struct{};
  const Value<BareStruct> crvalue_bare(const_bare_struct);
}

GTEST_TEST(ValueTest, Make) {
  auto abstract_value = AbstractValue::Make<int>(42);
  EXPECT_EQ(42, abstract_value->GetValue<int>());
}

GTEST_TEST(ValueTest, Access) {
  Value<int> value(3);
  const AbstractValue& erased = value;
  EXPECT_EQ(3, erased.GetValue<int>());
  EXPECT_EQ(3, erased.GetValueOrThrow<int>());
}

GTEST_TEST(ValueTest, Clone) {
  Value<int> value(43);
  const AbstractValue& erased = value;
  std::unique_ptr<AbstractValue> cloned = erased.Clone();
  EXPECT_EQ(43, cloned->GetValue<int>());
}

GTEST_TEST(ValueTest, Mutation) {
  Value<int> value(5);
  value.set_value(6);
  AbstractValue& erased = value;
  EXPECT_EQ(6, erased.GetValue<int>());
  erased.SetValue<int>(7);
  EXPECT_EQ(7, erased.GetValue<int>());
  erased.SetValueOrThrow<int>(8);
  EXPECT_EQ(8, erased.GetValue<int>());
  erased.SetFrom(Value<int>(9));
  EXPECT_EQ(9, erased.GetValue<int>());
  erased.SetFromOrThrow(Value<int>(10));
  EXPECT_EQ(10, erased.GetValue<int>());
}

GTEST_TEST(ValueTest, BadCast) {
  Value<double> value(4);
  AbstractValue& erased = value;
  EXPECT_THROW(erased.GetValueOrThrow<int>(), std::bad_cast);
  EXPECT_THROW(erased.GetMutableValueOrThrow<int>(), std::bad_cast);
  EXPECT_THROW(erased.SetValueOrThrow<int>(3), std::bad_cast);
  EXPECT_THROW(erased.SetFromOrThrow(Value<int>(2)), std::bad_cast);
}

class PrintInterface {
 public:
  virtual ~PrintInterface() {}
  virtual std::string print() const = 0;
};

// A trivial class that implements a trivial interface.
class Point : public PrintInterface {
 public:
  Point(int x, int y) : x_(x), y_(y) {}
  virtual ~Point() {}

  int x() const { return x_; }
  int y() const { return y_; }
  void set_x(int x) { x_ = x; }
  void set_y(int y) { y_ = y; }

  std::string print() const override {
    std::ostringstream out;
    out << x_ << "," << y_;
    return out.str();
  }

 private:
  int x_, y_;
};

// Tests that classes can be erased in an AbstractValue.
GTEST_TEST(ValueTest, ClassType) {
  Point point(1, 2);
  Value<Point> value(point);
  AbstractValue& erased = value;
  erased.GetMutableValue<Point>().set_x(-1);
  EXPECT_EQ(-1, erased.GetValue<Point>().x());
  EXPECT_EQ(2, erased.GetValue<Point>().y());
  erased.GetMutableValueOrThrow<Point>().set_y(-2);
  EXPECT_EQ(-1, erased.GetValue<Point>().x());
  EXPECT_EQ(-2, erased.GetValue<Point>().y());
}

class SubclassOfPoint : public Point {
 public:
  SubclassOfPoint() : Point(-1, -2) {}
};

// Tests that attempting to unerase an AbstractValue to a parent class of the
// original class throws std::bad_cast.
GTEST_TEST(ValueTest, CannotUneraseToParentClass) {
  SubclassOfPoint point;
  Value<SubclassOfPoint> value(point);
  AbstractValue& erased = value;
  EXPECT_THROW(erased.GetMutableValueOrThrow<Point>(), std::bad_cast);
}

// A child class of Value<T> that requires T to satisfy PrintInterface, and
// also satisfies PrintInterface itself.
template <typename T>
class PrintableValue : public Value<T>, public PrintInterface {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PrintableValue)

  explicit PrintableValue(const T& v) : Value<T>(v) {}

  std::unique_ptr<AbstractValue> Clone() const override {
    return std::unique_ptr<PrintableValue<T>>(
        new PrintableValue<T>(this->get_value()));
  }

  std::string print() const override {
    const PrintInterface& print_interface = Value<T>::get_value();
    return print_interface.print();
  }
};

// Tests that AbstractValues can be unerased to interfaces implemented by
// subclasses of Value<T>.
GTEST_TEST(ValueTest, SubclassOfValue) {
  Point point(3, 4);
  PrintableValue<Point> printable_value(point);
  AbstractValue* erased = &printable_value;
  PrintInterface* printable_erased = dynamic_cast<PrintInterface*>(erased);
  ASSERT_NE(nullptr, printable_erased);
  EXPECT_EQ("3,4", printable_erased->print());
}

// Tests that even after being cloned, PrintableValue can be unerased to
// PrintInterface.
GTEST_TEST(ValueTest, SubclassOfValueSurvivesClone) {
  Point point(5, 6);
  PrintableValue<Point> printable_value(point);
  const AbstractValue& erased = printable_value;
  std::unique_ptr<AbstractValue> cloned = erased.Clone();
  PrintInterface* printable_erased =
      dynamic_cast<PrintInterface*>(cloned.get());
  ASSERT_NE(nullptr, printable_erased);
  EXPECT_EQ("5,6", printable_erased->print());
}

GTEST_TEST(VectorValueTest, Access) {
  VectorValue<int> value(BasicVector<int>::Make({1, 2, 3}));
  EXPECT_EQ(1, value.get_value()->get_value().x());
  EXPECT_EQ(2, value.get_value()->get_value().y());
  EXPECT_EQ(3, value.get_value()->get_value().z());
}

GTEST_TEST(VectorValueTest, CopyConstructor) {
  VectorValue<int> value(BasicVector<int>::Make({1, 2, 3}));

  VectorValue<int> other_value(value);
  EXPECT_EQ(1, other_value.get_value()->get_value().x());
  EXPECT_EQ(2, other_value.get_value()->get_value().y());
  EXPECT_EQ(3, other_value.get_value()->get_value().z());
}

GTEST_TEST(VectorValueTest, CopyConstructorSubclass) {
  VectorValue<int> value(MyVector2i::Make(1, 2));
  VectorValue<int> other_value(value);
  value.get_value()->set_value(Eigen::Vector2i(3, 4));

  const MyVector2i* const vector =
      dynamic_cast<const MyVector2i*>(other_value.get_value());
  ASSERT_NE(vector, nullptr);
  EXPECT_EQ(vector->get_value()(0), 1);
  EXPECT_EQ(vector->get_value()(1), 2);
}

GTEST_TEST(VectorValueTest, CopyConstructorNull) {
  VectorValue<int> value{nullptr};

  VectorValue<int> other_value(value);
  EXPECT_EQ(other_value.get_value(), nullptr);
}

GTEST_TEST(VectorValueTest, AssignmentOperator) {
  VectorValue<int> value(BasicVector<int>::Make({1, 2, 3}));
  VectorValue<int> other_value(BasicVector<int>::Make({4, 5, 6}));

  value = other_value;
  EXPECT_EQ(4, value.get_value()->get_value().x());
  EXPECT_EQ(5, value.get_value()->get_value().y());
  EXPECT_EQ(6, value.get_value()->get_value().z());
}

GTEST_TEST(VectorValueTest, AssignmentOperatorSubclass) {
  VectorValue<int> value(MyVector2i::Make(1, 2));
  VectorValue<int> other_value(BasicVector<int>::Make({5, 6}));
  other_value = value;
  value.get_value()->set_value(Eigen::Vector2i(3, 4));

  const MyVector2i* const vector =
      dynamic_cast<const MyVector2i*>(other_value.get_value());
  ASSERT_NE(vector, nullptr);
  EXPECT_EQ(vector->get_value()(0), 1);
  EXPECT_EQ(vector->get_value()(1), 2);
}

GTEST_TEST(VectorValueTest, AssignmentOperatorNull) {
  VectorValue<int> value{nullptr};
  VectorValue<int> other_value(BasicVector<int>::Make({4, 5, 6}));

  value = other_value;
  other_value = VectorValue<int>{nullptr};
  EXPECT_EQ(4, value.get_value()->get_value().x());
  EXPECT_EQ(5, value.get_value()->get_value().y());
  EXPECT_EQ(6, value.get_value()->get_value().z());
  EXPECT_EQ(other_value.get_value(), nullptr);
}

GTEST_TEST(VectorValueTest, AssignmentOperatorSelf) {
  VectorValue<int> value(BasicVector<int>::Make({4, 5, 6}));
  value = value;
  EXPECT_EQ(4, value.get_value()->get_value().x());
  EXPECT_EQ(5, value.get_value()->get_value().y());
  EXPECT_EQ(6, value.get_value()->get_value().z());
}

}  // namespace
}  // namespace systems
}  // namespace drake

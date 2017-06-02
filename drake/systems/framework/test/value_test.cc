#include "drake/systems/framework/value.h"

#include <memory>
#include <sstream>
#include <string>
#include <type_traits>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/drake_copyable.h"

namespace drake {
namespace systems {
namespace {

// A type with no constructors.
struct BareStruct {
  int data;
};

// A copyable type with no default constructor.
struct CopyableInt {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(CopyableInt);
  explicit CopyableInt(int i) : data{i} {}
  CopyableInt(int c1, int c2) : data{c1 * c2} {}

  int data;
};

// A clone-only type with no default constructor.
struct CloneableInt {
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(CloneableInt);
  explicit CloneableInt(int i) : data{i} {}

  std::unique_ptr<CloneableInt> Clone() const {
    return std::make_unique<CloneableInt>(data);
  }

  const int data;
};

// A move-or-clone (not copy) type with a default constructor.
struct MoveOrCloneInt {
  MoveOrCloneInt() {}
  explicit MoveOrCloneInt(int i) : data{i} {}
  MoveOrCloneInt(MoveOrCloneInt&& other) {
    std::swap(data, other.data);
  }
  MoveOrCloneInt& operator=(MoveOrCloneInt&& other)  {
    std::swap(data, other.data);
    return *this;
  }
  MoveOrCloneInt(const MoveOrCloneInt&) = delete;
  void operator=(const MoveOrCloneInt&) = delete;

  std::unique_ptr<MoveOrCloneInt> Clone() const {
    return std::make_unique<MoveOrCloneInt>(data);
  }

  int data{};
};

// Helper for EXPECT_EQ to unwrap the data field.
template <typename T>
bool operator==(int i, const T& value) { return i == value.data; }

// Boilerplate for tests that are identical across different types.  Our
// TYPED_TESTs will run using all of the below types as the TypeParam.
template <typename TypeParam> class TypedValueTest : public ::testing::Test {};
typedef ::testing::Types<
    int,
    CopyableInt,
    CloneableInt,
    MoveOrCloneInt
    > Implementations;
TYPED_TEST_CASE(TypedValueTest, Implementations);

// Value<T>() should work if and only if T is default-constructible.
GTEST_TEST(ValueTest, DefaultConstructor) {
  const AbstractValue& value_int = Value<int>();
  EXPECT_EQ(0, value_int.GetValue<int>());

  const AbstractValue& value_bare_struct = Value<BareStruct>();
  EXPECT_EQ(0, value_bare_struct.GetValue<BareStruct>().data);

  static_assert(!std::is_default_constructible<Value<CopyableInt>>::value,
                "Value<CopyableInt>() should not work.");

  static_assert(!std::is_default_constructible<Value<CloneableInt>>::value,
                "Value<CloneableInt>() should not work.");

  const AbstractValue& value_move_or_clone_int = Value<MoveOrCloneInt>();
  EXPECT_EQ(0, value_move_or_clone_int.GetValue<MoveOrCloneInt>().data);
}

// Value<T>(int) should work (possibly using forwarding).
TYPED_TEST(TypedValueTest, ForwardingConstructor) {
  using T = TypeParam;
  const AbstractValue& abstract_value = Value<T>(22);
  EXPECT_EQ(22, abstract_value.GetValue<T>());
}

// A two-argument constructor should work using forwarding.  (The forwarding
// test case above is not quite enough, because the Value implementation treats
// the first argument and rest of the arguments separately.)
GTEST_TEST(ValueTest, ForwardingConstructorTwoArgs) {
  using T = CopyableInt;
  const AbstractValue& value = Value<T>(11, 2);
  EXPECT_EQ(22, value.GetValue<T>());
}

// Passing a single reference argument to the Value<T> constructor should use
// the `(const T&)` constructor, not the forwarding constructor.
TYPED_TEST(TypedValueTest, CopyConstructor) {
  using T = TypeParam;
  T param{0};
  const T const_param{0};
  const Value<T> xvalue(T{0});          // Called with `T&&`.
  const Value<T> lvalue(param);         // Called with `T&`.
  const Value<T> crvalue(const_param);  // Called with `const T&`.
}

// Ditto for BareStruct.
GTEST_TEST(ValueTest, BareCopyConstructor) {
  using T = BareStruct;
  T param{};
  const T const_param{};
  const Value<T> xvalue(T{});           // Called with `T&&`.
  const Value<T> lvalue(param);         // Called with `T&`.
  const Value<T> crvalue(const_param);  // Called with `const T&`.
}

// Passing a unique_ptr<T> to Value<T> should take over the value.
TYPED_TEST(TypedValueTest, UniquePtrConstructor) {
  using T = TypeParam;
  auto original = std::make_unique<T>(22);
  const Value<T> value{std::move(original)};
  EXPECT_EQ(original.get(), nullptr);
  EXPECT_EQ(22, value.template GetValue<T>());
}

TYPED_TEST(TypedValueTest, Make) {
  using T = TypeParam;
  // TODO(jwnimmer-tri) We should be able to forward this too, and lose the
  // explicit construction of T{42}.
  auto abstract_value = AbstractValue::Make<T>(T{42});
  EXPECT_EQ(42, abstract_value->template GetValue<T>());
}

TYPED_TEST(TypedValueTest, Access) {
  using T = TypeParam;
  Value<T> value(3);
  const AbstractValue& erased = value;
  EXPECT_EQ(3, erased.GetValue<T>());
  EXPECT_EQ(3, erased.GetValueOrThrow<T>());
}

TYPED_TEST(TypedValueTest, Clone) {
  using T = TypeParam;
  Value<T> value(43);
  const AbstractValue& erased = value;
  std::unique_ptr<AbstractValue> cloned = erased.Clone();
  EXPECT_EQ(43, cloned->GetValue<T>());
}

TYPED_TEST(TypedValueTest, Mutation) {
  using T = TypeParam;
  Value<T> value(5);
  value.set_value(T{6});
  AbstractValue& erased = value;
  EXPECT_EQ(6, erased.GetValue<T>());
  erased.SetValue<T>(T{7});
  EXPECT_EQ(7, erased.GetValue<T>());
  erased.SetValueOrThrow<T>(T{8});
  EXPECT_EQ(8, erased.GetValue<T>());
  erased.SetFrom(Value<T>(9));
  EXPECT_EQ(9, erased.GetValue<T>());
  erased.SetFromOrThrow(Value<T>(10));
  EXPECT_EQ(10, erased.GetValue<T>());
}

TYPED_TEST(TypedValueTest, BadCast) {
  using T = TypeParam;
  Value<double> value(4);
  AbstractValue& erased = value;
  EXPECT_THROW(erased.GetValueOrThrow<T>(), std::bad_cast);
  EXPECT_THROW(erased.GetMutableValueOrThrow<T>(), std::bad_cast);
  EXPECT_THROW(erased.SetValueOrThrow<T>(T{3}), std::bad_cast);
  EXPECT_THROW(erased.SetFromOrThrow(Value<T>(2)), std::bad_cast);
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

}  // namespace
}  // namespace systems
}  // namespace drake

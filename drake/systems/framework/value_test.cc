#include "drake/systems/framework/value.h"

#include <memory>
#include <sstream>
#include <string>

#include "gtest/gtest.h"

namespace drake {
namespace systems {
namespace {

TEST(ValueTest, Access) {
  Value<int> value(3);
  const AbstractValue& erased = value;
  EXPECT_EQ(3, erased.GetValue<int>());
}

TEST(ValueTest, Copy) {
  Value<int> value(42);
  Value<int> copied_value = value;
  EXPECT_EQ(42, copied_value.get_value());
}

TEST(ValueTest, Clone) {
  Value<int> value(43);
  const AbstractValue& erased = value;
  std::unique_ptr<AbstractValue> cloned = erased.Clone();
  EXPECT_EQ(43, cloned->GetValue<int>());
}

TEST(ValueTest, Mutation) {
  Value<int> value(5);
  value.set_value(6);
  AbstractValue& erased = value;
  EXPECT_EQ(6, erased.GetValue<int>());
  erased.SetValue<int>(7);
  EXPECT_EQ(7, erased.GetValue<int>());
}

TEST(ValueTest, BadCast) {
  Value<double> value(4);
  const AbstractValue& erased = value;
  EXPECT_THROW(erased.GetValue<int>(), std::bad_cast);
}

class PrintInterface {
 public:
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

  std::string print() const override {
    std::ostringstream out;
    out << x_ << "," << y_;
    return out.str();
  }

 private:
  int x_, y_;
};

// Tests that classes can be erased in an AbstractValue.
TEST(ValueTest, ClassType) {
  Point point(1, 2);
  Value<Point> value(point);
  AbstractValue& erased = value;
  erased.GetMutableValue<Point>().set_x(-1);
  EXPECT_EQ(-1, erased.GetValue<Point>().x());
  EXPECT_EQ(2, erased.GetValue<Point>().y());
}

// A child class of Value<T> that requires T to satisfy PrintInterface, and
// also satisfies PrintInterface itself.
template <typename T>
class PrintableValue : public Value<T>, public PrintInterface {
 public:
  explicit PrintableValue(const T& v) : Value<T>(v) {}

  // PrintableValues are copyable but not moveable.
  PrintableValue(const PrintableValue<T>& other) = default;
  PrintableValue& operator=(const PrintableValue<T>& other) = default;
  PrintableValue(PrintableValue<T>&& other) = delete;
  PrintableValue& operator=(PrintableValue<T>&& other) = delete;

  virtual std::unique_ptr<AbstractValue> Clone() const override {
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
TEST(ValueTest, SubclassOfValue) {
  Point point(3, 4);
  PrintableValue<Point> printable_value(point);
  AbstractValue* erased = &printable_value;
  PrintInterface* printable_erased = dynamic_cast<PrintInterface*>(erased);
  ASSERT_NE(nullptr, printable_erased);
  EXPECT_EQ("3,4", printable_erased->print());
}

// Tests that even after being cloned, PrintableValue can be unerased to
// PrintInterface.
TEST(ValueTest, SubclassOfValueSurvivesClone) {
  Point point(5, 6);
  PrintableValue<Point> printable_value(point);
  const AbstractValue& erased = printable_value;
  std::unique_ptr<AbstractValue> cloned = erased.Clone();
  PrintInterface* printable_erased = dynamic_cast<PrintInterface*>(
      cloned.get());
  ASSERT_NE(nullptr, printable_erased);
  EXPECT_EQ("5,6", printable_erased->print());
}


}  // namespace
}  // namespace systems
}  // namespace drake

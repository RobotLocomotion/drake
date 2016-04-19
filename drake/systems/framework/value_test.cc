#include "drake/systems/framework/value.h"

#include "gtest/gtest.h"

namespace drake {
namespace systems {
namespace {

TEST(ValueTest, Access) {
  Value<int> value(3);
  const AbstractValue& erased = value;
  EXPECT_EQ(3, erased.GetValue<int>());
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
  EXPECT_THROW(erased.GetValue<int>(), std::runtime_error);
}

// A two-dimensional point, for testing purposes.
class Point {
 public:
  Point(int x, int y)
      : x_(x), y_(y) {}
  int x() const { return x_; }
  int y() const { return y_; }
  void set_x(int x) { x_ = x; }

 private:
  int x_, y_;
};

TEST(ValueTest, ClassType) {
  Point point(1, 2);
  Value<Point> value(point);
  AbstractValue& erased = value;
  erased.GetMutableValue<Point>().set_x(-1);
  EXPECT_EQ(-1, erased.GetValue<Point>().x());
  EXPECT_EQ(2, erased.GetValue<Point>().y());
}

}  // namespace
}  // namespace systems
}  // namespace drake

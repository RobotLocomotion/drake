#include "drake/systems/framework/output_port_value.h"

#include <memory>
#include <string>

#include <gtest/gtest.h>

#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/test_utilities/my_vector.h"
#include "drake/systems/framework/value.h"

namespace drake {
namespace systems {


class SystemOutputTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Vector output port 0 is just a BasicVector<double>.
    std::unique_ptr<BasicVector<double>> vec(new BasicVector<double>(2));
    vec->get_mutable_value() << 5, 25;
    output_.add_port(std::move(vec));

    // Vector output port 1 is derived from BasicVector<double>. The concrete
    // type should be preserved when copying.
    auto my_vec = MyVector<3, double>::Make(125, 625, 3125);
    output_.add_port(std::move(my_vec));

    // Abstract output port 2 is a string.
    auto str = AbstractValue::Make(std::string("foo"));
    output_.add_port(std::move(str));
  }

  SystemOutput<double> output_;
};

TEST_F(SystemOutputTest, Access) {
  VectorX<double> expected(2);
  expected << 5, 25;
  EXPECT_EQ(
      expected,
      output_.get_vector_data(0)->get_value());

  EXPECT_EQ("foo", output_.get_data(2)->GetValue<std::string>());
}

TEST_F(SystemOutputTest, Mutation) {
  output_.GetMutableVectorData(0)->get_mutable_value() << 7, 8;

  // Check that the vector contents changed.
  VectorX<double> expected(2);
  expected << 7, 8;
  EXPECT_EQ(expected, output_.get_vector_data(0)->get_value());

  output_.GetMutableData(2)->GetMutableValue<std::string>() = "bar";

  // Check that the contents changed.
  EXPECT_EQ("bar", output_.get_data(2)->GetValue<std::string>());
}

// Tests that a copy of a SystemOutput has a deep copy of the data.
TEST_F(SystemOutputTest, Copy) {
  auto clone(output_);
  // Changes to the original port should not affect the clone.
  output_.GetMutableVectorData(0)->get_mutable_value() << 7, 8;
  output_.GetMutableData(2)->GetMutableValue<std::string>() = "bar";

  VectorX<double> expected(2);
  expected << 5, 25;
  EXPECT_EQ(expected, clone.get_vector_data(0)->get_value());

  EXPECT_EQ("foo", clone.get_data(2)->GetValue<std::string>());

  // The type and value should be preserved.
  const BasicVector<double>* basic = clone.get_vector_data(1);
  auto* my_vec = dynamic_cast<const MyVector<3, double>*>(basic);
  ASSERT_NE(my_vec, nullptr);

  expected.resize(3);
  expected << 125, 625, 3125;
  EXPECT_EQ(expected, my_vec->get_value());

  EXPECT_EQ(clone.get_data(2)->GetValueOrThrow<std::string>(), "foo");
}

}  // namespace systems
}  // namespace drake

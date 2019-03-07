#include "drake/systems/framework/system_output.h"

#include <memory>
#include <string>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/is_dynamic_castable.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/test_utilities/my_vector.h"

namespace drake {
namespace systems {

// Construction and add_port() calls here are permitted only because this test
// class has been granted friend access to SystemOutput. Otherwise a
// SystemOutput<T> object can only be created by a System<T> object, or by
// copying an existing SystemOutput object.
class SystemOutputTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Vector output port 0 is just a BasicVector<double>.
    const BasicVector<double> vec{5, 25};
    output_.add_port(AbstractValue::Make(vec));

    // Vector output port 1 is derived from BasicVector<double>. The concrete
    // type should be preserved when copying.
    auto my_vec = MyVector3d::Make(125, 625, 3125);
    auto my_basic_vec =
        std::make_unique<Value<BasicVector<double>>>(std::move(my_vec));
    output_.add_port(std::move(my_basic_vec));

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

  EXPECT_EQ("foo", output_.get_data(2)->get_value<std::string>());
}

TEST_F(SystemOutputTest, Mutation) {
  output_.GetMutableVectorData(0)->get_mutable_value() << 7, 8;

  // Check that the vector contents changed.
  VectorX<double> expected(2);
  expected << 7, 8;
  EXPECT_EQ(expected, output_.get_vector_data(0)->get_value());

  output_.GetMutableData(2)->get_mutable_value<std::string>() = "bar";

  // Check that the contents changed.
  EXPECT_EQ("bar", output_.get_data(2)->get_value<std::string>());
}

// Tests that a copy of a SystemOutput has a deep copy of the data.
TEST_F(SystemOutputTest, Copy) {
  SystemOutput<double> copy(output_);
  // Changes to the original port should not affect the copy.
  output_.GetMutableVectorData(0)->get_mutable_value() << 7, 8;
  output_.GetMutableData(2)->get_mutable_value<std::string>() = "bar";

  VectorX<double> expected(2);
  expected << 5, 25;
  EXPECT_EQ(expected, copy.get_vector_data(0)->get_value());

  EXPECT_EQ("foo", copy.get_data(2)->get_value<std::string>());

  // The type and value should be preserved.
  const BasicVector<double>* basic = copy.get_vector_data(1);
  ASSERT_TRUE((is_dynamic_castable<const MyVector3d>(basic)));

  expected.resize(3);
  expected << 125, 625, 3125;
  EXPECT_EQ(expected, basic->get_value());
}

}  // namespace systems
}  // namespace drake

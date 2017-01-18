#include "drake/systems/framework/system_port_descriptor.h"

#include <memory>
#include <string>

#include "gtest/gtest.h"

#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {

class TestSystem : public LeafSystem<double> {
 public:
  TestSystem() {}
  ~TestSystem() override {}

  void DoCalcOutput(const Context<double>& context,
                    SystemOutput<double>* output) const override {}
};

GTEST_TEST(InputPortDescriptor, CopyConstructable) {
  TestSystem system;
  InputPortDescriptor<double> descriptor(&system, 42, kVectorValued, 28);
  InputPortDescriptor<double> copy(descriptor);
  EXPECT_EQ(&system, copy.get_system());
  EXPECT_EQ(42, copy.get_index());
  EXPECT_EQ(kVectorValued, copy.get_data_type());
  EXPECT_EQ(28, copy.size());
}

GTEST_TEST(InputPortDescriptor, Assignable) {
  TestSystem system;
  InputPortDescriptor<double> descriptor(&system, 42, kVectorValued, 28);
  InputPortDescriptor<double> copy(&system, 1, kAbstractValued, 2);
  copy = descriptor;
  EXPECT_EQ(&system, copy.get_system());
  EXPECT_EQ(42, copy.get_index());
  EXPECT_EQ(kVectorValued, copy.get_data_type());
  EXPECT_EQ(28, copy.size());
}

GTEST_TEST(OutputPortDescriptor, CopyConstructable) {
  TestSystem system;
  OutputPortDescriptor<double> descriptor(&system, 42, kVectorValued, 28);
  OutputPortDescriptor<double> copy(descriptor);
  EXPECT_EQ(&system, copy.get_system());
  EXPECT_EQ(42, copy.get_index());
  EXPECT_EQ(kVectorValued, copy.get_data_type());
  EXPECT_EQ(28, copy.size());
}

GTEST_TEST(OutputPortDescriptor, Assignable) {
  TestSystem system;
  OutputPortDescriptor<double> descriptor(&system, 42, kVectorValued, 28);
  OutputPortDescriptor<double> copy(&system, 1, kAbstractValued, 2);
  copy = descriptor;
  EXPECT_EQ(&system, copy.get_system());
  EXPECT_EQ(42, copy.get_index());
  EXPECT_EQ(kVectorValued, copy.get_data_type());
  EXPECT_EQ(28, copy.size());
}

}  // namespace systems
}  // namespace drake

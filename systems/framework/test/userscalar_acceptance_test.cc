// @file
// These test cases demonstrates that the systems framework can be used with a
// scalar type other than one of Drake's built-in scalars.

#include <gtest/gtest.h>

#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/test_utilities/my_vector.h"

namespace drake {
namespace systems {
namespace {

// A scalar type that is not one of drake's default scalars.
using UserScalar = Eigen::AutoDiffScalar<Eigen::Matrix<double, 11, 1>>;

using VariousScalars = ::testing::Types<
  double,
  AutoDiffXd,
  symbolic::Expression,
  UserScalar>;

template <typename T>
class UserscalarAcceptanceTest : public ::testing::Test {};
TYPED_TEST_CASE(UserscalarAcceptanceTest, VariousScalars);

template <typename T>
class TestSystem : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TestSystem)

  using StateVector = MyVector<2, T>;
  using OutputVector = MyVector<1, T>;

  TestSystem() {
    this->DeclareContinuousState(StateVector{});
    this->DeclareVectorOutputPort(OutputVector{}, &TestSystem::CalcOutput);
  }

  void CalcOutput(const Context<T>& context, OutputVector* output) const {
    (*output)[0] = context.get_continuous_state_vector()[0];
  }
};

TYPED_TEST(UserscalarAcceptanceTest, LeafSystemTest) {
  using T = TypeParam;
  const TestSystem<T> dut;
  EXPECT_EQ(dut.get_num_input_ports(), 0);
  EXPECT_EQ(dut.get_num_output_ports(), 1);
  auto context = dut.CreateDefaultContext();
  EXPECT_NE(context, nullptr);
}

// TODO(jwnimmer-tri) Test uses of Diagram as well.

}  // namespace
}  // namespace systems
}  // namespace drake

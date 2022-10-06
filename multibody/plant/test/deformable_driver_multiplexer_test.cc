#include <gtest/gtest.h>

#include "drake/multibody/plant/deformable_driver.h"

namespace drake {
namespace multibody {
namespace internal {

class DeformableDriverMultiplexerTest : public ::testing::Test {
 protected:
  /* Sets `dut_` to a DeformableDriver::Multiplexer with the given sizes. */
  void MakeMultiplexer(std::vector<int> sizes) {
    dut_ = DeformableDriver<double>::Multiplexer(std::move(sizes));
  }

  /* Sets `dut_` to an empty DeformableDriver::Multiplexer. */
  void MakeEmptyMultiplexer() {
    dut_ = DeformableDriver<double>::Multiplexer();
  }

  DeformableDriver<double>::Multiplexer dut_;
};

namespace {

TEST_F(DeformableDriverMultiplexerTest, Constructors) {
  /* Valid constructions. */
  EXPECT_NO_THROW(MakeEmptyMultiplexer());
  EXPECT_EQ(dut_.num_vectors(), 0);
  EXPECT_NO_THROW(MakeMultiplexer({1, 2, 3}));
  EXPECT_EQ(dut_.num_vectors(), 3);
  /* Invalid constructions. */
  EXPECT_THROW(MakeMultiplexer({}), std::exception);
  EXPECT_THROW(MakeMultiplexer({-1}), std::exception);
}

TEST_F(DeformableDriverMultiplexerTest, Multiplex) {
  const Eigen::Vector2d v2(0.1, 0.2);
  const Eigen::Vector3d v3(0.1, 0.2, 0.3);
  const Eigen::Vector4d v4(0.1, 0.2, 0.3, 0.4);
  MakeMultiplexer({2, 3});
  const VectorX<double> result = dut_.Multiplex({v2, v3});
  VectorX<double> expected_result(5);
  expected_result << v2, v3;
  EXPECT_EQ(result, expected_result);

  EXPECT_THROW(dut_.Multiplex({v2}), std::exception);
  EXPECT_THROW(dut_.Multiplex({v2, v4}), std::exception);
}

TEST_F(DeformableDriverMultiplexerTest, Demultiplex) {
  const Eigen::Vector2d v2(0.1, 0.2);
  const Eigen::Vector3d v3(0.1, 0.2, 0.3);
  MakeMultiplexer({2, 3});
  VectorX<double> mux(5);
  mux << v2, v3;
  EXPECT_EQ(dut_.Demultiplex(mux, 0), v2);
  EXPECT_EQ(dut_.Demultiplex(mux, 1), v3);

  const Eigen::Vector4d bad_vector(0.1, 0.2, 0.3, 0.4);
  EXPECT_THROW(dut_.Demultiplex(mux, -1), std::exception);
  EXPECT_THROW(dut_.Demultiplex(mux, 2), std::exception);
  EXPECT_THROW(dut_.Demultiplex(bad_vector, 0), std::exception);
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake

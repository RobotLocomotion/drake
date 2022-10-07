#include <gtest/gtest.h>

#include "drake/multibody/plant/deformable_driver.h"

namespace drake {
namespace multibody {
namespace internal {
namespace {

GTEST_TEST(DeformableDriverMultiplexerTest, Constructors) {
  /* Valid constructions. */
  Multiplexer<double> dut;
  EXPECT_EQ(dut.num_vectors(), 0);
  dut = Multiplexer<double>({1, 2, 3});
  EXPECT_EQ(dut.num_vectors(), 3);
  /* Invalid constructions. */
  EXPECT_THROW(Multiplexer<double>{{}}, std::exception);
  EXPECT_THROW(Multiplexer<double>{{-1}}, std::exception);
}

GTEST_TEST(DeformableDriverMultiplexerTest, Multiplex) {
  const Eigen::Vector2d v2(0.1, 0.2);
  const Eigen::Vector3d v3(0.1, 0.2, 0.3);
  const Eigen::Vector4d v4(0.1, 0.2, 0.3, 0.4);
  Multiplexer<double> dut({2, 3});
  const VectorX<double> result = dut.Multiplex({v2, v3});
  VectorX<double> expected_result(5);
  expected_result << v2, v3;
  EXPECT_EQ(result, expected_result);

  EXPECT_THROW(dut.Multiplex({v2}), std::exception);
  EXPECT_THROW(dut.Multiplex({v2, v4}), std::exception);
}

GTEST_TEST(DeformableDriverMultiplexerTest, Demultiplex) {
  const Eigen::Vector2d v2(0.1, 0.2);
  const Eigen::Vector3d v3(0.1, 0.2, 0.3);
  Multiplexer<double> dut({2, 3});
  VectorX<double> mux(5);
  mux << v2, v3;
  EXPECT_EQ(dut.Demultiplex(mux, 0), v2);
  EXPECT_EQ(dut.Demultiplex(mux, 1), v3);

  const Eigen::Vector4d bad_vector(0.1, 0.2, 0.3, 0.4);
  EXPECT_THROW(dut.Demultiplex(mux, -1), std::exception);
  EXPECT_THROW(dut.Demultiplex(mux, 2), std::exception);
  EXPECT_THROW(dut.Demultiplex(bad_vector, 0), std::exception);
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake

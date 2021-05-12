#include "drake/math/fast_pose_composition_functions.h"

#include <limits>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace math {
namespace  {
using Eigen::Matrix3d;

GTEST_TEST(TestFastPoseCompositionFunctions, UsingAVX) {
#ifdef __APPLE__
  constexpr bool kApple = true;
#else
  constexpr bool kApple = false;
#endif

  EXPECT_EQ(IsUsingPortableCompositionMethods(), kApple);
}

// Test the given RotationMatrix composition function for correct functionality
// (just a matrix multiply in this case) and that it still works when the output
// overlaps in memory with the inputs.
void TestRR(
    std::function<void(const double*, const double*, double*)> compose_RR) {
  Matrix3d M, N;
  M << 1, 5, 9, 2, 6, 10, 3, 7, 11;
  N << 13, 17, 21, 14, 18, 22, 15, 19, 23;
  const Matrix3d MM_expected = M * M;
  const Matrix3d MN_expected = M * N;

  Matrix3d MN;
  compose_RR(M.data(), N.data(), MN.data());

  // Should be a perfect match with integer elements.
  EXPECT_TRUE(CompareMatrices(MN, MN_expected, 0));

  // Now test in-place compositions.
  Matrix3d Mwork = M, Nwork = N;  // Copies to overwrite.

  // Results should be perfect match with integer elements.
  compose_RR(Mwork.data(), Nwork.data(), Mwork.data());  // Mwork=M*N
  EXPECT_TRUE(CompareMatrices(Mwork, MN_expected, 0));
  Mwork = M;                                             // Restore value.
  compose_RR(Mwork.data(), Nwork.data(), Nwork.data());  // Nwork=M*N
  EXPECT_TRUE(CompareMatrices(Nwork, MN_expected, 0));
  compose_RR(Mwork.data(), Mwork.data(), Mwork.data());  // Mwork=M*M
  EXPECT_TRUE(CompareMatrices(Mwork, MM_expected, 0));
}

/* Test the user-callable methods first. Those are ideally implemented
with SIMD instructions, however they may just punt to the plain C++ fallback
methods, which we'll test separately below. */
GTEST_TEST(TestFastPoseCompositionFunctions, TestRotationCompositions) {
  SCOPED_TRACE("testing ComposeRR()");
  TestRR(ComposeRR);
}

GTEST_TEST(TestFastPoseCompositionFunctions, TestPortableRotationCompositions) {
  SCOPED_TRACE("testing internal::ComposeRRPortable()");
  TestRR(internal::ComposeRRPortable);
}

}  // namespace
}  // namespace math
}  // namespace drake

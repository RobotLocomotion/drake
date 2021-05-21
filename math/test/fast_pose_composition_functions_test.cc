#include "drake/math/fast_pose_composition_functions.h"

#include <cstdio>
#include <limits>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace math {

namespace  {
using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Matrix34d = Eigen::Matrix<double, 3, 4>;

GTEST_TEST(TestFastPoseCompositionFunctions, UsingAVX) {
#ifdef __APPLE__
  constexpr bool kApple = true;
#else
  constexpr bool kApple = false;
#endif

  EXPECT_EQ(IsUsingPortableCompositionFunctions(), kApple);
}

// Test the given RotationMatrix composition function for correct functionality
// (just matrix multiply or matrix transpose multiply here) and that it still
// works when the output overlaps in memory with the inputs.
void TestRxR(
    std::function<void(const double*, const double*, double*)> compose_RxR,
    bool invert_first_matrix) {

  // Note that M and N are not legitimate RotationMatrix values. We are just
  // testing that the correct matrix operations are performed.
  Matrix3d M, N;
  M << 1, 5, 9,
       2, 6, 10,
       3, 7, 11;
  N << 13, 17, 21,
       14, 18, 22,
       15, 19, 23;
  // Inverse is just transpose for a RotationMatrix.
  const Matrix3d Mx = invert_first_matrix ? M.transpose() : M;
  const Matrix3d MxM_expected = Mx * M;
  const Matrix3d MxN_expected = Mx * N;

  Matrix3d MxN;
  compose_RxR(M.data(), N.data(), MxN.data());

  // Should be a perfect match with integer elements.
  EXPECT_TRUE(CompareMatrices(MxN, MxN_expected, 0));

  // Now test in-place compositions.
  Matrix3d Mwork = M, Nwork = N;  // Copies to overwrite.

  // Results should be perfect match with integer elements.
  compose_RxR(Mwork.data(), Nwork.data(), Mwork.data());  // Mwork=Mx*N
  EXPECT_TRUE(CompareMatrices(Mwork, MxN_expected, 0));
  Mwork = M;  // Restore value.
  compose_RxR(Mwork.data(), Nwork.data(), Nwork.data());  // Nwork=Mx*N
  EXPECT_TRUE(CompareMatrices(Nwork, MxN_expected, 0));
  compose_RxR(Mwork.data(), Mwork.data(), Mwork.data());  // Mwork=Mx*M
  EXPECT_TRUE(CompareMatrices(Mwork, MxM_expected, 0));
}

// TODO(sherm1) TestXxX() function for ComposeXX() and ComposeXinvX().

/* Test the user-callable methods first. Those are ideally implemented
with SIMD instructions, however they may just punt to the plain C++ fallback
methods, which we'll test separately below. */

GTEST_TEST(TestFastPoseCompositionFunctions, TestRR) {
  SCOPED_TRACE("testing ComposeRR()");
  TestRxR(ComposeRR, false);
}
GTEST_TEST(TestFastPoseCompositionFunctions, TestRinvR) {
  SCOPED_TRACE("testing ComposeRinvR()");
  TestRxR(ComposeRinvR, true);
}

// TODO(sherm1) ComposeXX() and ComposeXinvX() tests.

/* Now repeat these tests for the portable methods. */

GTEST_TEST(TestFastPoseCompositionFunctions, TestRRPortable) {
  SCOPED_TRACE("testing internal::ComposeRR()");
  TestRxR(internal::ComposeRRPortable, false);
}
GTEST_TEST(TestFastPoseCompositionFunctions, TestRinvRPortable) {
  SCOPED_TRACE("testing internal::ComposeRinvR()");
  TestRxR(internal::ComposeRinvRPortable, true);
}

// TODO(sherm1) ComposeXXPortable() and ComposeXinvXPortable() tests.

}  // namespace

}  // namespace math
}  // namespace drake

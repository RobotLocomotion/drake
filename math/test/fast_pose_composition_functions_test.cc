#include "drake/math/fast_pose_composition_functions.h"

#include <string>
#include <tuple>

#include "hwy/tests/hwy_gtest.h"
#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/common/hwy_dynamic.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"

namespace drake {
namespace math {
namespace internal {
namespace {
using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Matrix34d = Eigen::Matrix<double, 3, 4>;

// Note that A's and B's rotation matrices are not legitimate. We are just
// testing that the correct matrix operations are performed, and using integer
// values is a lot easier for exact testing and debugging than floating point.
RigidTransformd MakeA() {
  Matrix34d value;
  // clang-format off
  value << 1, 4, 7, 10,
           2, 5, 8, 11,
           3, 6, 9, 12;
  // clang-format on
  return RigidTransformd::MakeUnchecked(value);
}

RigidTransformd MakeAinv() {
  Matrix34d A = MakeA().GetAsMatrix34();
  Matrix3d Rt = A.leftCols(3).transpose();
  Matrix34d value;
  value.block<3, 3>(0, 0) = Rt;
  value.block<3, 1>(0, 3) = -Rt * A.col(3);
  return RigidTransformd::MakeUnchecked(value);
}

RigidTransformd MakeB() {
  Matrix34d value;
  // clang-format off
  value << 13, 16, 19, 22,
           14, 17, 20, 23,
           15, 18, 21, 24;
  // clang-format on
  return RigidTransformd::MakeUnchecked(value);
}

struct Param {
  /* The 'args' string encodes a particular argument order to some particular
  function under test (e.g., ComposeRR). For example if 'args' is "ABO" then the
  test will call `ComposeRR(A, B, O)` -- each test case has local variables A,
  B, and O. */
  std::string args;

  /* When this is true, the test checks ComposeRinvR instead of ComposeRR, or
  ComposeXinvX instead of ComposeXX. */
  bool inverse{};

  /* A printer so that test names are helpful. */
  friend std::ostream& operator<<(std::ostream& out, const Param& param) {
    out << param.args;
    if (param.inverse) {
      out << "inv";
    }
    return out;
  }
};

// TODO(jwnimmer-tri) This similar of fixture (using hwy_gtest.h) is duplicated
// in two other places (//common and //geometry/proximity). We should probably
// seek a more elegant way to do this, instead of copying it to more places.

/* This hwy-infused test fixture replicates every test case to be run against
every target architecture variant (e.g., SSE4, AVX2, AVX512VL, etc). When run,
it filters the suite to only run tests that the current CPU can handle. */
class FastPoseCompositionFunctions
    : public hwy::TestWithParamTargetAndT<Param> {
 protected:
  void SetUp() override {
    // Reset Drake's dispatcher, to be sure that we run all of the target
    // architectures.
    drake::internal::HwyDynamicReset();
    hwy::TestWithParamTargetAndT<Param>::SetUp();
  }
};

/* All possible permutations of argument aliasing. */
auto MakePermutations() {
  return ::testing::Values(Param{.args = "ABO", .inverse = false},
                           Param{.args = "ABA", .inverse = false},
                           Param{.args = "ABB", .inverse = false},
                           Param{.args = "AAA", .inverse = false},
                           Param{.args = "ABO", .inverse = true},
                           Param{.args = "ABA", .inverse = true},
                           Param{.args = "ABB", .inverse = true},
                           Param{.args = "AAA", .inverse = true});
}

// Instatiate the suite for all CPU targets (using the HWY macro), crossed with
// all Param permutations from MakePermutations().
HWY_TARGET_INSTANTIATE_TEST_SUITE_P_T(FastPoseCompositionFunctions,
                                      MakePermutations());

TEST_P(FastPoseCompositionFunctions, ComposeRR) {
  const Param& param = GetParam();

  // We have three options for input / output objects.
  RotationMatrixd A = MakeA().rotation();
  RotationMatrixd B = MakeB().rotation();
  RotationMatrixd O;

  // Choose which arguments we want to pass in this instance.
  auto lookup = [&A, &B, &O](char abo) -> RotationMatrixd* {
    if (abo == 'A') return &A;
    if (abo == 'B') return &B;
    if (abo == 'O') return &O;
    DRAKE_UNREACHABLE();
  };
  const RotationMatrixd* const arg1 = lookup(param.args.at(0));
  const RotationMatrixd* const arg2 = lookup(param.args.at(1));
  RotationMatrixd* const arg3 = lookup(param.args.at(2));

  // We must prepare the answer prior to calling the function. Some calls will
  // alias their input and output.
  DRAKE_DEMAND(param.args.at(0) == 'A');
  const Matrix3d expected =
      (param.inverse ? MakeAinv() : MakeA()).rotation().matrix() *
      arg2->matrix();

  // Call the function under test.
  (param.inverse ? ComposeRinvR : ComposeRR)(*arg1, *arg2, arg3);

  // Check the answer.
  EXPECT_TRUE(CompareMatrices(arg3->matrix(), expected));
}

TEST_P(FastPoseCompositionFunctions, ComposeXX) {
  const Param& param = GetParam();

  // We have three options for input / output objects.
  RigidTransformd A = MakeA();
  RigidTransformd B = MakeB();
  RigidTransformd O;

  // Choose which arguments we want to pass in this instance.
  auto lookup = [&A, &B, &O](char abo) -> RigidTransformd* {
    if (abo == 'A') return &A;
    if (abo == 'B') return &B;
    if (abo == 'O') return &O;
    DRAKE_UNREACHABLE();
  };
  const RigidTransformd* const arg1 = lookup(param.args.at(0));
  const RigidTransformd* const arg2 = lookup(param.args.at(1));
  RigidTransformd* const arg3 = lookup(param.args.at(2));

  // We must prepare the answer prior to calling the function. Some calls will
  // alias their input and output.
  DRAKE_DEMAND(param.args.at(0) == 'A');
  const Matrix4d expected =
      (param.inverse ? MakeAinv() : MakeA()).GetAsMatrix4() *
      arg2->GetAsMatrix4();

  // Call the function under test.
  (param.inverse ? ComposeXinvX : ComposeXX)(*arg1, *arg2, arg3);

  // Check the answer.
  EXPECT_TRUE(CompareMatrices(arg3->GetAsMatrix4(), expected));
}

}  // namespace
}  // namespace internal
}  // namespace math
}  // namespace drake

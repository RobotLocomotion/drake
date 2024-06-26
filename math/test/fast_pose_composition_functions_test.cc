#include "drake/math/fast_pose_composition_functions.h"

#include <tuple>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wold-style-cast"
#include "hwy/tests/hwy_gtest.h"
#pragma GCC diagnostic pop

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
using Eigen::Matrix4d;  // xxx
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
  Matrix34d value = MakeA().GetAsMatrix34();
  Matrix3d Rinv = value.leftCols(3).transpose();
  value.block<3, 3>(0, 0) = Rinv;
  value.block<3, 1>(0, 3) = -Rinv * value.col(3);
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
  std::string args;
  bool inverse{};

  friend std::ostream& operator<<(std::ostream& out, const Param& param) {
    out << param.args;
    if (param.inverse) {
      out << "inv";
    }
    return out;
  }
};

class FastPoseCompositionFunctions
    : public hwy::TestWithParamTargetAndT<Param> {
 protected:
  void SetUp() override {
    drake::internal::HwyDynamicReset();
    hwy::TestWithParamTargetAndT<Param>::SetUp();
  }
};

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

HWY_TARGET_INSTANTIATE_TEST_SUITE_P_T(FastPoseCompositionFunctions,
                                      MakePermutations());

TEST_P(FastPoseCompositionFunctions, ComposeRR) {
  const Param& param = GetParam();

  // We have three options for input / output objects.
  RotationMatrixd A =
      param.inverse ? MakeAinv().rotation() : MakeA().rotation();
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
  const Matrix3d expected =
      (param.inverse ? Matrix3d{arg1->matrix().transpose()} : arg1->matrix()) *
      arg2->matrix();

  // Call the function under test.
  (param.inverse ? ComposeRinvR : ComposeRR)(*arg1, *arg2, arg3);

  // Check the answer.
  EXPECT_TRUE(CompareMatrices(arg3->matrix(), expected));
}

#if 0  // XXX
TEST_P(FastPoseCompositionFunctions, TestXX) {
  SCOPED_TRACE("testing ComposeXX()");
  TestXxX(internal::ComposeXX, false);
}
#endif

}  // namespace
}  // namespace internal
}  // namespace math
}  // namespace drake

#include "drake/common/eigen_types.h"

#include <iostream>

#include <gtest/gtest.h>

#include "drake/common/nice_type_name.h"
#include "drake/common/test_utilities/expect_no_throw.h"

using Eigen::MatrixXd;
using Eigen::Matrix3d;
using Eigen::VectorXd;
using Eigen::Vector3d;
using Eigen::ArrayXXd;
using Eigen::Array33d;
using Eigen::ArrayXd;
using Eigen::Array3d;

namespace drake {
namespace {

template <typename Derived>
void CheckTraits(bool is_vector) {
  std::string info = "Type: " + NiceTypeName::Get<Derived>() + "\n";
  EXPECT_TRUE(is_eigen_type<Derived>::value) << info;
  EXPECT_TRUE((is_eigen_scalar_same<Derived, double>::value)) << info;
  EXPECT_EQ(is_vector, (is_eigen_vector_of<Derived, double>::value)) << info;
  EXPECT_EQ(!is_vector, (is_eigen_nonvector_of<Derived, double>::value))
      << info;
}

// Test traits within eigen_types.h
GTEST_TEST(EigenTypesTest, TraitsPositive) {
  // MatrixBase<>
  CheckTraits<MatrixXd>(false);
  CheckTraits<Matrix3d>(false);
  CheckTraits<VectorXd>(true);
  CheckTraits<Vector3d>(true);
  // ArrayBase<>
  CheckTraits<ArrayXXd>(false);
  CheckTraits<Array33d>(false);
  CheckTraits<ArrayXd>(true);
  CheckTraits<Array3d>(true);
}

// SFINAE check.
bool IsEigen(...) {
  return false;
}
template <typename T,
          typename Cond = typename std::enable_if_t<
              is_eigen_type<T>::value>>
bool IsEigen(const T&) {
  return true;
}

// SFINAE check with edge case described below.
bool IsEigenOfDouble(...) {
  return false;
}
template <typename T,
          typename Cond = typename std::enable_if_t<
              is_eigen_scalar_same<T, double>::value>>
bool IsEigenOfDouble(const T&) {
  return true;
}

// Ensure that we do not capture non-eigen things.
template <typename Derived>
class NonEigenBase {};
class NonEigen : public NonEigenBase<NonEigen> {
 public:
  typedef double Scalar;
};

GTEST_TEST(EigenTypesTest, TraitsSFINAE) {
  EXPECT_TRUE(IsEigen(MatrixXd()));
  EXPECT_TRUE(IsEigenOfDouble(MatrixXd()));
  EXPECT_TRUE(IsEigen(ArrayXXd()));
  EXPECT_TRUE(IsEigenOfDouble(ArrayXXd()));
  EXPECT_FALSE(IsEigen(NonEigen()));
  EXPECT_FALSE(IsEigenOfDouble(NonEigen()));
  EXPECT_FALSE(IsEigen(1));
  // TODO(eric.cousineau): See if there is a way to short-circuit conditionals.
  // Presently, the following code will throw an error, even in SFINAE.
  // EXPECT_FALSE(IsEigenOfDouble(1));
  // EXPECT_FALSE((is_eigen_vector_of<std::string, double>::value));
}

GTEST_TEST(EigenTypesTest, EigenPtr_Null) {
  EigenPtr<const Matrix3d> null_ptr = nullptr;
  EXPECT_FALSE(null_ptr);
  EXPECT_FALSE(null_ptr != nullptr);
  EXPECT_TRUE(!null_ptr);
  EXPECT_TRUE(null_ptr == nullptr);

  Matrix3d X;
  X.setConstant(0);
  EigenPtr<const Matrix3d> ptr = &X;
  EXPECT_TRUE(ptr);
  EXPECT_TRUE(ptr != nullptr);
  EXPECT_FALSE(!ptr);
  EXPECT_FALSE(ptr == nullptr);
  DRAKE_EXPECT_NO_THROW(*ptr);
}

bool ptr_optional_arg(EigenPtr<MatrixXd> arg = nullptr) {
  return arg;
}

GTEST_TEST(EigenTypesTest, EigenPtr_OptionalArg) {
  EXPECT_FALSE(ptr_optional_arg());
  MatrixXd X(0, 0);
  EXPECT_TRUE(ptr_optional_arg(&X));
}

// Sets M(i, j) = c.
void set(EigenPtr<MatrixXd> M, const int i, const int j, const double c) {
  (*M)(i, j) = c;
}

// Returns M(i, j).
double get(const EigenPtr<const MatrixXd> M, const int i, const int j) {
  return M->coeff(i, j);
}

GTEST_TEST(EigenTypesTest, EigenPtr) {
  Eigen::MatrixXd M1 = Eigen::MatrixXd::Zero(3, 3);
  const Eigen::MatrixXd M2 = Eigen::MatrixXd::Zero(3, 3);

  // Tests set.
  set(&M1, 0, 0, 1);       // Sets M1(0,0) = 1
  EXPECT_EQ(M1(0, 0), 1);  // Checks M1(0, 0) = 1

  // Tests get.
  EXPECT_EQ(get(&M1, 0, 0), 1);  // Checks M1(0, 0) = 1
  EXPECT_EQ(get(&M2, 0, 0), 0);  // Checks M2(0, 0) = 1

  // Shows how to use EigenPtr with .block(). Here we introduce `tmp` to avoid
  // taking the address of temporary object.
  auto tmp = M1.block(1, 1, 2, 2);
  set(&tmp, 0, 0, 1);  // tmp(0, 0) = 1. That is, M1(1, 1) = 1.
  EXPECT_EQ(get(&M1, 1, 1), 1);
}

GTEST_TEST(EigenTypesTest, EigenPtr_EigenRef) {
  Eigen::MatrixXd M = Eigen::MatrixXd::Zero(3, 3);
  Eigen::Ref<Eigen::MatrixXd> M_ref(M);

  // Tests set.
  set(&M_ref, 0, 0, 1);       // Sets M(0,0) = 1
  EXPECT_EQ(M_ref(0, 0), 1);  // Checks M(0, 0) = 1

  // Tests get.
  EXPECT_EQ(get(&M_ref, 0, 0), 1);  // Checks M(0, 0) = 1
}

// EigenPtr_MixMatrixTypes1 and EigenPtr_MixMatrixTypes2 tests check if we can
// mix static-size and dynamic-size matrices with EigenPtr. They only check if
// the code compiles. There are no runtime assertions.
GTEST_TEST(EigenTypesTest, EigenPtr_MixMatrixTypes1) {
  MatrixXd M(3, 3);
  Eigen::Ref<Matrix3d> M_ref(M);         // MatrixXd -> Ref<Matrix3d>
  EigenPtr<Matrix3d> M_ptr(&M);          // MatrixXd -> EigenPtr<Matrix3d>
  EigenPtr<MatrixXd> M_ptr_ref(&M_ref);  // Ref<Matrix3d> -> EigenPtr<MatrixXd>
  EigenPtr<MatrixXd> M_ptr_ref2(M_ptr);  // EigenPtr<MatrixXd> ->
                                         // EigenPtr<Matrix3d>
}

GTEST_TEST(EigenTypesTest, EigenPtr_MixMatrixTypes2) {
  Matrix3d M;
  Eigen::Ref<MatrixXd> M_ref(M);         // Matrix3d -> Ref<MatrixXd>
  EigenPtr<MatrixXd> M_ptr(&M);          // Matrix3d -> EigenPtr<MatrixXd>
  EigenPtr<Matrix3d> M_ptr_ref(&M_ref);  // Ref<MatrixXd> -> EigenPtr<Matrix3d>
  EigenPtr<Matrix3d> M_ptr_ref2(M_ptr);  // EigenPtr<Matrix3d> ->
                                         // EigenPtr<MatrixXd>
}

GTEST_TEST(EigenTypesTest, EigenPtr_Assignment) {
  const double a = 1;
  const double b = 2;
  Matrix3d A;
  A.setConstant(a);
  MatrixXd B(3, 3);
  B.setConstant(b);

  // Ensure that we can use const pointers.
  EigenPtr<const Matrix3d> const_ptr = &A;
  EXPECT_TRUE((const_ptr->array() == a).all());
  const_ptr = &B;
  EXPECT_TRUE((const_ptr->array() == b).all());

  // Test mutable pointers.
  EigenPtr<Matrix3d> mutable_ptr = &A;
  EXPECT_TRUE((mutable_ptr->array() == a).all());
  mutable_ptr = &B;
  EXPECT_TRUE((mutable_ptr->array() == b).all());

  // Ensure we have changed neither A nor B.
  EXPECT_TRUE((A.array() == a).all());
  EXPECT_TRUE((B.array() == b).all());

  // Ensure that we can assign nullptr.
  const_ptr = nullptr;
  EXPECT_FALSE(const_ptr);
  mutable_ptr = nullptr;
  EXPECT_FALSE(mutable_ptr);
}

GTEST_TEST(EigenTypesTest, FixedSizeVector) {
  Vector<double, 2> col;
  EXPECT_EQ(col.rows(), 2);
  EXPECT_EQ(col.cols(), 1);

  RowVector<double, 2> row;
  EXPECT_EQ(row.rows(), 1);
  EXPECT_EQ(row.cols(), 2);
}

}  // namespace
}  // namespace drake

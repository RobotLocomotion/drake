#include "drake/solvers/cost.h"

#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>

#include <gtest/gtest.h>

#include "drake/common/eigen_matrix_compare.h"
#include "drake/common/test/is_dynamic_castable.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/solvers/create_cost.h"
#include "drake/solvers/test/generic_trivial_costs.h"

using std::cout;
using std::endl;
using std::make_shared;
using std::make_unique;
using std::numeric_limits;
using std::runtime_error;
using std::shared_ptr;
using std::unique_ptr;
using std::vector;

using Eigen::Ref;
using drake::Vector1d;
using Eigen::Vector2d;
using Eigen::VectorXd;
using drake::solvers::detail::is_convertible_workaround;
using drake::solvers::test::GenericTrivialCost2;

namespace drake {
namespace solvers {
namespace {

// Check for the failure of libstdc++ 4.9's std::is_convertible if used to
// check From = std::unique_ptr<T>, To = std::shared_ptr<U>
template <typename From, typename To>
struct check_ptr_convertible {
  typedef std::unique_ptr<From> FromPtr;
  typedef std::shared_ptr<To> ToPtr;
  // static constexpr bool std_value =
  //    std::is_convertible<FromPtr, ToPtr>::value;
  static constexpr bool workaround_value =
      is_convertible_workaround<FromPtr, ToPtr>::value;
};

struct A {};
struct B {};
struct C : B {};

GTEST_TEST(testCost, testIsConvertibleWorkaround) {
  EXPECT_TRUE((is_convertible_workaround<C*, B*>::value));
  EXPECT_TRUE((is_convertible_workaround<shared_ptr<C>, shared_ptr<B>>::value));
  EXPECT_TRUE((is_convertible_workaround<unique_ptr<C>, unique_ptr<B>>::value));
  EXPECT_TRUE((is_convertible_workaround<Binding<LinearConstraint>,
                                         Binding<Constraint>>::value));
  EXPECT_FALSE((is_convertible_workaround<Binding<LinearConstraint>,
                                          Binding<Cost>>::value));

  // TODO(eric.cousineau): Determine exact conditions for failure in GCC.
  // Difficult to pinpoint in a multi-platform fashion via __GLIBCXX__ or
  // __GNUC_* version macros
  EXPECT_FALSE((check_ptr_convertible<A, B>::workaround_value));
}

// For a given Constraint, return the equivalent Cost type
template <typename C>
struct related_cost {
  using type = void;
};
template <>
struct related_cost<LinearConstraint> {
  using type = LinearCost;
};
template <>
struct related_cost<QuadraticConstraint> {
  using type = QuadraticCost;
};
template <>
struct related_cost<PolynomialConstraint> {
  using type = PolynomialCost;
};

// Utility to explicitly pass a vector rather than an initializer list.
// This must be done since you cannot forward std::initializer_list's via
// parameter packs. (GCC 4.9.3 is non-standards compliant and permits this,
// but this is fixed in GCC 4.9.4.)
template <typename T>
auto make_vector(std::initializer_list<T> items) {
  return vector<std::decay_t<T>>(items);
}

GTEST_TEST(testCost, testLinearCost) {
  const double tol = numeric_limits<double>::epsilon();

  // Simple ground truth test.
  Eigen::Vector2d a(1, 2);
  const Eigen::Vector2d x0(3, 4);
  const double obj_expected = 11.;

  auto cost = make_shared<LinearCost>(a);
  Eigen::VectorXd y(1);
  cost->Eval(x0, y);
  EXPECT_EQ(y.rows(), 1);
  EXPECT_NEAR(y(0), obj_expected, tol);

  // Update with a constant term.
  const double b = 100;
  cost->UpdateCoefficients(a, b);
  cost->Eval(x0, y);
  EXPECT_NEAR(y(0), obj_expected + b, tol);
  EXPECT_THROW(cost->UpdateCoefficients(Eigen::Vector3d::Ones(), b),
               runtime_error);

  // Reconstruct the same cost with the constant term.
  auto new_cost = make_shared<LinearCost>(a, b);
  new_cost->Eval(x0, y);
  EXPECT_NEAR(y(0), obj_expected + b, tol);
}

GTEST_TEST(testCost, testQuadraticCost) {
  const double tol = numeric_limits<double>::epsilon();

  // Simple ground truth test.
  Eigen::Matrix2d Q;
  Q << 1, 2, 3, 4;
  const Eigen::Vector2d b(5, 6);
  const Eigen::Vector2d x0(7, 8);
  const double obj_expected = 375.5;

  auto cost = make_shared<QuadraticCost>(Q, b);
  Eigen::VectorXd y(1);

  cost->Eval(x0, y);
  EXPECT_EQ(y.rows(), 1);
  EXPECT_NEAR(y(0), obj_expected, tol);

  // Update with a constant term.
  const double c = 100;
  cost->UpdateCoefficients(Q, b, c);
  cost->Eval(x0, y);
  EXPECT_NEAR(y(0), obj_expected + c, tol);

  EXPECT_THROW(cost->UpdateCoefficients(Eigen::Matrix3d::Identity(), b, c),
               runtime_error);
  EXPECT_THROW(cost->UpdateCoefficients(Q, Eigen::Vector3d::Ones(), c),
               runtime_error);

  // Reconstruct the same cost with the constant term.
  auto new_cost = make_shared<QuadraticCost>(Q, b, c);
  new_cost->Eval(x0, y);
  EXPECT_NEAR(y(0), obj_expected + c, tol);
}

// TODO(eric.cousineau): Move QuadraticErrorCost and L2NormCost tests here from
// MathematicalProgram.

template <typename C, typename BoundType, typename... Args>
void VerifyRelatedCost(const Ref<const VectorXd>& x_value, Args&&... args) {
  // Ensure that a constraint constructed in a particular fashion yields
  // equivalent results to its shim, and the related cost.
  const auto inf = std::numeric_limits<double>::infinity();
  auto lb = -BoundType(-inf);
  auto ub = BoundType(inf);
  C constraint(std::forward<Args>(args)..., lb, ub);
  typename related_cost<C>::type cost(std::forward<Args>(args)...);
  VectorXd y_expected, y;
  constraint.Eval(x_value, y);
  cost.Eval(x_value, y_expected);
  EXPECT_TRUE(CompareMatrices(y, y_expected));
}

GTEST_TEST(testCost, testCostShim) {
  // Test CostShim's by means of the related constraints.

  VerifyRelatedCost<LinearConstraint, Vector1d>(Vector1d(2), Vector1d(3));

  VerifyRelatedCost<QuadraticConstraint, double>(Vector1d(2), Vector1d(3),
                                                 Vector1d(4));

  const Polynomiald x("x");
  const auto poly = (x - 1) * (x - 1);
  const auto var_mapping = make_vector({x.GetSimpleVariable()});
  VerifyRelatedCost<PolynomialConstraint, Vector1d>(
      Vector1d(2), VectorXPoly::Constant(1, poly), var_mapping);
}

template <typename T, bool is_dereferencable>
struct deref {
  static const auto& to_const_ref(T&& x) { return x; }
};
template <typename T>
struct deref<T, true> {
  static const auto& to_const_ref(T&& x) { return *x; }
};

template <bool is_dereferencable, typename T>
const auto& to_const_ref(T&& t) {
  using impl = deref<T, is_dereferencable>;
  return impl::to_const_ref(std::forward<T>(t));
}

// Verifies that FunctionCost form can be constructed correctly.
template <bool is_pointer, typename F>
void VerifyFunctionCost(F&& f, const Ref<const VectorXd>& x_value) {
  auto cost = MakeFunctionCost(std::forward<F>(f));
  EXPECT_TRUE(is_dynamic_castable<Cost>(cost));
  // Compare values
  Eigen::VectorXd y_expected(1), y;
  to_const_ref<is_pointer>(f).eval(x_value, y_expected);
  cost->Eval(x_value, y);
  EXPECT_TRUE(CompareMatrices(y, y_expected));
}

GTEST_TEST(testCost, testFunctionCost) {
  // Test that we can construct FunctionCosts with different signatures.
  Eigen::Vector2d x(1, 2);
  VerifyFunctionCost<false>(GenericTrivialCost2(), x);
  // Ensure that we explictly call the default constructor for a const class.
  // @ref http://stackoverflow.com/a/28338123/7829525
  const GenericTrivialCost2 obj_const{};
  VerifyFunctionCost<false>(obj_const, x);
  VerifyFunctionCost<true>(make_shared<GenericTrivialCost2>(), x);
  VerifyFunctionCost<true>(make_unique<GenericTrivialCost2>(), x);
}

}  // anonymous namespace
}  // namespace solvers
}  // namespace drake

#include "drake/solvers/cost.h"

#include <iostream>
#include <memory>

#include <gtest/gtest.h>

#include "drake/common/eigen_matrix_compare.h"
#include "drake/common/test/is_dynamic_castable.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/solvers/create_cost.h"
#include "drake/solvers/decision_variable.h"
#include "drake/solvers/test/generic_trivial_costs.h"

using std::cout;
using std::endl;
using std::make_shared;
using std::make_unique;
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

template <typename C, typename... Args>
void VerifyRelatedCost(const Ref<const VectorXd>& x_value, Args&&... args) {
  // Ensure that a constraint constructed in a particular fashion yields
  // equivalent results to its shim, and the related cost
  C constraint(std::forward<Args>(args)...);
  typename related_cost<C>::type cost(std::forward<Args>(args)...);
  VectorXd y_expected, y;
  constraint.Eval(x_value, y);
  cost.Eval(x_value, y_expected);
  EXPECT_TRUE(CompareMatrices(y, y_expected));
}

GTEST_TEST(testCost, testCostShim) {
  // Test CostShim's by means of the related constraints
  const auto inf = std::numeric_limits<double>::infinity();

  VerifyRelatedCost<LinearConstraint>(Vector1d(2), Vector1d(3),
                                      -Vector1d::Constant(inf),
                                      Vector1d::Constant(inf));

  VerifyRelatedCost<QuadraticConstraint>(Vector1d(2), Vector1d(3), Vector1d(4),
                                         -inf, inf);

  const Polynomiald x("x");
  const auto poly = (x - 1) * (x - 1);
  const auto var_mapping = make_vector({x.GetSimpleVariable()});
  VerifyRelatedCost<PolynomialConstraint>(
      Vector1d(2), VectorXPoly::Constant(1, poly), var_mapping,
      Vector1d::Constant(2), Vector1d::Constant(2));
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

// Verifies that FunctionCost form can be constructed correctly
template <bool is_pointer, typename F>
void VerifyFunctionCost(F&& f, const Ref<const VectorXd>& x_value) {
  auto cost = CreateFunctionCost(std::forward<F>(f));
  EXPECT_TRUE(is_dynamic_castable<Cost>(cost));
  // TODO(eric.cousineau): Remove when shim is removed
  EXPECT_TRUE(is_dynamic_castable<Constraint>(cost));
  // Compare values
  Eigen::VectorXd y_expected(1), y;
  to_const_ref<is_pointer>(f).eval(x_value, y_expected);
  cost->Eval(x_value, y);
  EXPECT_TRUE(CompareMatrices(y, y_expected));
}

GTEST_TEST(testCost, testFunctionCost) {
  // Test that we can construct FunctionCosts with different signatures
  Eigen::Vector2d x(1, 2);
  VerifyFunctionCost<false>(GenericTrivialCost2(), x);
  // Ensure that we explictly call the default constructor for a const class
  // @ref http://stackoverflow.com/a/28338123/7829525
  const GenericTrivialCost2 obj_const {};
  VerifyFunctionCost<false>(obj_const, x);
  VerifyFunctionCost<true>(make_shared<GenericTrivialCost2>(), x);
  VerifyFunctionCost<true>(make_unique<GenericTrivialCost2>(), x);
}

}  // anonymous namespace
}  // namespace solvers
}  // namespace drake

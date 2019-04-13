#include <vector>
#include <utility>

#include <gtest/gtest.h>

#include "drake/common/symbolic.h"
#include "drake/common/test_utilities/symbolic_test_util.h"

using std::vector;

namespace drake {
namespace symbolic {
namespace {

using test::ExprEqual;
using test::ExprLess;
using test::ExprNotEqual;
using test::ExprNotLess;
using test::FormulaEqual;

// Provides common variables that are used by the following tests.
class SymbolicExpressionTest : public ::testing::Test {
 protected:
  const Variable var_a_{"a"};
  const Variable var_x_{"x"};
  const Variable var_y_{"y"};
  const Variable var_z_{"z"};
  const Expression a_{var_a_};
  const Expression x_{var_x_};
  const Expression y_{var_y_};
  const Expression z_{var_z_};
  const Expression x_plus_y_{x_ + y_};
  const Expression x_plus_z_{x_ + z_};

  const Expression zero_{0.0};
  const Expression one_{1.0};
  const Expression two_{2.0};
  const Expression neg_one_{-1.0};
  const Expression pi_{M_PI};
  const Expression neg_pi_{-M_PI};
  const Expression e_{M_E};

  const Expression c1_{-10.0};
  const Expression c2_{1.0};
  const Expression c3_{3.14159};
  const Expression c4_{-2.718};

  const Expression e_constant_{1.0};
  const Expression e_var_{var_x_};
  const Expression e_add_{x_ + y_};
  const Expression e_neg_{-x_};  // -1 * x_
  const Expression e_mul_{x_ * y_};
  const Expression e_div_{x_ / y_};
  const Expression e_log_{log(x_)};
  const Expression e_abs_{abs(x_)};
  const Expression e_exp_{exp(x_)};
  const Expression e_sqrt_{sqrt(x_)};
  const Expression e_pow_{pow(x_, y_)};
  const Expression e_sin_{sin(x_)};
  const Expression e_cos_{cos(x_)};
  const Expression e_tan_{tan(x_)};
  const Expression e_asin_{asin(x_)};
  const Expression e_acos_{acos(x_)};
  const Expression e_atan_{atan(x_)};
  const Expression e_atan2_{atan2(x_, y_)};
  const Expression e_sinh_{sinh(x_)};
  const Expression e_cosh_{cosh(x_)};
  const Expression e_tanh_{tanh(x_)};
  const Expression e_min_{min(x_, y_)};
  const Expression e_max_{max(x_, y_)};
  const Expression e_ceil_{ceil(x_)};
  const Expression e_floor_{floor(x_)};
  const Expression e_ite_{if_then_else(x_ < y_, x_, y_)};
  const Expression e_nan_{Expression::NaN()};
  const Expression e_uf_{uninterpreted_function("uf", {var_x_, var_y_})};

  const vector<Expression> collection_{
      e_constant_, e_var_,  e_add_,  e_neg_,   e_mul_,  e_div_,  e_log_,
      e_abs_,      e_exp_,  e_sqrt_, e_pow_,   e_sin_,  e_cos_,  e_tan_,
      e_asin_,     e_acos_, e_atan_, e_atan2_, e_sinh_, e_cosh_, e_tanh_,
      e_min_,      e_max_,  e_ceil_, e_floor_, e_ite_,  e_nan_,  e_uf_};
};

namespace internal {
using Func = std::function<void(void)>;
class iff;
class iff_then final {
 public:
  iff_then(iff* parent) : parent_(parent) {}
  void operator-(Func&& else_func);
 private:
  iff* parent_{};
};
class iff final {
 public:
  template <typename... Args>
  iff(bool b, Args...) : b_{b} {}
  template <typename... Args>
  iff(Formula&& f, Args... e_phi_ptr) : f_{&f}, e_phi_{*e_phi_ptr...} {
  }
  ~iff() {
    if (!f_) { return; }
    DRAKE_THROW_UNLESS(then_func_);
    const size_t count = e_phi_.size();
    // Save the originals.
    std::vector<Expression> e_orig;
    for (size_t i = 0; i < count; ++i) { e_orig.push_back(e_phi_[i]); }
    // Compute the then-values.
    (*then_func_)();
    std::vector<Expression> e_then;
    for (size_t i = 0; i < count; ++i) { e_then.push_back(e_phi_[i]); }
    // Compute the else-values.
    std::vector<Expression> e_else;
    if (else_func_) {
      for (size_t i = 0; i < count; ++i) { e_phi_[i].get() = e_orig[i]; }
      (*else_func_)();
      for (size_t i = 0; i < count; ++i) { e_else.push_back(e_phi_[i]); }
    } else {
      e_else = e_orig;
    }
    // Write back the final phi values.
    for (size_t i = 0; i < count; ++i) {
      e_phi_[i].get() = if_then_else(*f_, e_then[i], e_else[i]);
    }
  }
  iff_then operator+(Func&& then_func) {
    if (f_) {
      then_func_ = &then_func;
      return iff_then(this);
    } else if (b_) {
      then_func();
      return iff_then(nullptr);
    } else {
      return iff_then(this);
    }
  }
  bool b_;
  Formula* f_{};
  std::vector<std::reference_wrapper<Expression>> e_phi_;
  Func* then_func_{};
  Func* else_func_{};
};
void iff_then::operator-(Func&& else_func) {
  if (parent_) {
    if (parent_->f_) {
      parent_->else_func_ = &else_func;
    } else {
      else_func();
    }
  }
}
}  // namespace internal

using iff = internal::iff;

template <typename T>
std::pair<T, T> min_max(const T& a, const T& b) {
  T less;
  T more;
  symbolic::iff(a < b, &less, &more) +[&]() {
    less = a;
    more = b;
  } /* else */ -[&]() {
    less = b;
    more = a;
  };
  return std::make_pair(less, more);
}

TEST_F(SymbolicExpressionTest, MinMax) {
  auto result = min_max(x_, y_);
  EXPECT_PRED2(ExprEqual, result.first, if_then_else(x_ < y_, x_, y_));
  EXPECT_PRED2(ExprEqual, result.second, if_then_else(x_ < y_, y_, x_));
}

GTEST_TEST(DoubleTest, ShortCircuit) {
  using T = double;
  T a{0};
  symbolic::iff(a >= 0, &a) +[&]() {
    --a;
  } /* else */ -[&]() {
    DRAKE_UNREACHABLE();
  };
  EXPECT_EQ(a, -1.0);
}

}  // namespace
}  // namespace symbolic
}  // namespace drake

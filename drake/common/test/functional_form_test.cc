#include "drake/common/functional_form.h"

#include <cmath>
#include <sstream>

#include <Eigen/Core>

#include "gtest/gtest.h"

namespace drake {
namespace core {
namespace test {
namespace {

using Vars = FunctionalForm::Variables;

GTEST_TEST(FunctionalFormVariableTest, NilVariant) {
  using Variable = FunctionalForm::Variable;
  Variable const v_nil;
  EXPECT_TRUE(v_nil.is_nil());
  EXPECT_NE(v_nil.index(), 0u);

  // copy ctor
  Variable v(v_nil);
  EXPECT_EQ(v, v_nil);

  // move ctor
  {
    Variable v2(std::move(v));
    EXPECT_EQ(v2, v_nil);
    EXPECT_EQ(v, v_nil);
  }

  // copy assign
  v = v_nil;
  EXPECT_EQ(v, v_nil);

  // move assign
  {
    Variable v2;
    v2 = std::move(v);
    EXPECT_EQ(v2, v_nil);
    EXPECT_EQ(v, v_nil);
  }

  // comparison
  EXPECT_TRUE(v_nil == v_nil);
  EXPECT_FALSE(v_nil != v_nil);
  EXPECT_FALSE(v_nil < v_nil);

  // stream
  {
    std::ostringstream ss;
    ss << v_nil;
    EXPECT_EQ(ss.str(), "");
  }
}

GTEST_TEST(FunctionalFormVariableTest, IndexVariant) {
  using Variable = FunctionalForm::Variable;
  Variable const v_nil;
  Variable const v_index = 0;
  EXPECT_TRUE(v_nil.is_nil());
  EXPECT_TRUE(v_index.is_index());

  // ctor
  {
    std::size_t index = 0;
    Variable v1(index);
    Variable v2(std::move(index));
    std::vector<Variable> vec({{0}});
    EXPECT_EQ(v1, v2);
    EXPECT_EQ(v1, vec[0]);
  }

  // copy ctor
  Variable v(v_index);
  EXPECT_EQ(v, v_index);

  // move ctor
  {
    Variable v2(std::move(v));
    EXPECT_EQ(v2, v_index);
    EXPECT_EQ(v, v_nil);
  }

  // copy assign
  v = v_index;
  EXPECT_EQ(v, v_index);

  // move assign
  {
    Variable v2;
    v2 = std::move(v);
    EXPECT_EQ(v2, v_index);
    EXPECT_EQ(v, v_nil);
  }

  // move assign from conversion temporary
  v = 123;
  EXPECT_TRUE(v.is_index());
  EXPECT_EQ(v.index(), 123u);

  // comparison
  EXPECT_TRUE(v_index == v_index);
  EXPECT_TRUE(v_index <= v_index);
  EXPECT_TRUE(v_index >= v_index);
  EXPECT_FALSE(v_index != v_index);
  EXPECT_FALSE(v_index < v_index);
  EXPECT_FALSE(v_index < v_index);
  EXPECT_FALSE(v_index == v_nil);
  EXPECT_TRUE(v_nil < v_index);
  EXPECT_TRUE(v_nil <= v_index);
  EXPECT_TRUE(v_index > v_nil);
  EXPECT_TRUE(v_index >= v_nil);

  // stream
  {
    std::ostringstream ss;
    ss << v_index;
    EXPECT_EQ(ss.str(), "0");
  }
}

GTEST_TEST(FunctionalFormVariableTest, NamedVariant) {
  using Variable = FunctionalForm::Variable;
  Variable const v_nil;
  Variable const v_named = "x";
  Variable const v_index = 0;
  EXPECT_TRUE(v_nil.is_nil());
  EXPECT_TRUE(v_named.is_named());
  EXPECT_TRUE(v_index.is_index());

  // ctor
  {
    std::string name = "x";
    Variable v1(name);
    Variable v2(std::move(name));
    std::vector<Variable> vec({"x"});
    EXPECT_EQ(v1, v2);
    EXPECT_EQ(v1, vec[0]);
  }

  // copy ctor
  Variable v(v_named);
  EXPECT_EQ(v, v_named);

  // move ctor
  {
    Variable v2(std::move(v));
    EXPECT_EQ(v2, v_named);
    EXPECT_EQ(v, v_nil);
  }

  // copy assign
  v = v_named;
  EXPECT_EQ(v, v_named);

  // move assign
  {
    Variable v2;
    v2 = std::move(v);
    EXPECT_EQ(v2, v_named);
    EXPECT_EQ(v, v_nil);
  }

  // move assign from conversion temporary
  v = "x";
  EXPECT_TRUE(v.is_named());
  EXPECT_EQ(v.name(), "x");
  v = std::string("y");
  EXPECT_TRUE(v.is_named());
  EXPECT_EQ(v.name(), "y");

  // comparison
  EXPECT_TRUE(v_named == v_named);
  EXPECT_FALSE(v_named != v_named);
  EXPECT_FALSE(v_named < v_named);
  EXPECT_FALSE(v_named == v_nil);
  EXPECT_TRUE(v_nil < v_named);
  EXPECT_TRUE(v_index < v_named);

  // stream
  {
    std::ostringstream ss;
    ss << v_named;
    EXPECT_EQ(ss.str(), "x");
  }
}

GTEST_TEST(FunctionalFormVariablesTest, Basic) {
  FunctionalForm::Variables v_default;
  FunctionalForm::Variables v_init_empty({});
  EXPECT_TRUE(v_default.empty());
  EXPECT_EQ(v_default.begin(), v_default.end());
  EXPECT_EQ(v_default, v_init_empty);

  FunctionalForm::Variables a({"a1", "a2", 3});
  {
    ASSERT_EQ(a.size(), 3u);
    FunctionalForm::Variables::const_iterator i = a.begin();
    EXPECT_EQ(*i, FunctionalForm::Variable(3));
    ++i;
    EXPECT_EQ(*i, FunctionalForm::Variable("a1"));
    ++i;
    EXPECT_EQ(*i, FunctionalForm::Variable("a2"));
    ++i;
    EXPECT_EQ(i, a.end());
  }

  FunctionalForm::Variables b({"b1", "b2", 3});
  {
    ASSERT_EQ(b.size(), 3u);
    FunctionalForm::Variables::const_iterator i = b.begin();
    EXPECT_EQ(*i, FunctionalForm::Variable(3));
    ++i;
    EXPECT_EQ(*i, FunctionalForm::Variable("b1"));
    ++i;
    EXPECT_EQ(*i, FunctionalForm::Variable("b2"));
    ++i;
    EXPECT_EQ(i, b.end());
  }

  EXPECT_NE(a, b);

  FunctionalForm::Variables u = FunctionalForm::Variables::Union(a, b);
  {
    ASSERT_EQ(u.size(), 5u);
    FunctionalForm::Variables::const_iterator i = u.begin();
    EXPECT_EQ(*i, FunctionalForm::Variable(3));
    ++i;
    EXPECT_EQ(*i, FunctionalForm::Variable("a1"));
    ++i;
    EXPECT_EQ(*i, FunctionalForm::Variable("a2"));
    ++i;
    EXPECT_EQ(*i, FunctionalForm::Variable("b1"));
    ++i;
    EXPECT_EQ(*i, FunctionalForm::Variable("b2"));
    ++i;
    EXPECT_EQ(i, u.end());
  }

  // Copy assignment.
  a = u;
  EXPECT_EQ(a, u);

  // Move assignment.
  b = std::move(u);
  EXPECT_EQ(b, a);
  EXPECT_EQ(u, v_default);

  // Copy construction.
  FunctionalForm::Variables ca = a;
  EXPECT_EQ(ca, a);
  EXPECT_EQ(a, b);

  // Move construction.
  FunctionalForm::Variables cb = std::move(b);
  EXPECT_EQ(cb, ca);
  EXPECT_EQ(b, v_default);
}

GTEST_TEST(FunctionalFormTest, Construct) {
  FunctionalForm default_constructed;
  EXPECT_TRUE(default_constructed.IsUndefined());

  FunctionalForm double_zero(0.0);
  EXPECT_TRUE(double_zero.IsZero());

  FunctionalForm double_nonzero(0.1);
  EXPECT_TRUE(double_nonzero.IsConstant());

  FunctionalForm double_nan(std::nan(""));
  EXPECT_TRUE(double_nan.IsUndefined());
}

GTEST_TEST(FunctionalFormTest, Basic) {
  FunctionalForm zero = FunctionalForm::Zero();
  FunctionalForm cons = FunctionalForm::Constant();
  FunctionalForm lin = FunctionalForm::Linear({"x"});
  FunctionalForm aff = FunctionalForm::Affine({"x"});
  FunctionalForm poly = FunctionalForm::Polynomial({"x"});
  FunctionalForm diff = FunctionalForm::Differentiable({"x"});
  FunctionalForm arb = FunctionalForm::Arbitrary({"x"});
  FunctionalForm undf = FunctionalForm::Undefined({"x"});

  EXPECT_TRUE(zero.IsZero());
  EXPECT_TRUE(cons.IsConstant());
  EXPECT_TRUE(lin.IsLinear());
  EXPECT_TRUE(aff.IsAffine());
  EXPECT_TRUE(poly.IsPolynomial());
  EXPECT_TRUE(diff.IsDifferentiable());
  EXPECT_TRUE(arb.IsArbitrary());
  EXPECT_TRUE(undf.IsUndefined());

  EXPECT_FALSE(zero.IsConstant());
  EXPECT_FALSE(cons.IsLinear());
  EXPECT_FALSE(lin.IsAffine());
  EXPECT_FALSE(aff.IsPolynomial());
  EXPECT_FALSE(poly.IsDifferentiable());
  EXPECT_FALSE(diff.IsArbitrary());
  EXPECT_FALSE(arb.IsUndefined());
  EXPECT_FALSE(undf.IsZero());
}

GTEST_TEST(FunctionalFormTest, Equality) {
  FunctionalForm zero = FunctionalForm::Zero();
  FunctionalForm cons = FunctionalForm::Constant();
  FunctionalForm lin = FunctionalForm::Linear({"x"});

  EXPECT_TRUE(zero.Is(FunctionalForm::Zero()));
  EXPECT_TRUE(cons.Is(FunctionalForm::Constant()));
  EXPECT_TRUE(lin.Is(FunctionalForm::Linear({"x"})));
  EXPECT_TRUE(!lin.Is(FunctionalForm::Linear({0})));
}

GTEST_TEST(FunctionalFormTest, Stream) {
  {
    std::ostringstream ss;
    ss << FunctionalForm::Zero();
    EXPECT_EQ(ss.str(), "zero");
  }

  {
    std::ostringstream ss;
    ss << FunctionalForm::Constant();
    EXPECT_EQ(ss.str(), "cons");
  }

  {
    std::ostringstream ss;
    ss << FunctionalForm::Linear({"x", "y", "z"});
    EXPECT_EQ(ss.str(), "lin(x,y,z)");
  }

  {
    std::ostringstream ss;
    ss << FunctionalForm::Affine({4, 3, 2, 1});
    EXPECT_EQ(ss.str(), "aff(1,2,3,4)");
  }

  {
    std::ostringstream ss;
    ss << FunctionalForm::Polynomial({"x", "y", "z"});
    EXPECT_EQ(ss.str(), "poly(x,y,z)");
  }

  {
    std::ostringstream ss;
    ss << FunctionalForm::Differentiable({"x", "y", "z"});
    EXPECT_EQ(ss.str(), "diff(x,y,z)");
  }

  {
    std::ostringstream ss;
    ss << FunctionalForm::Arbitrary({"x", 0});
    EXPECT_EQ(ss.str(), "arb(0,x)");
  }

  {
    std::ostringstream ss;
    ss << FunctionalForm::Undefined({});
    EXPECT_EQ(ss.str(), "undf");
  }

  {
    std::ostringstream ss;
    ss << FunctionalForm::Undefined({"x", 0});
    EXPECT_EQ(ss.str(), "undf(0,x)");
  }
}

GTEST_TEST(FunctionalFormTest, Add) {
  {
    FunctionalForm f = FunctionalForm::Zero() + FunctionalForm::Linear({"x"});
    EXPECT_TRUE(f.IsLinear());
    EXPECT_EQ(f.GetVariables(), Vars({"x"}));
  }

  {
    FunctionalForm f =
        FunctionalForm::Linear({"x"}) + FunctionalForm::Linear({"y"});
    EXPECT_TRUE(f.IsLinear());
    EXPECT_EQ(f.GetVariables(), Vars({"x", "y"}));
  }

  {
    FunctionalForm f =
        FunctionalForm::Constant() + FunctionalForm::Linear({"x"});
    EXPECT_TRUE(f.IsAffine());
    EXPECT_EQ(f.GetVariables(), Vars({"x"}));
  }

  {
    FunctionalForm f = 1 + FunctionalForm::Linear({"x"});
    EXPECT_TRUE(f.IsAffine());
    EXPECT_EQ(f.GetVariables(), Vars({"x"}));
  }

  {
    FunctionalForm f = FunctionalForm::Linear({"x"}) + 1;
    EXPECT_TRUE(f.IsAffine());
    EXPECT_EQ(f.GetVariables(), Vars({"x"}));
  }

  {
    FunctionalForm f = FunctionalForm::Linear({"x"});
    f += 1;
    EXPECT_TRUE(f.IsAffine());
    EXPECT_EQ(f.GetVariables(), Vars({"x"}));
  }

  {
    FunctionalForm f = FunctionalForm::Linear({"x"});
    f += FunctionalForm::Linear({"y"});
    EXPECT_TRUE(f.IsLinear());
    EXPECT_EQ(f.GetVariables(), Vars({"x", "y"}));
  }
}

GTEST_TEST(FunctionalFormTest, Subtract) {
  {
    FunctionalForm f = FunctionalForm::Zero() - FunctionalForm::Linear({"x"});
    EXPECT_TRUE(f.IsLinear());
    EXPECT_EQ(f.GetVariables(), Vars({"x"}));
  }

  {
    FunctionalForm f =
        FunctionalForm::Linear({"x"}) - FunctionalForm::Linear({"y"});
    EXPECT_TRUE(f.IsLinear());
    EXPECT_EQ(f.GetVariables(), Vars({"x", "y"}));
  }

  {
    FunctionalForm f =
        FunctionalForm::Constant() - FunctionalForm::Linear({"x"});
    EXPECT_TRUE(f.IsAffine());
    EXPECT_EQ(f.GetVariables(), Vars({"x"}));
  }

  {
    FunctionalForm f = 1 - FunctionalForm::Linear({"x"});
    EXPECT_TRUE(f.IsAffine());
    EXPECT_EQ(f.GetVariables(), Vars({"x"}));
  }

  {
    FunctionalForm f = FunctionalForm::Linear({"x"}) - 1;
    EXPECT_TRUE(f.IsAffine());
    EXPECT_EQ(f.GetVariables(), Vars({"x"}));
  }

  {
    FunctionalForm f = FunctionalForm::Linear({"x"});
    f -= 1;
    EXPECT_TRUE(f.IsAffine());
    EXPECT_EQ(f.GetVariables(), Vars({"x"}));
  }

  {
    FunctionalForm f = FunctionalForm::Linear({"x"});
    f -= FunctionalForm::Linear({"y"});
    EXPECT_TRUE(f.IsLinear());
    EXPECT_EQ(f.GetVariables(), Vars({"x", "y"}));
  }
}

GTEST_TEST(FunctionalFormTest, Multiply) {
  {
    FunctionalForm f = FunctionalForm::Zero() * FunctionalForm::Linear({"x"});
    EXPECT_TRUE(f.IsZero());
    EXPECT_EQ(f.GetVariables(), Vars());
  }

  {
    FunctionalForm f =
        FunctionalForm::Linear({"x"}) * FunctionalForm::Linear({"y"});
    EXPECT_TRUE(f.IsPolynomial());
    EXPECT_EQ(f.GetVariables(), Vars({"x", "y"}));
  }

  {
    FunctionalForm f =
        FunctionalForm::Constant() * FunctionalForm::Linear({"x"});
    EXPECT_TRUE(f.IsLinear());
    EXPECT_EQ(f.GetVariables(), Vars({"x"}));
  }

  {
    FunctionalForm f = 1 * FunctionalForm::Linear({"x"});
    EXPECT_TRUE(f.IsLinear());
    EXPECT_EQ(f.GetVariables(), Vars({"x"}));
  }

  {
    FunctionalForm f = FunctionalForm::Linear({"x"}) * 1;
    EXPECT_TRUE(f.IsLinear());
    EXPECT_EQ(f.GetVariables(), Vars({"x"}));
  }

  {
    FunctionalForm f = FunctionalForm::Linear({"x"});
    f *= 1;
    EXPECT_TRUE(f.IsLinear());
    EXPECT_EQ(f.GetVariables(), Vars({"x"}));
  }

  {
    FunctionalForm f = FunctionalForm::Linear({"x"});
    f *= FunctionalForm::Linear({"y"});
    EXPECT_TRUE(f.IsPolynomial());
    EXPECT_EQ(f.GetVariables(), Vars({"x", "y"}));
  }
}

GTEST_TEST(FunctionalFormTest, Divide) {
  {
    FunctionalForm f = FunctionalForm::Zero() / FunctionalForm::Linear({"x"});
    EXPECT_TRUE(f.IsZero());
    EXPECT_EQ(f.GetVariables(), Vars());
  }

  {
    FunctionalForm f =
        FunctionalForm::Linear({"x"}) / FunctionalForm::Linear({"y"});
    EXPECT_TRUE(f.IsDifferentiable());
    EXPECT_EQ(f.GetVariables(), Vars({"x", "y"}));
  }

  {
    FunctionalForm f =
        FunctionalForm::Constant() / FunctionalForm::Linear({"x"});
    EXPECT_TRUE(f.IsDifferentiable());
    EXPECT_EQ(f.GetVariables(), Vars({"x"}));
  }

  {
    FunctionalForm f = 1 / FunctionalForm::Linear({"x"});
    EXPECT_TRUE(f.IsDifferentiable());
    EXPECT_EQ(f.GetVariables(), Vars({"x"}));
  }

  {
    FunctionalForm f = 1 / FunctionalForm::Zero();
    EXPECT_TRUE(f.IsUndefined());
    EXPECT_EQ(f.GetVariables(), Vars());
  }

  {
    FunctionalForm f = FunctionalForm::Linear({"x"}) / 1;
    EXPECT_TRUE(f.IsLinear());
    EXPECT_EQ(f.GetVariables(), Vars({"x"}));
  }

  {
    FunctionalForm f = FunctionalForm::Linear({"x"});
    f /= 1;
    EXPECT_TRUE(f.IsLinear());
    EXPECT_EQ(f.GetVariables(), Vars({"x"}));
  }

  {
    FunctionalForm f = FunctionalForm::Linear({"x"});
    f /= FunctionalForm::Linear({"y"});
    EXPECT_TRUE(f.IsDifferentiable());
    EXPECT_EQ(f.GetVariables(), Vars({"x", "y"}));
  }
}

GTEST_TEST(FunctionalFormTest, UnaryFunctions) {
  {
    FunctionalForm f = abs(FunctionalForm::Linear({"x"}));
    EXPECT_TRUE(f.IsDifferentiable());
    EXPECT_EQ(f.GetVariables(), Vars({"x"}));
  }

  {
    FunctionalForm f = cos(FunctionalForm::Linear({"x"}));
    EXPECT_TRUE(f.IsDifferentiable());
    EXPECT_EQ(f.GetVariables(), Vars({"x"}));
  }

  {
    FunctionalForm f = exp(FunctionalForm::Linear({"x"}));
    EXPECT_TRUE(f.IsDifferentiable());
    EXPECT_EQ(f.GetVariables(), Vars({"x"}));
  }

  {
    FunctionalForm f = log(FunctionalForm::Linear({"x"}));
    EXPECT_TRUE(f.IsDifferentiable());
    EXPECT_EQ(f.GetVariables(), Vars({"x"}));
  }

  {
    FunctionalForm f = log(FunctionalForm::Zero());
    EXPECT_TRUE(f.IsUndefined());
    EXPECT_EQ(f.GetVariables(), Vars());
  }

  {
    FunctionalForm f = sin(FunctionalForm::Linear({"x"}));
    EXPECT_TRUE(f.IsDifferentiable());
    EXPECT_EQ(f.GetVariables(), Vars({"x"}));
  }

  {
    FunctionalForm f = sqrt(FunctionalForm::Linear({"x"}));
    EXPECT_TRUE(f.IsDifferentiable());
    EXPECT_EQ(f.GetVariables(), Vars({"x"}));
  }
}

GTEST_TEST(FunctionalFormTest, BinaryFunctions) {
  {
    FunctionalForm f = max(FunctionalForm::Linear({"x"}),
                           FunctionalForm::Linear({"y"}));
    EXPECT_TRUE(f.IsDifferentiable());
    EXPECT_EQ(f.GetVariables(), Vars({"x", "y"}));
  }

  {
    FunctionalForm f = max(FunctionalForm::Linear({"x"}), 1);
    EXPECT_TRUE(f.IsDifferentiable());
    EXPECT_EQ(f.GetVariables(), Vars({"x"}));
  }

  {
    FunctionalForm f = max(1, FunctionalForm::Linear({"x"}));
    EXPECT_TRUE(f.IsDifferentiable());
    EXPECT_EQ(f.GetVariables(), Vars({"x"}));
  }

  {
    FunctionalForm f = min(FunctionalForm::Linear({"x"}),
                           FunctionalForm::Linear({"y"}));
    EXPECT_TRUE(f.IsDifferentiable());
    EXPECT_EQ(f.GetVariables(), Vars({"x", "y"}));
  }

  {
    FunctionalForm f = min(FunctionalForm::Linear({"x"}), 1);
    EXPECT_TRUE(f.IsDifferentiable());
    EXPECT_EQ(f.GetVariables(), Vars({"x"}));
  }

  {
    FunctionalForm f = min(1, FunctionalForm::Linear({"x"}));
    EXPECT_TRUE(f.IsDifferentiable());
    EXPECT_EQ(f.GetVariables(), Vars({"x"}));
  }
}

class FunctionalFormMatrixTest : public ::testing::Test {
 protected:
  using FF = FunctionalForm;

  Eigen::Matrix<FunctionalForm, 3, 2> A;
  Eigen::Matrix<FunctionalForm, 2, 3> B;
  Eigen::Matrix<double, 3, 2> C;

  template <typename Derived>
  Eigen::Ref<typename Derived::PlainObject const> ref(
      Eigen::MatrixBase<Derived> const& m) const {
    return m;
  }

  template <typename Derived>
  Eigen::Matrix<typename Derived::Scalar, Eigen::Dynamic, Eigen::Dynamic> dyn(
      Eigen::MatrixBase<Derived> const& m) const {
    return m;
  }

  void SetUp() {
    // clang-format off
    A <<
        FF::Linear({"a11"}), FF::Constant(),
        FF::Linear({"a21"}), FF::Polynomial({"a22"}),
        FF::Linear({"a31"}), FF::Linear({"a32"});
    B <<
        FF::Constant(),      FF::Constant(), FF::Zero(),
        FF::Linear({"b21"}), FF::Constant(), FF::Linear({"b23"});
    // clang-format on
    C.setConstant(1);
  }

  void CheckAPlusBTranspose(Eigen::Matrix<FunctionalForm, 3, 2> const& f) {
    EXPECT_TRUE(f(0, 0).Is(FF::Affine({"a11"})));
    EXPECT_TRUE(f(0, 1).Is(FF::Affine({"b21"})));
    EXPECT_TRUE(f(1, 0).Is(FF::Affine({"a21"})));
    EXPECT_TRUE(f(1, 1).Is(FF::Polynomial({"a22"})));
    EXPECT_TRUE(f(2, 0).Is(FF::Linear({"a31"})));
    EXPECT_TRUE(f(2, 1).Is(FF::Linear({"a32", "b23"})));
  }

  void CheckAPlusC(Eigen::Matrix<FunctionalForm, 3, 2> const& f) {
    EXPECT_TRUE(f(0, 0).Is(FF::Affine({"a11"})));
    EXPECT_TRUE(f(0, 1).Is(FF::Constant()));
    EXPECT_TRUE(f(1, 0).Is(FF::Affine({"a21"})));
    EXPECT_TRUE(f(1, 1).Is(FF::Polynomial({"a22"})));
    EXPECT_TRUE(f(2, 0).Is(FF::Affine({"a31"})));
    EXPECT_TRUE(f(2, 1).Is(FF::Affine({"a32"})));
  }

  void CheckATimesScalar(Eigen::Matrix<FunctionalForm, 3, 2> const& f) {
    EXPECT_TRUE(f(0, 0).Is(FF::Linear({"a11"})));
    EXPECT_TRUE(f(0, 1).Is(FF::Constant()));
    EXPECT_TRUE(f(1, 0).Is(FF::Linear({"a21"})));
    EXPECT_TRUE(f(1, 1).Is(FF::Polynomial({"a22"})));
    EXPECT_TRUE(f(2, 0).Is(FF::Linear({"a31"})));
    EXPECT_TRUE(f(2, 1).Is(FF::Linear({"a32"})));
  }

  void CheckATimesB(Eigen::Matrix<FunctionalForm, 3, 3> const& f) {
    EXPECT_TRUE(f(0, 0).Is(FF::Linear({"a11", "b21"})));
    EXPECT_TRUE(f(0, 1).Is(FF::Affine({"a11"})));
    EXPECT_TRUE(f(0, 2).Is(FF::Linear({"b23"})));
    EXPECT_TRUE(f(1, 0).Is(FF::Polynomial({"a21", "a22", "b21"})));
    EXPECT_TRUE(f(1, 1).Is(FF::Polynomial({"a21", "a22"})));
    EXPECT_TRUE(f(1, 2).Is(FF::Polynomial({"a22", "b23"})));
    EXPECT_TRUE(f(2, 0).Is(FF::Polynomial({"a31", "a32", "b21"})));
    EXPECT_TRUE(f(2, 1).Is(FF::Linear({"a31", "a32"})));
    EXPECT_TRUE(f(2, 2).Is(FF::Polynomial({"a32", "b23"})));
  }

  void CheckATimesCTranspose(Eigen::Matrix<FunctionalForm, 3, 3> const& f) {
    EXPECT_TRUE(f(0, 0).Is(FF::Affine({"a11"})));
    EXPECT_TRUE(f(0, 1).Is(FF::Affine({"a11"})));
    EXPECT_TRUE(f(0, 2).Is(FF::Affine({"a11"})));
    EXPECT_TRUE(f(1, 0).Is(FF::Polynomial({"a21", "a22"})));
    EXPECT_TRUE(f(1, 1).Is(FF::Polynomial({"a21", "a22"})));
    EXPECT_TRUE(f(1, 2).Is(FF::Polynomial({"a21", "a22"})));
    EXPECT_TRUE(f(2, 0).Is(FF::Linear({"a31", "a32"})));
    EXPECT_TRUE(f(2, 1).Is(FF::Linear({"a31", "a32"})));
    EXPECT_TRUE(f(2, 2).Is(FF::Linear({"a31", "a32"})));
  }

  void CheckCTimesATranspose(Eigen::Matrix<FunctionalForm, 2, 2> const& f) {
    EXPECT_TRUE(f(0, 0).Is(FF::Linear({"a11", "a21", "a31"})));
    EXPECT_TRUE(f(0, 1).Is(FF::Polynomial({"a22", "a32"})));
    EXPECT_TRUE(f(1, 0).Is(FF::Linear({"a11", "a21", "a31"})));
    EXPECT_TRUE(f(1, 1).Is(FF::Polynomial({"a22", "a32"})));
  }

  void CheckADivScalar(Eigen::Matrix<FunctionalForm, 3, 2> const& f) {
    EXPECT_TRUE(f(0, 0).Is(FF::Linear({"a11"})));
    EXPECT_TRUE(f(0, 1).Is(FF::Constant()));
    EXPECT_TRUE(f(1, 0).Is(FF::Linear({"a21"})));
    EXPECT_TRUE(f(1, 1).Is(FF::Polynomial({"a22"})));
    EXPECT_TRUE(f(2, 0).Is(FF::Linear({"a31"})));
    EXPECT_TRUE(f(2, 1).Is(FF::Linear({"a32"})));
  }

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

TEST_F(FunctionalFormMatrixTest, Constant) {
  Eigen::Matrix<FunctionalForm, 3, 3> M;
  M.setConstant(FunctionalForm::Constant());
  EXPECT_TRUE(M.isConstant(FunctionalForm::Constant()));
  EXPECT_TRUE(M.transpose().isConstant(FunctionalForm::Constant()));
}

TEST_F(FunctionalFormMatrixTest, Add) {
  CheckAPlusBTranspose(A + B.transpose());
  CheckAPlusBTranspose(ref(A) + ref(B.transpose()));
  CheckAPlusC(A + C);
  CheckAPlusC(ref(A) + C);
  CheckAPlusC(dyn(A) + C);
  CheckAPlusC(A + ref(C));
  CheckAPlusC(A + dyn(C));
  CheckAPlusC(ref(A) + ref(C));
  CheckAPlusC(dyn(A) + dyn(C));
  CheckAPlusC(C + A);
  CheckAPlusC(ref(C) + A);
  CheckAPlusC(dyn(C) + A);
  CheckAPlusC(C + ref(A));
  CheckAPlusC(C + dyn(A));
  CheckAPlusC(ref(C) + ref(A));
  CheckAPlusC(dyn(C) + dyn(A));
  {
    Eigen::Matrix<FunctionalForm, 3, 2> f = A;
    f += C;
    CheckAPlusC(f);
  }
}

TEST_F(FunctionalFormMatrixTest, Subtract) {
  CheckAPlusC(A - C);
  CheckAPlusC(ref(A) - C);
  CheckAPlusC(dyn(A) - C);
  CheckAPlusC(A - ref(C));
  CheckAPlusC(A - dyn(C));
  CheckAPlusC(ref(A) - ref(C));
  CheckAPlusC(dyn(A) - dyn(C));
  CheckAPlusC(C - A);
  CheckAPlusC(ref(C) - A);
  CheckAPlusC(dyn(C) - A);
  CheckAPlusC(C - ref(A));
  CheckAPlusC(C - dyn(A));
  CheckAPlusC(ref(C) - ref(A));
  CheckAPlusC(dyn(C) - dyn(A));
  {
    Eigen::Matrix<FunctionalForm, 3, 2> f = A;
    f -= C;
    CheckAPlusC(f);
  }
}

TEST_F(FunctionalFormMatrixTest, Multiply) {
  CheckATimesScalar(A * 2);
  CheckATimesScalar(ref(A) * 2);
  CheckATimesScalar(dyn(A) * 2);
  CheckATimesScalar(2 * A);
  CheckATimesScalar(2 * ref(A));
  CheckATimesScalar(2 * dyn(A));
  {
    Eigen::Matrix<FunctionalForm, 3, 2> f = A;
    f *= 2;
    CheckATimesScalar(f);
  }
  CheckATimesB(A * B);
  CheckATimesB(ref(A) * B);
  CheckATimesB(dyn(A) * B);
  CheckATimesB(A * ref(B));
  CheckATimesB(A * dyn(B));
  CheckATimesB(ref(A) * ref(B));
  CheckATimesB(dyn(A) * dyn(B));
  CheckATimesCTranspose(A * C.transpose());
  CheckATimesCTranspose(ref(A) * C.transpose());
  CheckATimesCTranspose(dyn(A) * C.transpose());
  CheckATimesCTranspose(A * ref(C.transpose()));
  CheckATimesCTranspose(A * dyn(C.transpose()));
  CheckATimesCTranspose(ref(A) * ref(C.transpose()));
  CheckATimesCTranspose(dyn(A) * dyn(C.transpose()));
  CheckCTimesATranspose(C.transpose() * A);
  CheckCTimesATranspose(C.transpose() * ref(A));
  CheckCTimesATranspose(C.transpose() * dyn(A));
  CheckCTimesATranspose(ref(C.transpose()) * A);
  CheckCTimesATranspose(dyn(C.transpose()) * A);
  CheckCTimesATranspose(ref(C.transpose()) * ref(A));
  CheckCTimesATranspose(dyn(C.transpose()) * dyn(A));
}

TEST_F(FunctionalFormMatrixTest, Divide) {
  CheckADivScalar(A / 2);
  CheckADivScalar(ref(A) / 2);
  CheckADivScalar(dyn(A) / 2);
  {
    Eigen::Matrix<FunctionalForm, 3, 2> f = A;
    f /= 2;
    CheckADivScalar(f);
  }
}

}  // namespace
}  // namespace test
}  // namespace core
}  // namespace drake

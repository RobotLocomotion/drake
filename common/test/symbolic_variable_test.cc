#include <map>
#include <sstream>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "drake/common/symbolic.h"
#include "drake/common/test_utilities/is_memcpy_movable.h"
#include "drake/common/test_utilities/symbolic_test_util.h"

namespace drake {

using test::IsMemcpyMovable;

namespace symbolic {
namespace {

using std::map;
using std::move;
using std::ostringstream;
using std::unordered_map;
using std::unordered_set;
using std::vector;
using test::VarEqual;
using test::VarLess;
using test::VarNotEqual;
using test::VarNotLess;

template <typename T>
size_t get_std_hash(const T& item) {
  return std::hash<T>{}(item);
}

// Provides common variables that are used by the following tests.
class VariableTest : public ::testing::Test {
 protected:
  const Variable x_{"x"};
  const Variable y_{"y"};
  const Variable z_{"z"};
  const Variable w_{"w"};
  Eigen::Matrix<Variable, 2, 2> M_;

  void SetUp() override {
    // clang-format off
      M_ << x_, y_,
            z_, w_;
    // clang-format on
  }
};

TEST_F(VariableTest, GetId) {
  const Variable dummy{};
  const Variable x_prime{"x"};
  EXPECT_TRUE(dummy.is_dummy());
  EXPECT_FALSE(x_.is_dummy());
  EXPECT_FALSE(x_prime.is_dummy());
  EXPECT_NE(x_.get_id(), x_prime.get_id());
}

TEST_F(VariableTest, GetName) {
  const Variable x_prime{"x"};
  EXPECT_EQ(x_.get_name(), x_prime.get_name());
}

TEST_F(VariableTest, MoveCopyPreserveId) {
  Variable x{"x"};
  const size_t x_id{x.get_id()};
  const size_t x_hash{get_std_hash(x)};
  const Variable x_copied{x};
  const Variable x_moved{move(x)};
  EXPECT_EQ(x_id, x_copied.get_id());
  EXPECT_EQ(x_hash, get_std_hash(x_copied));
  EXPECT_EQ(x_id, x_moved.get_id());
  EXPECT_EQ(x_hash, get_std_hash(x_moved));
}

TEST_F(VariableTest, Less) {
  EXPECT_PRED2(VarNotLess, x_, x_);
  EXPECT_PRED2(VarLess, x_, y_);
  EXPECT_PRED2(VarLess, x_, z_);
  EXPECT_PRED2(VarLess, x_, w_);

  EXPECT_PRED2(VarNotLess, y_, x_);
  EXPECT_PRED2(VarNotLess, y_, y_);
  EXPECT_PRED2(VarLess, y_, z_);
  EXPECT_PRED2(VarLess, y_, w_);

  EXPECT_PRED2(VarNotLess, z_, x_);
  EXPECT_PRED2(VarNotLess, z_, y_);
  EXPECT_PRED2(VarNotLess, z_, z_);
  EXPECT_PRED2(VarLess, z_, w_);

  EXPECT_PRED2(VarNotLess, w_, x_);
  EXPECT_PRED2(VarNotLess, w_, y_);
  EXPECT_PRED2(VarNotLess, w_, z_);
  EXPECT_PRED2(VarNotLess, w_, w_);
}

TEST_F(VariableTest, EqualTo) {
  EXPECT_PRED2(VarEqual, x_, x_);
  EXPECT_PRED2(VarNotEqual, x_, y_);
  EXPECT_PRED2(VarNotEqual, x_, z_);
  EXPECT_PRED2(VarNotEqual, x_, w_);

  EXPECT_PRED2(VarNotEqual, y_, x_);
  EXPECT_PRED2(VarEqual, y_, y_);
  EXPECT_PRED2(VarNotEqual, y_, z_);
  EXPECT_PRED2(VarNotEqual, y_, w_);

  EXPECT_PRED2(VarNotEqual, z_, x_);
  EXPECT_PRED2(VarNotEqual, z_, y_);
  EXPECT_PRED2(VarEqual, z_, z_);
  EXPECT_PRED2(VarNotEqual, z_, w_);

  EXPECT_PRED2(VarNotEqual, w_, x_);
  EXPECT_PRED2(VarNotEqual, w_, y_);
  EXPECT_PRED2(VarNotEqual, w_, z_);
  EXPECT_PRED2(VarEqual, w_, w_);
}

TEST_F(VariableTest, ToString) {
  EXPECT_EQ(x_.to_string(), "x");
  EXPECT_EQ(y_.to_string(), "y");
  EXPECT_EQ(z_.to_string(), "z");
  EXPECT_EQ(w_.to_string(), "w");
}

TEST_F(VariableTest, EqualityCheck) {
  // `v₁ == v₂` and `v₁ != v₂` form instances of symbolic::Formula. Inside of
  // EXPECT_{TRUE,FALSE}, they are converted to bool by Formula::Evaluate() with
  // an empty environment. This test checks that we do not have
  // `runtime_error("The following environment does not have an entry for the
  // variable ...")` in the process.
  EXPECT_TRUE(x_ != y_);
  EXPECT_TRUE(x_ != z_);
  EXPECT_FALSE(x_ == y_);
  EXPECT_FALSE(x_ == z_);
}

// This test checks whether Variable is compatible with std::unordered_set.
TEST_F(VariableTest, CompatibleWithUnorderedSet) {
  unordered_set<Variable> uset;
  uset.emplace(x_);
  uset.emplace(y_);
}

// This test checks whether Variable is compatible with std::unordered_map.
TEST_F(VariableTest, CompatibleWithUnorderedMap) {
  unordered_map<Variable, Variable> umap;
  umap.emplace(x_, y_);
}

TEST_F(VariableTest, MapEqual) {
  // Checking equality between two map<Variable, T> invokes `v₁ == v₂`, which
  // will calls Formula::Evaluate(). This test checks that we do not have
  // `runtime_error("The following environment does not have an entry for the
  // variable ...")` in the process.
  const map<Variable, int> m1{{x_, 3}, {y_, 4}};
  const map<Variable, int> m2{{x_, 5}, {y_, 6}};
  const map<Variable, int> m3{{x_, 5}, {z_, 6}};
  const map<Variable, int> m4{{y_, 4}, {x_, 3}};  // same as m1.

  EXPECT_EQ(m1, m1);  // m1 == m1
  EXPECT_NE(m1, m2);
  EXPECT_NE(m1, m3);
  EXPECT_EQ(m1, m4);  // m1 == m4

  EXPECT_NE(m2, m1);
  EXPECT_EQ(m2, m2);  // m2 == m2
  EXPECT_NE(m2, m3);
  EXPECT_NE(m2, m4);

  EXPECT_NE(m3, m1);
  EXPECT_NE(m3, m2);
  EXPECT_EQ(m3, m3);  // m3 == m3
  EXPECT_NE(m3, m4);

  EXPECT_EQ(m4, m1);  // m4 == m1
  EXPECT_NE(m4, m2);
  EXPECT_NE(m4, m3);
  EXPECT_EQ(m4, m4);  // m4 == m4
}

// This test checks whether Variable is compatible with std::vector.
TEST_F(VariableTest, CompatibleWithVector) {
  vector<Variable> vec;
  vec.push_back(x_);
}

TEST_F(VariableTest, EigenVariableMatrix) {
  EXPECT_PRED2(VarEqual, M_(0, 0), x_);
  EXPECT_PRED2(VarEqual, M_(0, 1), y_);
  EXPECT_PRED2(VarEqual, M_(1, 0), z_);
  EXPECT_PRED2(VarEqual, M_(1, 1), w_);
}

TEST_F(VariableTest, EigenVariableMatrixOutput) {
  ostringstream oss1;
  oss1 << M_;

  ostringstream oss2;
  oss2 << "x y"
       << "\n"
       << "z w";

  EXPECT_EQ(oss1.str(), oss2.str());
}

TEST_F(VariableTest, MemcpyKeepsVariableIntact) {
  // We have it to test that a variable with a long name (>16 chars), which is
  // not using SSO (Short String Optimization) internally, is memcpy-movable.
  const Variable long_var("12345678901234567890");
  for (const Variable& var : {x_, y_, z_, w_, long_var}) {
    EXPECT_TRUE(IsMemcpyMovable(var));
  }
}

TEST_F(VariableTest, CheckType) {
  // By default, a symbolic variable has CONTINUOUS type if not specified at
  // construction time.
  const Variable v1("v1");
  EXPECT_EQ(v1.get_type(), Variable::Type::CONTINUOUS);

  // When a type is specified, it should be correctly assigned.
  const Variable v2("v2", Variable::Type::CONTINUOUS);
  const Variable v3("v3", Variable::Type::INTEGER);
  const Variable v4("v4", Variable::Type::BINARY);
  const Variable v5("v5", Variable::Type::BOOLEAN);
  EXPECT_EQ(v2.get_type(), Variable::Type::CONTINUOUS);
  EXPECT_EQ(v3.get_type(), Variable::Type::INTEGER);
  EXPECT_EQ(v4.get_type(), Variable::Type::BINARY);
  EXPECT_EQ(v5.get_type(), Variable::Type::BOOLEAN);

  // Dummy variable gets CONTINUOUS type.
  EXPECT_TRUE(Variable{}.get_type() == Variable::Type::CONTINUOUS);

  // Variables are identified by their IDs. Names and types are not considered
  // in the identification process.
  const Variable v_continuous("v", Variable::Type::CONTINUOUS);
  const Variable v_int("v", Variable::Type::INTEGER);
  EXPECT_FALSE(v_continuous.equal_to(v_int));
}
}  // namespace
}  // namespace symbolic
}  // namespace drake

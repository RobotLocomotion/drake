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

// Tests that default constructor and EIGEN_INITIALIZE_MATRICES_BY_ZERO
// constructor both create the same value.
TEST_F(VariableTest, DefaultConstructors) {
  const Variable v_default;
  const Variable v_zero(0);
  EXPECT_TRUE(v_default.is_dummy());
  EXPECT_TRUE(v_zero.is_dummy());
}

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

TEST_F(VariableTest, MakeVectorVariable) {
  const VectorX<Variable> vec1{
      MakeVectorVariable(2, "x", Variable::Type::CONTINUOUS)};
  const Vector2<Variable> vec2{
      MakeVectorVariable<2>("x", Variable::Type::CONTINUOUS)};
  EXPECT_EQ(vec1.size(), 2);
  EXPECT_EQ(vec1[0].get_name(), "x(0)");
  EXPECT_EQ(vec1[0].get_type(), Variable::Type::CONTINUOUS);
  EXPECT_EQ(vec1[1].get_name(), "x(1)");
  EXPECT_EQ(vec1[1].get_type(), Variable::Type::CONTINUOUS);
  EXPECT_EQ(vec2.size(), 2);
  EXPECT_EQ(vec2[0].get_name(), "x(0)");
  EXPECT_EQ(vec2[0].get_type(), Variable::Type::CONTINUOUS);
  EXPECT_EQ(vec2[1].get_name(), "x(1)");
  EXPECT_EQ(vec2[1].get_type(), Variable::Type::CONTINUOUS);

  const VectorX<Variable> vec3{MakeVectorVariable(2, "x")};
  const VectorX<Variable> vec4{MakeVectorVariable<2>("x")};
  for (int i = 0; i < 2; ++i) {
    EXPECT_EQ(vec3[i].get_type(), Variable::Type::CONTINUOUS);
    EXPECT_EQ(vec4[i].get_type(), Variable::Type::CONTINUOUS);
  }
}

TEST_F(VariableTest, MakeVectorBooleanVariable) {
  const VectorX<Variable> vec1{MakeVectorBooleanVariable(2, "x")};
  const Vector2<Variable> vec2{MakeVectorBooleanVariable<2>("x")};
  EXPECT_EQ(vec1.size(), 2);
  EXPECT_EQ(vec1[0].get_name(), "x(0)");
  EXPECT_EQ(vec1[0].get_type(), Variable::Type::BOOLEAN);
  EXPECT_EQ(vec1[1].get_name(), "x(1)");
  EXPECT_EQ(vec1[1].get_type(), Variable::Type::BOOLEAN);
  EXPECT_EQ(vec2.size(), 2);
  EXPECT_EQ(vec2[0].get_name(), "x(0)");
  EXPECT_EQ(vec2[0].get_type(), Variable::Type::BOOLEAN);
  EXPECT_EQ(vec2[1].get_name(), "x(1)");
  EXPECT_EQ(vec2[1].get_type(), Variable::Type::BOOLEAN);
}

TEST_F(VariableTest, MakeVectorBinaryVariable) {
  const VectorX<Variable> vec1{MakeVectorBinaryVariable(2, "x")};
  const Vector2<Variable> vec2{MakeVectorBinaryVariable<2>("x")};
  EXPECT_EQ(vec1.size(), 2);
  EXPECT_EQ(vec1[0].get_name(), "x(0)");
  EXPECT_EQ(vec1[0].get_type(), Variable::Type::BINARY);
  EXPECT_EQ(vec1[1].get_name(), "x(1)");
  EXPECT_EQ(vec1[1].get_type(), Variable::Type::BINARY);
  EXPECT_EQ(vec2.size(), 2);
  EXPECT_EQ(vec2[0].get_name(), "x(0)");
  EXPECT_EQ(vec2[0].get_type(), Variable::Type::BINARY);
  EXPECT_EQ(vec2[1].get_name(), "x(1)");
  EXPECT_EQ(vec2[1].get_type(), Variable::Type::BINARY);
}

TEST_F(VariableTest, MakeVectorContinuousVariable) {
  const VectorX<Variable> vec1{MakeVectorContinuousVariable(2, "x")};
  const Vector2<Variable> vec2{MakeVectorContinuousVariable<2>("x")};
  EXPECT_EQ(vec1.size(), 2);
  EXPECT_EQ(vec1[0].get_name(), "x(0)");
  EXPECT_EQ(vec1[0].get_type(), Variable::Type::CONTINUOUS);
  EXPECT_EQ(vec1[1].get_name(), "x(1)");
  EXPECT_EQ(vec1[1].get_type(), Variable::Type::CONTINUOUS);
  EXPECT_EQ(vec2.size(), 2);
  EXPECT_EQ(vec2[0].get_name(), "x(0)");
  EXPECT_EQ(vec2[0].get_type(), Variable::Type::CONTINUOUS);
  EXPECT_EQ(vec2[1].get_name(), "x(1)");
  EXPECT_EQ(vec2[1].get_type(), Variable::Type::CONTINUOUS);
}

TEST_F(VariableTest, MakeVectorIntegerVariable) {
  const VectorX<Variable> vec1{MakeVectorIntegerVariable(2, "x")};
  const Vector2<Variable> vec2{MakeVectorIntegerVariable<2>("x")};
  EXPECT_EQ(vec1.size(), 2);
  EXPECT_EQ(vec1[0].get_name(), "x(0)");
  EXPECT_EQ(vec1[0].get_type(), Variable::Type::INTEGER);
  EXPECT_EQ(vec1[1].get_name(), "x(1)");
  EXPECT_EQ(vec1[1].get_type(), Variable::Type::INTEGER);
  EXPECT_EQ(vec2.size(), 2);
  EXPECT_EQ(vec2[0].get_name(), "x(0)");
  EXPECT_EQ(vec2[0].get_type(), Variable::Type::INTEGER);
  EXPECT_EQ(vec2[1].get_name(), "x(1)");
  EXPECT_EQ(vec2[1].get_type(), Variable::Type::INTEGER);
}

TEST_F(VariableTest, MakeMatrixVariable) {
  const MatrixX<Variable> m1{
      MakeMatrixVariable(1, 2, "x", Variable::Type::CONTINUOUS)};
  const auto m2 = MakeMatrixVariable<1, 2>("x", Variable::Type::CONTINUOUS);
  EXPECT_EQ(m1.rows(), 1);
  EXPECT_EQ(m1.cols(), 2);
  EXPECT_EQ(m1(0, 0).get_name(), "x(0, 0)");
  EXPECT_EQ(m1(0, 0).get_type(), Variable::Type::CONTINUOUS);
  EXPECT_EQ(m1(0, 1).get_name(), "x(0, 1)");
  EXPECT_EQ(m1(0, 1).get_type(), Variable::Type::CONTINUOUS);
  EXPECT_EQ(m2.rows(), 1);
  EXPECT_EQ(m2.cols(), 2);
  EXPECT_EQ(m2(0, 0).get_name(), "x(0, 0)");
  EXPECT_EQ(m2(0, 0).get_type(), Variable::Type::CONTINUOUS);
  EXPECT_EQ(m2(0, 1).get_name(), "x(0, 1)");
  EXPECT_EQ(m2(0, 1).get_type(), Variable::Type::CONTINUOUS);

  const MatrixX<Variable> m3{MakeMatrixVariable(1, 2, "x")};
  const MatrixX<Variable> m4{MakeMatrixVariable<1, 2>("x")};
  for (int i = 0; i < 2; ++i) {
    EXPECT_EQ(m3(0, i).get_type(), Variable::Type::CONTINUOUS);
    EXPECT_EQ(m4(0, i).get_type(), Variable::Type::CONTINUOUS);
  }
}

TEST_F(VariableTest, MakeMatrixBooleanVariable) {
  const MatrixX<Variable> m1{MakeMatrixBooleanVariable(1, 2, "x")};
  const auto m2 = MakeMatrixBooleanVariable<1, 2>("x");
  EXPECT_EQ(m1.rows(), 1);
  EXPECT_EQ(m1.cols(), 2);
  EXPECT_EQ(m1(0, 0).get_name(), "x(0, 0)");
  EXPECT_EQ(m1(0, 0).get_type(), Variable::Type::BOOLEAN);
  EXPECT_EQ(m1(0, 1).get_name(), "x(0, 1)");
  EXPECT_EQ(m1(0, 1).get_type(), Variable::Type::BOOLEAN);
  EXPECT_EQ(m2.rows(), 1);
  EXPECT_EQ(m2.cols(), 2);
  EXPECT_EQ(m2(0, 0).get_name(), "x(0, 0)");
  EXPECT_EQ(m2(0, 0).get_type(), Variable::Type::BOOLEAN);
  EXPECT_EQ(m2(0, 1).get_name(), "x(0, 1)");
  EXPECT_EQ(m2(0, 1).get_type(), Variable::Type::BOOLEAN);
}

TEST_F(VariableTest, MakeMatrixBinaryVariable) {
  const MatrixX<Variable> m1{MakeMatrixBinaryVariable(1, 2, "x")};
  const auto m2 = MakeMatrixBinaryVariable<1, 2>("x");
  EXPECT_EQ(m1.rows(), 1);
  EXPECT_EQ(m1.cols(), 2);
  EXPECT_EQ(m1(0, 0).get_name(), "x(0, 0)");
  EXPECT_EQ(m1(0, 0).get_type(), Variable::Type::BINARY);
  EXPECT_EQ(m1(0, 1).get_name(), "x(0, 1)");
  EXPECT_EQ(m1(0, 1).get_type(), Variable::Type::BINARY);
  EXPECT_EQ(m2.rows(), 1);
  EXPECT_EQ(m2.cols(), 2);
  EXPECT_EQ(m2(0, 0).get_name(), "x(0, 0)");
  EXPECT_EQ(m2(0, 0).get_type(), Variable::Type::BINARY);
  EXPECT_EQ(m2(0, 1).get_name(), "x(0, 1)");
  EXPECT_EQ(m2(0, 1).get_type(), Variable::Type::BINARY);
}

TEST_F(VariableTest, MakeMatrixContinuousVariable) {
  const MatrixX<Variable> m1{MakeMatrixContinuousVariable(1, 2, "x")};
  const auto m2 = MakeMatrixContinuousVariable<1, 2>("x");
  EXPECT_EQ(m1.rows(), 1);
  EXPECT_EQ(m1.cols(), 2);
  EXPECT_EQ(m1(0, 0).get_name(), "x(0, 0)");
  EXPECT_EQ(m1(0, 0).get_type(), Variable::Type::CONTINUOUS);
  EXPECT_EQ(m1(0, 1).get_name(), "x(0, 1)");
  EXPECT_EQ(m1(0, 1).get_type(), Variable::Type::CONTINUOUS);
  EXPECT_EQ(m2.rows(), 1);
  EXPECT_EQ(m2.cols(), 2);
  EXPECT_EQ(m2(0, 0).get_name(), "x(0, 0)");
  EXPECT_EQ(m2(0, 0).get_type(), Variable::Type::CONTINUOUS);
  EXPECT_EQ(m2(0, 1).get_name(), "x(0, 1)");
  EXPECT_EQ(m2(0, 1).get_type(), Variable::Type::CONTINUOUS);
}

TEST_F(VariableTest, MakeMatrixIntegerVariable) {
  const MatrixX<Variable> m1{MakeMatrixIntegerVariable(1, 2, "x")};
  const auto m2 = MakeMatrixIntegerVariable<1, 2>("x");
  EXPECT_EQ(m1.rows(), 1);
  EXPECT_EQ(m1.cols(), 2);
  EXPECT_EQ(m1(0, 0).get_name(), "x(0, 0)");
  EXPECT_EQ(m1(0, 0).get_type(), Variable::Type::INTEGER);
  EXPECT_EQ(m1(0, 1).get_name(), "x(0, 1)");
  EXPECT_EQ(m1(0, 1).get_type(), Variable::Type::INTEGER);
  EXPECT_EQ(m2.rows(), 1);
  EXPECT_EQ(m2.cols(), 2);
  EXPECT_EQ(m2(0, 0).get_name(), "x(0, 0)");
  EXPECT_EQ(m2(0, 0).get_type(), Variable::Type::INTEGER);
  EXPECT_EQ(m2(0, 1).get_name(), "x(0, 1)");
  EXPECT_EQ(m2(0, 1).get_type(), Variable::Type::INTEGER);
}

// Shows that a random uniform variable and std::uniform_real_distribution with
// [0, 1) show the same behavior when the same random number generator is
// passed.
TEST_F(VariableTest, RandomUniform) {
  RandomGenerator generator{};
  RandomGenerator generator_copy{generator};

  const Variable v{"v", Variable::Type::RANDOM_UNIFORM};
  const double sample{Expression{v}.Evaluate(&generator)};

  std::uniform_real_distribution<double> d(0.0, 1.0);  // [0, 1).
  const double expected{d(generator_copy)};

  EXPECT_EQ(sample, expected);
}

// Shows that a random Gaussian variable and std::normal_distribution with (mean
// = 0.0, stddev = 1.0) show the same behavior when the same random number
// generator is passed.
TEST_F(VariableTest, RandomGaussian) {
  RandomGenerator generator{};
  RandomGenerator generator_copy{generator};

  const Variable v{"v", Variable::Type::RANDOM_GAUSSIAN};
  const double sample{Expression{v}.Evaluate(&generator)};

  std::normal_distribution<double> d(0.0, 1.0);  // mean = 0, stddev = 1.0.
  const double expected{d(generator_copy)};

  EXPECT_EQ(sample, expected);
}

// Shows that a random exponential variable and std::exponential_distribution
// with lambda = 1.0 show the same behavior when the same random number
// generator is passed.
TEST_F(VariableTest, RandomExponential) {
  RandomGenerator generator{};
  RandomGenerator generator_copy{generator};

  const Variable v{"v", Variable::Type::RANDOM_EXPONENTIAL};
  const double sample{Expression{v}.Evaluate(&generator)};

  std::exponential_distribution<double> d(1.0);  // lambda = 1.0.
  const double expected{d(generator_copy)};

  EXPECT_EQ(sample, expected);
}

}  // namespace
}  // namespace symbolic
}  // namespace drake

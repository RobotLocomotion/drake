#include "drake/common/schema/stochastic.h"

#include <algorithm>
#include <cmath>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/test_utilities/symbolic_test_util.h"
#include "drake/common/yaml/yaml_io.h"

using drake::symbolic::Expression;
using drake::symbolic::test::ExprEqual;
using drake::symbolic::Variable;
using drake::symbolic::Variables;
using drake::yaml::LoadYamlString;
using drake::yaml::SaveYamlString;

namespace drake {
namespace schema {
namespace {

struct DistributionStruct {
  std::vector<DistributionVariant> vec;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(vec));
  }
};

const char* all_variants = R"""(
vec: [ !Deterministic { value: 5.0 },
       !Gaussian { mean: 2.0, stddev: 4.0 },
       !Uniform { min: 1.0, max: 5.0 },
       !UniformDiscrete { values: [1, 1.5, 2] },
       3.2 ]
)""";

const char* floats = "vec: [5.0, 6.1, 7.2]";

void CheckGaussianSymbolic(const Expression& e, double mean, double stddev) {
  const Variables vars{e.GetVariables()};
  ASSERT_EQ(vars.size(), 1);
  const Variable& v{*(vars.begin())};
  EXPECT_EQ(v.get_type(), Variable::Type::RANDOM_GAUSSIAN);
  EXPECT_PRED2(ExprEqual, e, mean + stddev * v);
}

void CheckUniformSymbolic(const Expression& e, double min, double max) {
  const Variables vars{e.GetVariables()};
  ASSERT_EQ(vars.size(), 1);
  const Variable& v{*(vars.begin())};
  EXPECT_EQ(v.get_type(), Variable::Type::RANDOM_UNIFORM);
  EXPECT_PRED2(ExprEqual, e, min + (max - min) * v);
}

void CheckUniformDiscreteSymbolic(
    const Expression& e, std::vector<double> values) {
  const Variables vars{e.GetVariables()};
  ASSERT_EQ(vars.size(), 1);
  const Variable& v{*(vars.begin())};
  EXPECT_EQ(v.get_type(), Variable::Type::RANDOM_UNIFORM);
  // We don't check the structure here; it can be checked more readably by the
  // caller via string comparison.
}

GTEST_TEST(StochasticTest, ScalarTest) {
  const auto variants = LoadYamlString<DistributionStruct>(all_variants);

  RandomGenerator generator;

  const Deterministic& d = std::get<Deterministic>(variants.vec[0]);
  EXPECT_EQ(d.Sample(&generator), 5.0);
  EXPECT_EQ(d.Mean(), 5.0);
  EXPECT_PRED2(ExprEqual, d.ToSymbolic(), 5.0);
  EXPECT_TRUE(IsDeterministic(variants.vec[0]));
  EXPECT_EQ(GetDeterministicValue(variants.vec[0]), 5.0);

  const Gaussian& g = std::get<Gaussian>(variants.vec[1]);
  EXPECT_EQ(g.mean, 2.);
  EXPECT_EQ(g.stddev, 4.);
  EXPECT_TRUE(std::isfinite(d.Sample(&generator)));
  EXPECT_EQ(g.Mean(), 2.);
  CheckGaussianSymbolic(g.ToSymbolic(), g.mean, g.stddev);
  EXPECT_FALSE(IsDeterministic(variants.vec[1]));
  EXPECT_THROW(GetDeterministicValue(variants.vec[1]),
               std::logic_error);

  const Uniform& u = std::get<Uniform>(variants.vec[2]);
  EXPECT_EQ(u.min, 1.);
  EXPECT_EQ(u.max, 5.);
  double uniform_sample = u.Sample(&generator);
  EXPECT_LE(u.min, uniform_sample);
  EXPECT_GE(u.max, uniform_sample);
  EXPECT_EQ(u.Mean(), 3.);
  CheckUniformSymbolic(u.ToSymbolic(), u.min, u.max);
  EXPECT_FALSE(IsDeterministic(variants.vec[2]));
  EXPECT_THROW(GetDeterministicValue(variants.vec[2]),
               std::logic_error);

  const UniformDiscrete& ub = std::get<UniformDiscrete>(variants.vec[3]);
  EXPECT_NE(std::find(ub.values.begin(), ub.values.end(),
                      ub.Sample(&generator)),
            ub.values.end());
  EXPECT_EQ(ub.Mean(), 1.5);
  CheckUniformDiscreteSymbolic(ub.ToSymbolic(), ub.values);
  EXPECT_EQ(ub.ToSymbolic().to_string(),
            "(if ((3 * random_uniform_0) < 1) then 1 else "
            "(if ((3 * random_uniform_0) < 2) then 1.5 else "
            "2))");
  EXPECT_FALSE(IsDeterministic(variants.vec[3]));
  EXPECT_THROW(GetDeterministicValue(variants.vec[3]),
               std::logic_error);

  EXPECT_EQ(std::get<double>(variants.vec[4]), 3.2);
  EXPECT_EQ(Sample(variants.vec[4], &generator), 3.2);
  EXPECT_EQ(Mean(variants.vec[4]), 3.2);
  EXPECT_TRUE(IsDeterministic(variants.vec[4]));
  EXPECT_EQ(GetDeterministicValue(variants.vec[4]), 3.2);

  Eigen::VectorXd vec = Sample(variants.vec, &generator);
  ASSERT_EQ(vec.size(), 5);
  EXPECT_EQ(vec(0), 5.0);
  EXPECT_TRUE(std::isfinite(vec(1)));
  EXPECT_LE(u.min, vec(2));
  EXPECT_GE(u.max, vec(2));
  EXPECT_NE(std::find(ub.values.begin(), ub.values.end(), vec(3)),
            ub.values.end());
  EXPECT_EQ(vec(4), 3.2);

  vec = Mean(variants.vec);
  ASSERT_EQ(vec.size(), 5);
  EXPECT_EQ(vec(0), 5.0);
  EXPECT_EQ(vec(1), 2.0);
  EXPECT_EQ(vec(2), 3.0);
  EXPECT_EQ(vec(3), 1.5);
  EXPECT_EQ(vec(4), 3.2);

  VectorX<Expression> symbolic_vec = ToSymbolic(variants.vec);
  ASSERT_EQ(symbolic_vec.size(), 5);
  EXPECT_PRED2(ExprEqual, symbolic_vec(0), 5.0);
  CheckGaussianSymbolic(symbolic_vec(1), g.mean, g.stddev);
  CheckUniformSymbolic(symbolic_vec(2), u.min, u.max);
  CheckUniformDiscreteSymbolic(symbolic_vec(3), ub.values);
  EXPECT_PRED2(ExprEqual, symbolic_vec(4), 3.2);

  // Confirm that writeback works for every possible type.
  EXPECT_EQ(SaveYamlString(variants, "root"), R"""(root:
  vec:
    - !Deterministic
      value: 5.0
    - !Gaussian
      mean: 2.0
      stddev: 4.0
    - !Uniform
      min: 1.0
      max: 5.0
    - !UniformDiscrete
      values: [1.0, 1.5, 2.0]
    - 3.2
)""");

  // Try loading a value which looks like an ordinary vector.
  const auto parsed_floats = LoadYamlString<DistributionStruct>(floats);
  vec = Sample(parsed_floats.vec, &generator);
  ASSERT_EQ(vec.size(), 3);
  EXPECT_TRUE(CompareMatrices(vec, Eigen::Vector3d(5.0, 6.1, 7.2)));
}

struct DistributionVectorStruct {
  DistributionVectorVariantX vector;
  DistributionVectorVariantX deterministic;
  DistributionVectorVariantX gaussian1;
  DistributionVectorVariant<4> gaussian2;
  DistributionVectorVariantX uniform;
  DistributionVectorVariantX deterministic_scalar;
  DistributionVectorVariantX gaussian_scalar;
  DistributionVectorVariantX uniform_scalar;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(vector));
    a->Visit(DRAKE_NVP(deterministic));
    a->Visit(DRAKE_NVP(gaussian1));
    a->Visit(DRAKE_NVP(gaussian2));
    a->Visit(DRAKE_NVP(uniform));
    a->Visit(DRAKE_NVP(deterministic_scalar));
    a->Visit(DRAKE_NVP(gaussian_scalar));
    a->Visit(DRAKE_NVP(uniform_scalar));
  }
};

const char* vector_variants = R"""(
vector: [1., 2., 3.]
deterministic: !DeterministicVector { value: [3., 4., 5.] }
gaussian1: !GaussianVector { mean: [1.1, 1.2, 1.3], stddev: [0.1, 0.2, 0.3] }
gaussian2: !GaussianVector { mean: [2.1, 2.2, 2.3, 2.4], stddev: [1.0] }
uniform: !UniformVector { min: [10, 20], max: [11, 22] }
deterministic_scalar: !Deterministic { value: 19.5 }
gaussian_scalar: !Gaussian { mean: 5, stddev: 1 }
uniform_scalar: !Uniform { min: 1, max: 2 }
)""";

GTEST_TEST(StochasticTest, VectorTest) {
  const auto variants =
      LoadYamlString<DistributionVectorStruct>(vector_variants);

  RandomGenerator generator;

  EXPECT_TRUE(CompareMatrices(
      ToDistributionVector(variants.vector)->Sample(&generator),
      Eigen::Vector3d(1., 2., 3.)));
  EXPECT_TRUE(CompareMatrices(
      ToDistributionVector(variants.vector)->Mean(),
      Eigen::Vector3d(1., 2., 3.)));
  auto symbolic_vector = ToDistributionVector(variants.vector)->ToSymbolic();
  ASSERT_EQ(symbolic_vector.size(), 3);
  EXPECT_PRED2(ExprEqual, symbolic_vector(0), 1.);
  EXPECT_PRED2(ExprEqual, symbolic_vector(1), 2.);
  EXPECT_PRED2(ExprEqual, symbolic_vector(2), 3.);

  EXPECT_TRUE(CompareMatrices(
      ToDistributionVector(variants.deterministic)->Sample(&generator),
      Eigen::Vector3d(3., 4., 5.)));
  EXPECT_TRUE(CompareMatrices(
      ToDistributionVector(variants.deterministic)->Mean(),
      Eigen::Vector3d(3., 4., 5.)));
  symbolic_vector = ToDistributionVector(variants.deterministic)->ToSymbolic();
  ASSERT_EQ(symbolic_vector.size(), 3);
  EXPECT_PRED2(ExprEqual, symbolic_vector(0), 3.);
  EXPECT_PRED2(ExprEqual, symbolic_vector(1), 4.);
  EXPECT_PRED2(ExprEqual, symbolic_vector(2), 5.);

  EXPECT_EQ(
      ToDistributionVector(variants.gaussian1)->Sample(&generator).size(), 3);
  EXPECT_TRUE(CompareMatrices(
      ToDistributionVector(variants.gaussian1)->Mean(),
      Eigen::Vector3d(1.1, 1.2, 1.3)));
  symbolic_vector = ToDistributionVector(variants.gaussian1)->ToSymbolic();
  ASSERT_EQ(symbolic_vector.size(), 3);
  CheckGaussianSymbolic(symbolic_vector(0), 1.1, 0.1);
  CheckGaussianSymbolic(symbolic_vector(1), 1.2, 0.2);
  CheckGaussianSymbolic(symbolic_vector(2), 1.3, 0.3);

  EXPECT_EQ(
      ToDistributionVector(variants.gaussian2)->Sample(&generator).size(), 4);
  EXPECT_TRUE(CompareMatrices(
      ToDistributionVector(variants.gaussian2)->Mean(),
      Eigen::Vector4d(2.1, 2.2, 2.3, 2.4)));
  symbolic_vector = ToDistributionVector(variants.gaussian2)->ToSymbolic();
  ASSERT_EQ(symbolic_vector.size(), 4);
  CheckGaussianSymbolic(symbolic_vector(0), 2.1, 1.0);
  CheckGaussianSymbolic(symbolic_vector(1), 2.2, 1.0);
  CheckGaussianSymbolic(symbolic_vector(2), 2.3, 1.0);
  CheckGaussianSymbolic(symbolic_vector(3), 2.4, 1.0);

  EXPECT_TRUE(IsDeterministic(variants.vector));
  EXPECT_TRUE(CompareMatrices(
      GetDeterministicValue(variants.vector),
      Eigen::Vector3d(1., 2., 3.)));

  EXPECT_TRUE(IsDeterministic(variants.deterministic));
  EXPECT_TRUE(CompareMatrices(
      GetDeterministicValue(variants.deterministic),
      Eigen::Vector3d(3., 4., 5.)));

  EXPECT_FALSE(IsDeterministic(variants.gaussian1));
  EXPECT_THROW(GetDeterministicValue(variants.gaussian1),
               std::logic_error);

  EXPECT_FALSE(IsDeterministic(variants.gaussian2));
  EXPECT_THROW(GetDeterministicValue(variants.gaussian2),
               std::logic_error);

  EXPECT_FALSE(IsDeterministic(variants.uniform));
  EXPECT_THROW(GetDeterministicValue(variants.uniform),
               std::logic_error);

  EXPECT_TRUE(IsDeterministic(variants.deterministic_scalar));
  EXPECT_TRUE(CompareMatrices(
      GetDeterministicValue(variants.deterministic_scalar),
      Vector1d(19.5)));

  EXPECT_FALSE(IsDeterministic(variants.gaussian_scalar));
  EXPECT_THROW(GetDeterministicValue(variants.gaussian_scalar),
               std::logic_error);

  EXPECT_FALSE(IsDeterministic(variants.uniform_scalar));
  EXPECT_THROW(GetDeterministicValue(variants.uniform_scalar),
               std::logic_error);

  Eigen::VectorXd uniform =
      ToDistributionVector(variants.uniform)->Sample(&generator);
  ASSERT_EQ(uniform.size(), 2);
  EXPECT_LE(10, uniform(0));
  EXPECT_GE(11, uniform(0));
  EXPECT_LE(20, uniform(1));
  EXPECT_GE(22, uniform(1));
  EXPECT_TRUE(CompareMatrices(
      ToDistributionVector(variants.uniform)->Mean(),
      Eigen::Vector2d(10.5, 21.0)));
  symbolic_vector = ToDistributionVector(variants.uniform)->ToSymbolic();
  ASSERT_EQ(symbolic_vector.size(), 2);
  CheckUniformSymbolic(symbolic_vector(0), 10, 11);
  CheckUniformSymbolic(symbolic_vector(1), 20, 22);

  Eigen::VectorXd deterministic_scalar =
        ToDistributionVector(variants.deterministic_scalar)->Sample(&generator);
  ASSERT_EQ(deterministic_scalar.size(), 1);
  EXPECT_EQ(deterministic_scalar(0), 19.5);
  EXPECT_TRUE(CompareMatrices(
      ToDistributionVector(variants.deterministic_scalar)->Mean(),
      Vector1d(19.5)));

  Eigen::VectorXd gaussian_scalar =
      ToDistributionVector(variants.gaussian_scalar)->Sample(&generator);
  ASSERT_EQ(gaussian_scalar.size(), 1);
  EXPECT_TRUE(CompareMatrices(
      ToDistributionVector(variants.gaussian_scalar)->Mean(),
      Vector1d(5.0)));

  Eigen::VectorXd uniform_scalar =
      ToDistributionVector(variants.uniform_scalar)->Sample(&generator);
  ASSERT_EQ(uniform_scalar.size(), 1);
  EXPECT_LE(1, uniform_scalar(0));
  EXPECT_GE(2, uniform_scalar(0));
  EXPECT_TRUE(CompareMatrices(
      ToDistributionVector(variants.uniform_scalar)->Mean(),
      Vector1d(1.5)));

  // Confirm that writeback works for every possible type.
  EXPECT_EQ(SaveYamlString(variants, "root"), R"""(root:
  vector: [1.0, 2.0, 3.0]
  deterministic: !DeterministicVector
    value: [3.0, 4.0, 5.0]
  gaussian1: !GaussianVector
    mean: [1.1, 1.2, 1.3]
    stddev: [0.1, 0.2, 0.3]
  gaussian2: !GaussianVector
    mean: [2.1, 2.2, 2.3, 2.4]
    stddev: [1.0]
  uniform: !UniformVector
    min: [10.0, 20.0]
    max: [11.0, 22.0]
  deterministic_scalar: !Deterministic
    value: 19.5
  gaussian_scalar: !Gaussian
    mean: 5.0
    stddev: 1.0
  uniform_scalar: !Uniform
    min: 1.0
    max: 2.0
)""");
}

// Check the special cases of IsDeterministic for zero-size ranges.
GTEST_TEST(StochasticTest, ZeroSizeRandomRanges) {
  DistributionVectorVariantX gaussian = GaussianVector<Eigen::Dynamic>(
      Eigen::VectorXd::Constant(3, 4.5), Eigen::VectorXd::Constant(3, 0.0));
  DistributionVectorVariantX uniform = UniformVector<Eigen::Dynamic>(
      Eigen::VectorXd::Constant(2, 1.5), Eigen::VectorXd::Constant(2, 1.5));
  DistributionVectorVariantX gaussian_scalar = Gaussian(4.5, 0.0);
  DistributionVectorVariantX uniform_scalar = Uniform(1.5, 1.5);

  EXPECT_TRUE(IsDeterministic(gaussian));
  EXPECT_TRUE(CompareMatrices(
      GetDeterministicValue(gaussian),
      Eigen::VectorXd::Constant(3, 4.5)));

  EXPECT_TRUE(IsDeterministic(uniform));
  EXPECT_TRUE(CompareMatrices(
      GetDeterministicValue(uniform),
      Eigen::VectorXd::Constant(2, 1.5)));

  EXPECT_TRUE(IsDeterministic(gaussian_scalar));
  EXPECT_TRUE(CompareMatrices(
      GetDeterministicValue(gaussian_scalar),
      Eigen::VectorXd::Constant(1, 4.5)));

  EXPECT_TRUE(IsDeterministic(uniform_scalar));
  EXPECT_TRUE(CompareMatrices(
      GetDeterministicValue(uniform_scalar),
      Eigen::VectorXd::Constant(1, 1.5)));
}

struct FixedSize2 {
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(value));
  }

  DistributionVectorVariant<2> value;
};

GTEST_TEST(StochasticTest, IncorrectVectorTest) {
  const char* const input = R"""(
value: !Deterministic { value: 2.0 }
)""";
  DRAKE_EXPECT_THROWS_MESSAGE(
      LoadYamlString<FixedSize2>(input),
      ".*unsupported type tag !Deterministic while selecting a variant<> entry"
      " for std::variant<.*> value.*");
}

}  // namespace
}  // namespace schema
}  // namespace drake

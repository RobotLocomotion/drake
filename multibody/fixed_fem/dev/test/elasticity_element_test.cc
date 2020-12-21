#include "drake/multibody/fem/dev/elasticity_element.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/fem/dev/fem_state.h"
#include "drake/multibody/fem/dev/linear_constitutive_model.h"
#include "drake/multibody/fem/dev/linear_simplex_element.h"
#include "drake/multibody/fem/dev/quadrature.h"

namespace drake {
namespace multibody {
namespace fem {
constexpr int kNaturalDim = 3;
constexpr int kSpatialDim = 3;
constexpr int kProblemDim = 3;
constexpr int kQuadratureOrder = 1;
constexpr int kNumQuads = 1;
constexpr int kNumVertices = 4;
constexpr int kDof = kSpatialDim * kNumVertices;
constexpr double kDummyDensity = 1.23;
const ElementIndex kDummyElementIndex(0);

class ElasticityElementTest : public ::testing::Test {
 protected:
  using QuadratureType =
      SimplexGaussianQuadrature<AutoDiffXd, kQuadratureOrder, kSpatialDim>;
  using ShapeType = LinearSimplexElement<AutoDiffXd, kNaturalDim>;
  void SetUp() override {
    SetupElement();
    SetupState();
  }

  void SetupElement() {
    std::vector<NodeIndex> node_indices = {NodeIndex(0), NodeIndex(1),
                                           NodeIndex(2), NodeIndex(3)};
    linear_elasticity_ =
        std::make_unique<LinearConstitutiveModel<AutoDiffXd>>(1, 0.25);
    MatrixX<AutoDiffXd> reference_positions = get_reference_positions();
    elasticity_element_ = std::make_unique<
        ElasticityElement<AutoDiffXd, ShapeType, QuadratureType>>(
        kDummyElementIndex, node_indices, AutoDiffXd(kDummyDensity),
        std::move(linear_elasticity_), reference_positions);
  }

  void SetupState() {
    // Set arbitrary node positions and the gradient.
    Eigen::Matrix<double, kDof, 1> x;
    x << 0.18, 0.63, 0.54, 0.13, 0.92, 0.17, 0.03, 0.86, 0.85, 0.25, 0.53, 0.67;
    const Eigen::Matrix<double, kDof, Eigen::Dynamic> gradient =
        MatrixX<double>::Identity(kDof, kDof);
    const VectorX<AutoDiffXd> x_autodiff =
        math::initializeAutoDiffGivenGradientMatrix(x, gradient);
    const VectorX<AutoDiffXd> v = VectorX<AutoDiffXd>::Zero(kDof);
    state_ = std::make_unique<FemState<AutoDiffXd>>(x_autodiff, v);
    // Set up the element cache.
    std::vector<std::unique_ptr<ElementCacheEntry<AutoDiffXd>>> cache;
    cache.emplace_back(elasticity_element_->MakeElementCacheEntry());
    state_->ResetElementCache(std::move(cache));
  }

  // Set an arbitrary reference positions such that the tetrahedron is not
  // inverted.
  Matrix3X<AutoDiffXd> get_reference_positions() const {
    Matrix3X<AutoDiffXd> Q(kSpatialDim, kNumVertices);
    Q << -0.10, 0.90, 0.02, 0.10, 1.33, 0.23, 0.04, 0.01, 0.20, 0.03, 2.31,
        -0.12;
    return Q;
  }

  // Calculates the negative elastic force at state_.
  VectorX<AutoDiffXd> CalcNegativeElasticForce() const {
    VectorX<AutoDiffXd> neg_force(kDof);
    elasticity_element_->CalcNegativeElasticForce(*state_, &neg_force);
    return neg_force;
  }

  std::unique_ptr<ElasticityElement<AutoDiffXd, ShapeType, QuadratureType>>
      elasticity_element_;
  std::unique_ptr<LinearConstitutiveModel<AutoDiffXd>> linear_elasticity_;
  std::unique_ptr<FemState<AutoDiffXd>> state_;
};

namespace {
TEST_F(ElasticityElementTest, Basic) {
  EXPECT_EQ(elasticity_element_->num_nodes(), kNumVertices);
  EXPECT_EQ(elasticity_element_->num_quadrature_points(), kNumQuads);
  EXPECT_EQ(elasticity_element_->solution_dimension(), kProblemDim);
}

TEST_F(ElasticityElementTest, ElasticForceIsNegativeEnergyDerivative) {
  AutoDiffXd energy = elasticity_element_->CalcElasticEnergy(*state_);
  VectorX<AutoDiffXd> neg_force = CalcNegativeElasticForce();
  EXPECT_TRUE(CompareMatrices(energy.derivatives(), neg_force,
                              std::numeric_limits<double>::epsilon()));
  // TODO(xuchenhan-tri) Modify this to account for damping forces and inertia
  // terms.
  VectorX<AutoDiffXd> residual(kDof);
  elasticity_element_->CalcResidual(*state_, &residual);
  EXPECT_TRUE(CompareMatrices(residual, neg_force));
}

// TODO(xuchenhan-tri): Add TEST_F as needed for inertia and damping terms.

TEST_F(ElasticityElementTest, StiffnessMatrixIsNegativeElasticForceDerivative) {
  VectorX<AutoDiffXd> neg_force = CalcNegativeElasticForce();
  MatrixX<AutoDiffXd> stiffness_matrix(kDof, kDof);
  elasticity_element_->CalcStiffnessMatrix(*state_, &stiffness_matrix);
  for (int i = 0; i < kDof; ++i) {
    EXPECT_TRUE(CompareMatrices(neg_force(i).derivatives().transpose(),
                                stiffness_matrix.row(i),
                                std::numeric_limits<double>::epsilon()));
  }
  // TODO(xuchenhan-tri) Modify this to account for damping force derivatives
  // and inertia terms.
  MatrixX<AutoDiffXd> tangent_matrix(kDof, kDof);
  elasticity_element_->CalcTangentMatrix(*state_, &tangent_matrix);
  EXPECT_TRUE(CompareMatrices(tangent_matrix, stiffness_matrix));
}

}  // namespace
}  // namespace fem
}  // namespace multibody
}  // namespace drake
